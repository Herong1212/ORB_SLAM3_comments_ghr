/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue> // 先进先出，一端进入，一端移出
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h> // 桥梁库，用来在 ROS 的图像消息和 OpenCV 的 cv::Mat 图像格式之间进行转换。
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

float shift = 0;
using namespace std;

// 用于处理和缓存 IMU 数据。核心功能：将接收到的 IMU 消息存储起来，供后续同步和处理使用。
class ImuGrabber
{
public:
  // 默认构造函数
  // {}：这是构造函数的实现部分。在这里，它是空的，表示当实例化一个 ImuGrabber 对象时，什么都不会发生，也就是没有任何初始化逻辑。
  // 虽然目前它没有任何操作，但为类的对象实例化提供了一个入口，在需要时可以在构造函数中添加初始化逻辑。
  ImuGrabber() {};

  // 这是 ROS 订阅 IMU 话题的回调函数。当 IMU 消息到达时，
  // GrabImu 函数会被调用，它将接收到的IMU消息放入 imuBuf 队列中，并使用 mBufMutex 锁住队列，确保线程安全。
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  // 一个队列，用于缓存接收到的 IMU 传感器数据消息，每个元素通常是包含IMU数据的ROS消息。ROS通过回调函数接收到IMU数据后，数据会被存放在这个队列中。
  queue<sensor_msgs::ImuConstPtr> imuBuf;

  // 用于保证对 IMU 数据队列 imuBuf 的线程安全访问。由于 IMU 数据和图像数据是分别在不同线程中接收的，线程安全的访问对于保证数据一致性至关重要。
  std::mutex mBufMutex;
};

class ImageGrabber
{
public:
  // 构造函数：
  // ImuGrabber *pImuGb：指向ImuGrabber实例的指针，用于从ImuGrabber获取IMU数据，与图像数据进行同步。
  // const bool bClahe：布尔值，表示是否对图像数据执行CLAHE（对比度限制自适应直方图均衡化）。CLAHE是一种图像增强技术，用于改善图像的对比度。
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe) : mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe) {}
  // ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe) : mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe) {}

  // 成员函数：
  // 这是ROS订阅图像话题的回调函数。当图像消息到达时，该函数将图像存入 img0Buf 队列中，并使用 mBufMutex 锁住队列，确保线程安全。
  // void GrabImage(const sensor_msgs::ImageConstPtr &msg);

  void GrabImageRgb(const sensor_msgs::ImageConstPtr &msg);   // notice1：处理 RGB 图像数据
  void GrabImageDepth(const sensor_msgs::ImageConstPtr &msg); // notice2：处理深度图像数据

  // * 将 ROS 图像消息转换为 OpenCV 格式的 cv::Mat。这是图像处理的核心函数，用于将传入的图像数据转换为可以处理的OpenCV矩阵格式。
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

  // 这是 RGB 图像和 Depth 图与 IMU 数据同步的核心函数。它会从 imgRgbBuf, imgDepthBuf 和 imuBuf 中取出数据，
  // 并根据它们的时间戳进行对齐和同步处理。同步后的数据会传递给 ORB-SLAM3 系统用于 SLAM 处理。
  void SyncWithImu();

  // TODO 主要成员如下：
  // 缓存图像数据的队列，存储接收到的图像消息。与IMU数据队列类似，图像数据会被存储到该队列中，供同步和后续处理使用。
  // queue<sensor_msgs::ImageConstPtr> img0Buf;
  queue<sensor_msgs::ImageConstPtr> imgRgbBuf, imgDepthBuf;
  // 用于保护 img0Buf 队列，保证对图像数据的线程安全访问。
  std::mutex mBufMutexRGB, mBufMutexDepth; // RGB 和 Depth 各一个锁🔒

  ORB_SLAM3::System *mpSLAM; // 指向 ORB-SLAM3 系统的指针，ImageGrabber 将接收的图像和 IMU 数据传递给 SLAM 系统。
  // ? 为什么要把这个放在这里面？
  // 答：为了实现模块化设计和数据同步。这种结构能提高代码的可维护性和扩展性，使得每个类都专注于其核心功能。
  // 答：mpImuGb 使得 ImageGrabber 能够访问 ImuGrabber 中的 IMU 数据。
  ImuGrabber *mpImuGb;

  // const bool do_rectify;      // ? 这个参数好像不用吧，335L相机自带矫正功能吧？
  // cv::Mat M1l, M2l, M1r, M2r; // ? 目前还不知道这是干嘛的！

  // 布尔值，表示是否对图像数据执行CLAHE（对比度限制自适应直方图均衡化）。CLAHE是一种图像增强技术，用于改善图像的对比度。
  const bool mbClahe;
  // CLAHE算法的实例，用于对图像执行自适应直方图均衡化操作。
  // CLAHE可以提高图像的对比度，尤其是在光照条件不佳的情况下。该成员只在 bClahe 参数为 true 时被调用。
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
  // 解决乱码方法一
  // setlocale(LC_ALL, "");

  ros::init(argc, argv, "RGBD_Inertial"); // 初始化ros节点，RGBD_Inertial 是节点名称
  ros::NodeHandle n("~");                 // 节点句柄，与ROS系统进行交互的主要接口。
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;

  if (argc < 3 || argc > 4)
  {
    // [do_equalize] --- 表示是否需要图像均衡处理（可选，通常是布尔值）。
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 RGBD_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;

    ros::shutdown(); // 用于关闭ros节点。在发现命令行参数错误后，程序调用 ros::shutdown() 来停止ROS的运行环境，确保程序不再继续运行。

    return 1; // 返回1表示程序异常退出，常用于错误处理。在这种情况下，由于用户提供的参数不正确，程序不会继续执行而是直接退出。
  }

  if (argc == 4) // 都四个参数，可以选择是否启用图像均衡化或其他处理。
  {
    std::string sbEqual(argv[3]); // 将命令行传入的第四个参数（即argv[3]）转换为一个 std::string 类型的变量 sbEqual。

    if (sbEqual == "true")
    {
      bEqual = true; // 表示启用了该功能（比如图像均衡化功能）。
    }
    else if (sbEqual == "false")
    {
      bEqual = false;
    }
    else
    {
      cerr << "Invalid value for do_equalize. Expected 'true' or 'false'." << endl;
      return 1;
    }
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  // 创建 ORB-SLAM3 系统
  cout << "Creating System..." << endl;
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);

  ImuGrabber imugb; // 创建一个 ImuGrabber 对象，用于处理 IMU 数据
  ImageGrabber igb(&SLAM, &imugb, bEqual); // TODO 创建 ImageGrabber 对象，处理图像数据，并和IMU数据进行同步

  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    cout << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  shift = fsSettings["IMU.shift"]; // ? 这里还是不知道干嘛的

  // Maximum delay, 5 seconds
  // * 下面是如何在 ORB-SLAM3 的ROS接口中订阅 IMU 数据和图像数据，并通过多线程的方式将它们同步。
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  // 1000 --- 消息缓存的队列大小，确保即使处理速度稍慢，也不会丢失过多数据
  // 回调函数：&ImuGrabber::GrabImu，这是一个成员函数，负责接收 IMU 消息并存储。
  // 回调对象：&imugb，这是一个 ImuGrabber 的实例，IMU 数据会传递给这个对象进行处理。
  ros::Subscriber sub_image_rgb = n.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImageRgb, &igb);
  ros::Subscriber sub_image_depth = n.subscribe("/camera/depth/image_raw", 100, &ImageGrabber::GrabImageDepth, &igb);
  // 回调函数：&ImageGrabber::GrabImage，这是一个成员函数，负责接收图像消息。
  // 回调对象：&igb，这是 ImageGrabber 的实例，用于处理图像数据并和 IMU 数据进行同步。

  // 创建了一个新的线程，用于在后台进行 IMU 和图像数据的同步处理。
  // 同步线程会从 IMU 和图像缓存中读取数据，并进行时间戳匹配，确保图像和 IMU 数据能够同步处理。
  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  // 用于进入事件循环，等待 ROS 消息的到来并调用相应的回调函数。在这里，它确保 IMU 和图像消息的接收和处理是连续进行的。
  // ros::spin() 会保持程序运行，直到用户中断（通常是按下 Ctrl+C 结束进程）。
  ros::spin();

  /* 可能的改进点：
      异常处理：在多线程数据同步时，可能会出现数据丢失或者同步失败的情况，通常需要加入异常处理机制来确保系统的鲁棒性。

      时间戳对齐：图像和IMU数据的时间戳需要精确对齐，SyncWithImu 函数中应包含相应的逻辑。可能需要考虑对 IMU 和图像的采样频率差异进行处理，以确保系统的准确性。

      性能优化：如果系统需要处理高频率的数据，可以根据需要调整队列大小（目前设定为1000）以及优化多线程模型。 */

  return 0;
}

// notice1：处理 RGB 图像数据
void ImageGrabber::GrabImageRgb(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRGB.lock();

  // 如果缓冲区不为空，弹出最旧的图像
  if (!imgRgbBuf.empty())
    imgRgbBuf.pop();

  // 将最新的 RGB 图像消息存入缓冲区
  imgRgbBuf.push(img_msg);

  mBufMutexRGB.unlock();

  /* 改进建议：
      当前设计只存储最新的图像数据，每次有新数据时会丢弃旧数据。如果需要保留历史图像数据，可以考虑使用固定大小的环形缓冲区，或者设置多个图像缓冲区来保存不同时间段的图像。 */
}

// notice2：处理深度图像数据
void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexDepth.lock();

  if (!imgDepthBuf.empty())
  {
    imgDepthBuf.pop();
  }
  imgDepthBuf.push(img_msg);

  mBufMutexDepth.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;

  try
  {
    // 通过cv_bridge::toCvShare() 将 ROS 的图像消息转换为 OpenCV 图像，指定编码格式为 MONO8（即灰度图像），如果不指定，函数将使用原始图像的编码格式。
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);

    // 获取转换后的 OpenCV 图像
    // cv::Mat image = cv_ptr->image;

    // 在 OpenCV 中显示图像
    // cv::imshow("View", image);
    // cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // 转换成功后，代码会检查图像的类型是否为0。在OpenCV中，类型0表示CV_8UC1，也就是单通道8位灰度图。如果图像类型符合这个条件，函数会返回克隆的图像。
  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    // 如果图像类型不是0，代码会输出一条错误信息（"Error type"），但仍然返回克隆的图像。这种做法虽然保证了函数有返回值，但没有明确处理非灰度图像的情况。
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }

  /* 改进建议：
     类型检查逻辑：目前仅检查是否为CV_8UC1类型的灰度图，如果有其他类型的图像输入，可以考虑添加更详细的处理逻辑，而不是直接返回错误信息。
     返回值处理：如果图像类型不匹配，函数可以考虑返回一个空的图像（例如cv::Mat()），以便调用者明确知道转换失败。 */
}

// notice：实现了一个 RGB 图像和 Depth 图像与 IMU（惯性测量单元）数据的同步处理函数，主要用于将图像帧和 IMU 数据同步后一起传递给 SLAM（同时定位与地图构建）系统进行处理。
void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01; // 最大时间差，单位为秒，表示 RGB 和深度图像之间的最大时间差。

  while (1)
  {
    cv::Mat imRGB, imDepth;          // 定义 RGB 和深度图像矩阵
    double tImRGB = 0, tImDepth = 0; // 存储当前 RGB 和深度图像的时间戳

    // step1：检查 RGB 图像、深度图像和 IMU 数据缓冲区是否都有数据
    if (!imgRgbBuf.empty() && !imgDepthBuf.empty() && !mpImuGb->imuBuf.empty())
    {
      tImRGB = imgRgbBuf.front()->header.stamp.toSec();     // 获取 RGB 图像 时间戳
      tImDepth = imgDepthBuf.front()->header.stamp.toSec(); // 获取 深度图像 时间戳
    }

    // step2：锁定图像缓存区，保证线程安全
    this->mBufMutexRGB.lock(); // notice：这里的 this 也可以省略，但使用 this 是好的编程习惯
    while ((tImDepth - tImRGB) > maxTimeDiff && imgRgbBuf.size() > 1)
    {
      imgRgbBuf.pop();                                  // 弹出时间戳过旧的 RGB图像
      tImRGB = imgRgbBuf.front()->header.stamp.toSec(); // 更新 RGB时间戳
    }
    this->mBufMutexRGB.unlock();

    mBufMutexDepth.lock();
    while ((tImDepth - tImRGB) > maxTimeDiff && imgDepthBuf.size() > 1)
    {
      imgDepthBuf.pop();                                    // 弹出时间戳过旧的 深度图像
      tImDepth = imgDepthBuf.front()->header.stamp.toSec(); // 更新 深度时间戳
    }
    this->mBufMutexDepth.unlock();

    // step3：检查时间差是否超过最大限制
    // 锁定深度图像缓冲区，如果 RGB 图像时间戳与深度图像时间戳的差超过最大时间差，则弹出深度图像，直到时间戳符合要求。
    if ((tImRGB - tImDepth) > maxTimeDiff || (tImDepth - tImRGB) > maxTimeDiff)
    {
      // std::cout << "big time difference" << std::endl; // 调试用
      continue; // 跳过本次循环
    }

    // step4：检查 RGB 图像时间戳是否晚于最新的 IMU 数据时间戳
    // 如果当前 RGB 图像的时间戳晚于最新的 IMU 数据的时间戳，则继续等待。
    if (tImRGB > mpImuGb->imuBuf.back()->header.stamp.toSec())
      continue;

    // step5：从缓冲区中获取图像
    // 从 RGB 图像缓冲区中获取图像
    this->mBufMutexRGB.lock();
    imRGB = GetImage(imgRgbBuf.front()); // 获取缓冲区的第一很图像，就是把缓冲区最先进来的那第一帧图像转换为 cv::Mat 图像格式
    imgRgbBuf.pop();                     // 弹出已经获取的图像
    this->mBufMutexDepth.unlock();

    // 从深度图像缓冲区中获取图像
    this->mBufMutexDepth.lock();
    imDepth = GetImage(imgDepthBuf.front());
    imgDepthBuf.pop();
    this->mBufMutexDepth.unlock();

    // step6：提取 IMU 数据
    vector<ORB_SLAM3::IMU::Point> vImuMeas; // 用于存储 IMU 测量数据

    mpImuGb->mBufMutex.lock();
    if (!mpImuGb->imuBuf.empty())
    {
      // 从 IMU 缓冲区加载测量数据
      vImuMeas.clear();

      while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImRGB + shift)
      {
        double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
        cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
        cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
        mpImuGb->imuBuf.pop(); // 弹出已经处理过的 IMU 数据
      }
    }
    mpImuGb->mBufMutex.unlock();

    // step7：可选的图像增强处理
    if (mbClahe)
    {
      mClahe->apply(imRGB, imRGB);
      mClahe->apply(imDepth, imDepth);
    }

    // if(do_rectify)
    // {
    //   cv::remap(imRgb,imRgb,M1l,M2l,cv::INTER_LINEAR);
    //   cv::remap(imDepth,imDepth,M1r,M2r,cv::INTER_LINEAR);
    // }

    // step8：缩放图像到指定大小
    cv::Size dsize = cv::Size(960, 540);
    cv::Mat rgb_resize;
    cv::resize(imRGB, rgb_resize, dsize, 0, 0, cv::INTER_NEAREST);
    cv::Mat depth_resize;
    cv::resize(imDepth, depth_resize, dsize, 0, 0, cv::INTER_NEAREST);

    // step9：调用 SLAM 系统进行处理
    mpSLAM->TrackRGBD(rgb_resize, depth_resize, tImRGB, vImuMeas); // 调用 mpSLAM->TrackRGBD 方法进行 RGB-D SLAM 跟踪处理。

    // step10：延时，避免过度占用 CPU 资源
    // 每次循环结束后，调用 std::this_thread::sleep_for(tSleep) 进行短暂的休眠（1毫秒），以防止 CPU 资源的过度占用。
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();

  imuBuf.push(imu_msg);

  mBufMutex.unlock();

  return;
}
