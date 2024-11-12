/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

    class Viewer;
    class FrameDrawer;
    class Atlas;
    class LocalMapping;
    class LoopClosing;
    class System;
    class Settings;

    class Tracking
    {
        // todo 测试用的嗷
        // public:
        //     void TestA(); 

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas,
                     KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, Settings *settings, const string &_nameSeq = std::string());

            ~Tracking(); // f_track_stats.close();

            // Parse the config file
            // 提取配置文件数据
            bool ParseCamParamFile(cv::FileStorage &fSettings);
            bool ParseORBParamFile(cv::FileStorage &fSettings);
            bool ParseIMUParamFile(cv::FileStorage &fSettings);

            // Preprocess the input and call Track(). Extract features and performs stereo matching.
            // 输入图像输出位姿 Tcw，这个函数很重要！
            // cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename); // 但是视频里都是这种形式👈
            Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename);
            Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename);
            Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
            // Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp);

            // 放置 IMU 数据
            void GrabImuData(const IMU::Point &imuMeasurement);

            // 设置线程指针
            void SetLocalMapper(LocalMapping *pLocalMapper);
            void SetLoopClosing(LoopClosing *pLoopClosing);
            void SetViewer(Viewer *pViewer);
            void SetStepByStep(bool bSet);

            bool GetStepByStep();

            // Load new settings
            // The focal lenght should be similar or scale prediction will fail when projecting points
            // ! 更换新的标定参数，未使用
            void ChangeCalibration(const string &strSettingPath);

            // Use this function if you have deactivated local mapping and you only want to localize the camera.
            // 设置是否仅定位模式还是 SLAM 模式
            void InformOnlyTracking(const bool &flag);

            // LocalMapping 中更新了关键帧的位姿后，更新普通帧的位姿，通过 IMU 积分更新速度，LocalMapping 中初始化 IMU 使用
            void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame);
            KeyFrame *GetLastKeyFrame()
            {
                return mpLastKeyFrame;
        }

        // 新建地图
        void CreateMapInAtlas();
        // std::mutex mMutexTracks;

        void NewDataset();       // 更新数据集
        int GetNumberDataset();  // 获得数据集总数
        int GetMatchesInliers(); // 获得匹配内点总数

        // DEBUG
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder = "");
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap);

        float GetImageScale();

#ifdef REGISTER_LOOP
        void RequestStop();
        bool isStopped();
        void Release();
        bool stopRequested();
#endif

    public:
        // Tracking states
        // 跟踪状态
        enum eTrackingState
        {
            SYSTEM_NOT_READY = -1, // 系统没有准备好的状态，一般就是在启动后，加载配置文件和词典文件时候的状态
            NO_IMAGES_YET = 0,     // 当前无图像
            NOT_INITIALIZED = 1,   // 有图像但是还没有完成初始化
            OK = 2,                // 正常跟踪状态
            RECENTLY_LOST = 3,     // 新增的状态，如果是IMU 模式，当前地图中的 KF ＞ 10，且丢失时间 ＜ 5 秒；如果是纯视觉模式，则没有该状态
            LOST = 4,              // IMU 模式，当前帧跟丢超过 5秒；纯视觉模式即重定位失败
            OK_KLT = 5             // 未使用
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;
        Frame mLastFrame; // 跟踪成功后，保存当前帧数据

        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        // 单目初始化用到的一些变量
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<Sophus::SE3f> mlRelativeFramePoses;
        list<KeyFrame *> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // frames with estimated pose
        int mTrackedFr;
        bool mbStep;

        // True if local mapping is deactivated and we are performing only localization
        // true 表示仅定位模式，此时局部建图线程和闭环线程关闭
        bool mbOnlyTracking;

        void Reset(bool bLocMap = false);
        void ResetActiveMap(bool bLocMap = false);

        float mMeanTrack;
        bool mbInitWith3KFs;
        double t0;    // time-stamp of first read frame
        double t0vis; // time-stamp of first inserted keyframe
        double t0IMU; // time-stamp of IMU initialization
        bool mFastInit = false;

        // 获取局部地图点
        vector<MapPoint *> GetLocalMapMPS();

        bool mbWriteStats;

#ifdef REGISTER_TIMES
        void LocalMapStats2File();
        void TrackStats2File();
        void PrintTimeStats();

        vector<double> vdRectStereo_ms;
        vector<double> vdResizeImage_ms;
        vector<double> vdORBExtract_ms;
        vector<double> vdStereoMatch_ms;
        vector<double> vdIMUInteg_ms;
        vector<double> vdPosePred_ms;
        vector<double> vdLMTrack_ms;
        vector<double> vdNewKF_ms;
        vector<double> vdTrackTotal_ms;
#endif

    protected:
        // Main tracking function. It is independent of the input sensor.
        void Track(); // NOTE 主要的跟踪函数，大BOSS！

        // Map initialization for stereo and RGB-D
        void StereoInitialization();

        // Map initialization for monocular
        void MonocularInitialization();

        void CreateNewMapPoints();                          // ? 创建新的地图点，好像不是在这里用的，找到了 --> LocalMapping.cc
        cv::Mat ComputeF12(KeyFrame *pKF1, KeyFrame *pKF2); // ? 计算两个关键帧之间的 F 矩阵，好像不是在这里用的，找到了 --> GeometricTools.cc
        void CreateInitialMapMonocular();                   // 单目模式下创建初始化地图

        void CheckReplacedInLastFrame(); // 检查上一帧中的地图点是否需要被替换
        bool TrackReferenceKeyFrame();   // NOTICE1：参考关键帧跟踪
        void UpdateLastFrame();          // 更新上一帧位姿，在上一帧中生成临时地图点
        bool TrackWithMotionModel();     // NOTICE2：恒速模型跟踪
        bool PredictStateIMU();          // NOTE 用 IMU 预测位姿

        bool Relocalization(); // 重定位

        void UpdateLocalMap();       // 更新局部地图
        void UpdateLocalPoints();    // 更新局部地图点
        void UpdateLocalKeyFrames(); // 更新局部地图里的局部关键帧

        bool TrackLocalMap();     // NOTICE4：局部地图跟踪
        bool TrackLocalMap_old(); // ? 未定义，即好像不是在这里用的，没找到，应该是没实现！
        void SearchLocalPoints(); // 搜索局部地图点

        bool NeedNewKeyFrame();   // 判读是否需要插入关键帧，这个跟 ORB-SLAM2 的差别比较大！
        void CreateNewKeyFrame(); // 创建关键帧，跟 ORB-SLAM2 的差不多！

        // Perform preintegration from last frame
        void PreintegrateIMU(); // NOTE IMU 预积分

        // Reset IMU biases and compute frame velocity
        // 重置并重新计算 IMU 偏置，未使用呦~
        void ComputeGyroBias(const vector<Frame *> &vpFs, float &bwx, float &bwy, float &bwz);
        void ComputeVelocitiesAccBias(const vector<Frame *> &vpFs, float &bax, float &bay, float &baz);

        void ResetFrameIMU();

        bool mbMapUpdated;

        // Imu preintegration from last frame
        IMU::Preintegrated *mpImuPreintegratedFromLastKF;

        // Queue of IMU measurements between frames
        std::list<IMU::Point> mlQueueImuData; // 存放两帧之间的 IMU 数据

        // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
        std::vector<IMU::Point> mvImuFromLastFrame;
        std::mutex mMutexImuQueue; // 互斥锁，用来保护 mlQueueImuData

        // Imu calibration parameters
        IMU::Calib *mpImuCalib; // IMU 标定参数

        // Last Bias Estimation (at keyframe creation)
        IMU::Bias mLastBias;

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        // Other Thread Pointers
        // 其他线程的指针
        LocalMapping *mpLocalMapper;
        LoopClosing *mpLoopClosing;

        // ORB 特征提取器
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight; // 单目默认用的 Left；双目跟踪是默认用 Left，初始化时 Left 和 Right 都会用。
        ORBextractor *mpIniORBextractor;                        // 初始化时用的提取器，因为初始化时提取的不一样

        // BoW
        ORBVocabulary *mpORBVocabulary;
        KeyFrameDatabase *mpKeyFrameDB; // 关键帧数据库

        // Initalizat>ion (only for monocular)
        bool mbReadyToInitializate;
        bool mbSetInit;

        // Local Map
        KeyFrame *mpReferenceKF;
        std::vector<KeyFrame *> mvpLocalKeyFrames;
        std::vector<MapPoint *> mvpLocalMapPoints;

        // System
        System *mpSystem;

        // Drawers
        Viewer *mpViewer;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        bool bStepByStep;

        // Atlas
        Atlas *mpAtlas;

        // Calibration matrix
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        cv::Mat mDistCoef;
        float mbf;
        float mImageScale;

        float mImuFreq;
        double mImuPer;
        bool mInsertKFsLost;

        // New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        int mnFirstImuFrameId;
        // 经过多少帧后可以重置 IMU，一般设置为和频率相同，对应时间为 1s
        int mnFramesToResetIMU;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth; // 近远点阈值，基线的倍数

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor; // RGB-D 尺度缩放因子，不同相机不一样，一般是 1000 或 5000

        // Current matches in frame
        int mnMatchesInliers; // 当前帧匹配内点数，通过判断该值来判断是否跟踪成功

        // Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        double mTimeStampLost;
        double time_recently_lost; // 默认为 5s

        unsigned int mnFirstFrameId;
        unsigned int mnInitialFrameId;
        unsigned int mnLastInitFrameId;

        bool mbCreatedMap;

        // Motion Model
        bool mbVelocity{false};
        Sophus::SE3f mVelocity; // 恒速模型的速度，通过位姿增量获得，或者 IMU 积分得到

        // Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<MapPoint *> mlpTemporalPoints; // 存放临时地图点

        // int nMapChangeIndex;

        int mnNumDataset;

        ofstream f_track_stats;

        ofstream f_track_times;
        double mTime_PreIntIMU; // 这些都是调试用的
        double mTime_PosePred;
        double mTime_LocalMapTrack;
        double mTime_NewKF_Dec;

        GeometricCamera *mpCamera, *mpCamera2; // 相机类

        int initID, lastID;

        Sophus::SE3f mTlr;

        void newParameterLoader(Settings *settings);

#ifdef REGISTER_LOOP
        bool Stop();

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;
#endif

    public:
        cv::Mat mImRight;
    };

} // namespace ORB_SLAM

#endif // TRACKING_H
