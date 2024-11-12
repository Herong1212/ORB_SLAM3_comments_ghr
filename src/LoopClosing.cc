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

#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"

#include <mutex>
#include <thread>

namespace ORB_SLAM3
{

    /**
     * @brief 回环线程构造函数
     * @param pAtlas atlas
     * @param pKFDB 关键帧词典数据库
     * @param bFixScale 除了单目都为 true，包括 imu 单目
     * @param bActiveLC 开启回环，默认是开启的
     */
    LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC) : mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
                                                                                                                                      mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
                                                                                                                                      mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
                                                                                                                                      mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0), mbActiveLC(bActiveLC)
    {
        // 连续性阈值
        mnCovisibilityConsistencyTh = 3;
        mpLastCurrentKF = static_cast<KeyFrame *>(NULL);

#ifdef REGISTER_TIMES

        vdDataQuery_ms.clear();
        vdEstSim3_ms.clear();
        vdPRTotal_ms.clear();

        vdMergeMaps_ms.clear();
        vdWeldingBA_ms.clear();
        vdMergeOptEss_ms.clear();
        vdMergeTotal_ms.clear();
        vnMergeKFs.clear();
        vnMergeMPs.clear();
        nMerges = 0;

        vdLoopFusion_ms.clear();
        vdLoopOptEss_ms.clear();
        vdLoopTotal_ms.clear();
        vnLoopKFs.clear();
        nLoop = 0;

        vdGBA_ms.clear();
        vdUpdateMap_ms.clear();
        vdFGBATotal_ms.clear();
        vnGBAKFs.clear();
        vnGBAMPs.clear();
        nFGBA_exec = 0;
        nFGBA_abort = 0;

#endif

        mstrFolderSubTraj = "SubTrajectories/";
        mnNumCorrection = 0;
        mnCorrectionGBA = 0;
    }

    // notice1：将传入的 Tracking 类的对象指针 pTracker 赋值给 mpTracker，即将外部的 Tracking 对象指针与 LoopClosing 类内部的 mpTracker 关联起来。
    void LoopClosing::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

    // ps：测试用
    // void LoopClosing::Test()
    // {
    //     mpTracker->TestA();
    //     std::cout << "This is a test!" << std::endl;
    // }

    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    // NOTE：回环线程主函数！！！
    void LoopClosing::Run()
    {
        mbFinished = false;

        // 线程主循环
        while (1)
        {

            // NEW LOOP AND MERGE DETECTION ALGORITHM
            //----------------------------

            // * Loopclosing 中的关键帧是 LocalMapping 发送过来的，LocalMapping 是 Tracking 中发过来的
            // 在 LocalMapping 中通过 InsertKeyFrame 将关键帧插入闭环检测队列 mlpLoopKeyFrameQueue

            // Step 1 查看闭环检测队列 mlpLoopKeyFrameQueue 中有没有关键帧进来
            if (CheckNewKeyFrames()) // 检测是否有关键帧插入
            {
                // 这部分后续未使用
                if (mpLastCurrentKF)
                {
                    mpLastCurrentKF->mvpLoopCandKFs.clear();
                    mpLastCurrentKF->mvpMergeCandKFs.clear();
                }

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartPR = std::chrono::steady_clock::now();
#endif
                // Step 2 检测有没有共同区域
                bool bFindedRegion = NewDetectCommonRegions();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndPR = std::chrono::steady_clock::now();

                double timePRTotal = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndPR - time_StartPR).count();
                vdPRTotal_ms.push_back(timePRTotal);
#endif
                // if(NewDetectCommonRegions()) 也可以这样用
                if (bFindedRegion)
                {
                    // case 1：如果检测到共同区域发生在【当前帧】和【非活跃地图】中，则执行【地图融合】操作
                    if (mbMergeDetected)
                    {
                        // 在 imu 没有初始化就放弃融合
                        if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD) &&
                            (!mpCurrentKF->GetMap()->isImuInitialized()))
                        {
                            cout << "IMU is not initilized, merge is aborted" << endl;
                        }
                        // IMU 初始化完成，继续执行下面操作
                        else
                        {
                            // 拿到融合帧在自己地图所在坐标系(w2)下的位姿
                            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
                            g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
                            // 拿到当前帧在自己地图所在坐标系(w1)下的位姿
                            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
                            g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
                            // 根据共同区域检测时的 Sim3 结果得到当前帧在 w2 下的位姿
                            // mg2oMergeSlw 里存放的是融合候选关键帧所在的世界坐标系 w2 到当前帧的 Sim3 位姿
                            // l = c , w2 是融合候选关键帧所在的世界坐标系
                            g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                            // 这个没有用到 : 融合帧在 w1 下的位姿
                            g2o::Sim3 gSw1m = mg2oMergeSlw;

                            // 记录焊接变换(Sim3) T_w2_w1 , 这个量实际是两个地图坐标系的关系 T_w2_w1 = T_w2_c * T_c_w1
                            mSold_new = (gSw2c * gScw1);

                            // 如果是 imu 模式
                            if (mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                            {
                                cout << "Merge check transformation with IMU" << endl;
                                // 如果尺度变换太大, 认为累积误差较大，则放弃融合
                                if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1)
                                {
                                    mpMergeLastCurrentKF->SetErase();
                                    mpMergeMatchedKF->SetErase();
                                    mnMergeNumCoincidences = 0;
                                    mvpMergeMatchedMPs.clear();
                                    mvpMergeMPs.clear();
                                    mnMergeNumNotFound = 0;
                                    mbMergeDetected = false;
                                    Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                    continue;
                                }
                                // If inertial, force only yaw
                                // 如果是 imu 模式并且完成了初始化, 强制将焊接变换的 roll 和 pitch 设为 0
                                // 通过物理约束来保证两个坐标轴都是水平的
                                if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD) &&
                                    mpCurrentKF->GetMap()->GetIniertialBA1())
                                {
                                    Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                    phi(0) = 0;
                                    phi(1) = 0;
                                    mSold_new = g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
                                }
                            }

                            // 这个变量没有用到
                            mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                            // 更新 mg2oMergeScw
                            mg2oMergeScw = mg2oMergeSlw;

                            // mpTracker->SetStepByStep(true);

                            // 检测到地图合并
                            Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();

                            nMerges += 1;
#endif
                            // TODO UNCOMMENT
                            if (mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD)
                                // 如果是【imu】模式, 则开启 Visual-Inertial Map Merging
                                MergeLocal2();
                            else
                                // 如果是【纯视觉】模式, 则开启 Visual-Welding Map Merging
                                MergeLocal();

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();

                            double timeMergeTotal = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMerge - time_StartMerge).count();
                            vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

                            Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                        }
                        // 记录时间戳
                        vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                        vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                        // 标记 Place recognition 结果为地图融合
                        vnPR_TypeRecogn.push_back(1);

                        // 重置所有融合相关变量
                        mpMergeLastCurrentKF->SetErase();
                        mpMergeMatchedKF->SetErase();
                        mnMergeNumCoincidences = 0;
                        mvpMergeMatchedMPs.clear();
                        mvpMergeMPs.clear();
                        mnMergeNumNotFound = 0;
                        mbMergeDetected = false;

                        // 重置所有回环相关变量, 说明对与当前帧同时有回环和融合的情况只进行融合
                        if (mbLoopDetected)
                        {
                            mpLoopLastCurrentKF->SetErase();
                            mpLoopMatchedKF->SetErase();
                            mnLoopNumCoincidences = 0;
                            mvpLoopMatchedMPs.clear();
                            mvpLoopMPs.clear();
                            mnLoopNumNotFound = 0;
                            mbLoopDetected = false;
                        }
                    }

                    // case 2：如果检测到共同区域发生在【当前帧】和【活跃地图】中, 则执行【闭环】操作
                    if (mbLoopDetected)
                    {
                        // 标记时间戳
                        bool bGoodLoop = true;
                        vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                        vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                        vnPR_TypeRecogn.push_back(0);

                        Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);
                        
                        // 更新 mg2oLoopScw
                        mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];

                        // 如果是带 imu 的模式则做下判断，纯视觉跳过
                        if (mpCurrentKF->GetMap()->IsInertial())
                        {
                            // 拿到当前关键帧相对于世界坐标系的位姿
                            Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
                            g2o::Sim3 g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);

                            // mg2oLoopScw 是通过回环检测的 Sim3 计算出的回环矫正后的当前关键帧的初始位姿, Twc是当前关键帧回环矫正前的位姿.
                            // g2oSww_new 可以理解为correction
                            g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

                            // 拿到 roll ,pitch ,yaw
                            Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                            cout << "phi = " << phi.transpose() << endl;
                            // 这里算是通过imu重力方向验证回环结果, 如果pitch或roll角度偏差稍微有一点大,则回环失败. 对yaw容忍比较大(20度)
                            if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f && fabs(phi(2)) < 0.349f)
                            {
                                // 如果是 imu 模式
                                if (mpCurrentKF->GetMap()->IsInertial())
                                {
                                    // If inertial, force only yaw
                                    // 如果是 imu 模式,强制将焊接变换的的 roll 和 pitch 设为 0
                                    if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2())
                                    {
                                        phi(0) = 0;
                                        phi(1) = 0;
                                        g2oSww_new = g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                                        mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                                    }
                                }
                            }
                            else
                            {
                                cout << "BAD LOOP!!!" << endl;
                                bGoodLoop = false;
                            }
                        }

                        if (bGoodLoop)
                        {

                            mvpLoopMapPoints = mvpLoopMPs;

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();

                            nLoop += 1;

#endif
                            // 开启回环矫正及位姿图优化
                            CorrectLoop(); // step1.4：执行【回环矫正】
#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();

                            double timeLoopTotal = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLoop - time_StartLoop).count();
                            vdLoopTotal_ms.push_back(timeLoopTotal);
#endif

                            mnNumCorrection += 1;
                        }

                        // 重置所有的回环变量
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }
                }
                mpLastCurrentKF = mpCurrentKF;
            }

            // 查看是否有外部线程请求复位当前线程
            ResetIfRequested();

            // 查看外部线程是否有终止当前线程的请求, 如果有的话就跳出这个线程的主函数的主循环
            if (CheckFinish())
                break;

            usleep(5000);
            // std::this_thread::sleep_for(std::chrono::milliseconds(5)); 在 ORB-SLAM3 中这样用的
        }
        // 运行到这里说明有外部线程请求终止当前线程，在这个函数中执行终止当前线程的一些操作
        SetFinish();
    }

    // todo 插入关键帧
    void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        // 第一个关键帧不参与回环检测过程，因为其定义了整个地图的坐标系
        if (pKF->mnId != 0)
            mlpLoopKeyFrameQueue.push_back(pKF);
    }

    // todo 查看有没有未处理的关键帧
    bool LoopClosing::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        return (!mlpLoopKeyFrameQueue.empty());
    }

    // todo 作用：检测有没有共同区域 --- 对应于 ORB-SLAM2 里的函数 DetectLoop
    /**
     * @brief 检测有没有共同区域，包括检测回环和融合匹配, sim3 计算, 验证
     * @return true
     * @return false
     */
    bool LoopClosing::NewDetectCommonRegions()
    {
        // 如果一开始就不做回环的话这里就退出了，这个线程也就名存实亡了
        if (!mbActiveLC)
            return false;

        {
            // Step 1 从队列中取出一个关键帧, 作为当前检测共同区域的关键帧
            unique_lock<mutex> lock(mMutexLoopQueue);

            // 从队列头开始取（元素），也就是先取早进来的关键帧
            mpCurrentKF = mlpLoopKeyFrameQueue.front();

            // 取出关键帧后从队列里弹出该关键帧
            mlpLoopKeyFrameQueue.pop_front();

            // 设置当前关键帧不要在优化的过程中被删除
            mpCurrentKF->SetNotErase();
            mpCurrentKF->mbCurrentPlaceRecognition = true;

            // 当前关键帧对应的地图
            mpLastMap = mpCurrentKF->GetMap();
        }

        // Step 2 在某些情况下不进行共同区域检测

        // 1.【imu】模式下还没经过【第二阶段】初始化则不考虑
        if (mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
        {
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetErase();
            return false;
        }

        // 2.【双目】模式下且当前地图关键帧数【少于5】则不考虑
        if (mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) // 12
        {
            // cout << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetErase();
            return false;
        }

        // 3.当前地图关键帧【少于12】则不进行检测
        if (mpLastMap->GetAllKeyFrames().size() < 12)
        {
            // cout << "LoopClousure: Stereo KF inserted without check, map is small: " << mpCurrentKF->mnId << endl;
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetErase();
            return false;
        }

        // cout << "LoopClousure: Checking KF: " << mpCurrentKF->mnId << endl;

        // Check the last candidates with geometric validation
        //  Step 3 基于前一帧的历史信息判断是否进行【时序几何校验】, 注意这里是基于共视几何校验失败才会运行的代码, 阅读代码的时候可以先看后面

        //  Loop candidates
        bool bLoopDetectedInKF = false; // 某次时序验证是否成功
        bool bCheckSpatial = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_1 = std::chrono::steady_clock::now();
#endif
        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // note：实际执行顺序 3，时序几何校验。---注意，当顺序 2 没完成时才执行，若顺序 2 完成任务，则不执行顺序 3
        // Step 3.1 【回环】的【时序几何校验】。注意初始化时 mnLoopNumCoincidences = 0, 所以可以先跳过看后面
        // 如果成功验证总次数【大于 0 】
        if (mnLoopNumCoincidences > 0) // 最初是不执行的，所以第一次可以先跳过
        {
            bCheckSpatial = true;

            // 通过上一关键帧的信息, 计算新的当前帧的 sim3 变换矩阵。 原理为：Tcl = Tcw * Twl
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);

            // * 修改为👇：修改上面原来的代码（有错误）---> 下面的代码（书上的）
            // cv::Mat mTcl = mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse();
            // g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)), Converter::toVector3d(mTcl.rowRange(0, 3).col(3)), 1.0);

            g2o::Sim3 gScw = gScl * mg2oLoopSlw;
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;

            // 通过把候选帧局部窗口内的地图点向新进来的关键帧投影，来验证回环检测结果, 并优化 Sim3 位姿
            bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);

            // case1：如果找到共同区域，则表示时序验证成功一次
            if (bCommonRegion)
            {
                // 标记时序检验--成功--一次
                bLoopDetectedInKF = true;

                // 累计正检验的成功次数
                mnLoopNumCoincidences++;

                // 不再参与新的回环检测
                mpLoopLastCurrentKF->SetErase();

                // 将当前关键帧作为上次关键帧
                mpLoopLastCurrentKF = mpCurrentKF;

                mg2oLoopSlw = gScw; // 记录当前优化的结果为{last T_cw}即为 T_lw

                // 记录匹配到的点
                mvpLoopMatchedMPs = vpMatchedMPs;

                // 如果验证数【大于等于 3 】则为成功回环
                mbLoopDetected = mnLoopNumCoincidences >= 3;

                // 记录失败的时序校验数为0
                mnLoopNumNotFound = 0;
                //! 这里的条件反了,不过对功能没什么影响,只是打印信息
                if (!mbLoopDetected)
                {
                    cout << "PR: Loop detected with Reffine Sim3" << endl;
                }
            }
            // case2：如果没找到共同区域，则认为时序验证失败一次，连续两次失败则认为整个融合检测失败
            else
            {
                // 当前时序验证失败
                bLoopDetectedInKF = false;
                // 递增失败的时序验证次数
                mnLoopNumNotFound++;
                // 若果连续两帧时序验证失败则整个回环检测失败
                if (mnLoopNumNotFound >= 2)
                {
                    // 失败后标记重置一些信息
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                }
            }
        }

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // ps：（融合帧的）时序几何校验实现代码👇

        // Merge candidates
        bool bMergeDetectedInKF = false; // 某次时序验证是否成功

        // Step 3.2 【融合】的【时序几何校验】。注意初始化时 mnMergeNumCoincidences = 0, 所以可以先跳过看后面
        // mnMergeNumCoincidences 表示成功校验总次数，跟上面一样，初始化为 0，最初也是不执行的
        // 会先经过后面的共视几何校验，如果【小于3】，会进到如下判断开始---时序几何校验---
        if (mnMergeNumCoincidences > 0)
        {
            // 通过上一关键帧的信息, 计算新的当前帧的 sim3 变换矩阵。 原理为：Tcl = Tcw * Twl
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);

            // * 修改为👇：修改上面原来的代码（有错误）---> 下面的代码（书上的）
            // cv::Mat mTcl = mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse();
            // g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)), Converter::toVector3d(mTcl.rowRange(0, 3).col(3)), 1.0);

            // mg2oMergeSlw 中的 w 指的是融合候选关键帧的世界坐标系
            g2o::Sim3 gScw = gScl * mg2oMergeSlw;
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;

            // 通过把候选帧局部窗口内的地图点向新进来的关键帧投影，来验证回环检测结果, 并优化 Sim3 位姿
            bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);

            // case1：如果找到共同区域，则表示时序验证成功一次
            if (bCommonRegion)
            {
                // 标记时序检验--成功--一次
                bMergeDetectedInKF = true;

                // 成功验证的总次数 +1
                mnMergeNumCoincidences++;

                // 不再参与新的回环检测
                mpMergeLastCurrentKF->SetErase();
                mpMergeLastCurrentKF = mpCurrentKF;
                mg2oMergeSlw = gScw;
                mvpMergeMatchedMPs = vpMatchedMPs;

                // 如果验证数【大于等于 3 】，则为成功
                mbMergeDetected = mnMergeNumCoincidences >= 3;
            }
            // case2：如果没找到共同区域，则认为时序验证失败一次，连续两次失败则认为整个融合检测失败
            else
            {
                // 没有找到融合
                mbMergeDetected = false;
                // 当前时序验证失败
                bMergeDetectedInKF = false;

                // 递增失败的时序验证次数
                mnMergeNumNotFound++;

                // 如果【连续两帧】时序验证都失败，则整个融合检测失败
                if (mnMergeNumNotFound >= 2)
                {
                    // 失败后标记重置一些信息
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                }
            }
        }

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_1 = std::chrono::steady_clock::now();

        double timeEstSim3 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndEstSim3_1 - time_StartEstSim3_1).count();
#endif
        // Step 3.3 若校验成功则把当前帧添加进数据库，且返回 true 表示找到共同区域
        // 注意初始化时 mbMergeDetected = mbLoopDetected = false，也不执行
        if (mbMergeDetected || mbLoopDetected)
        {
#ifdef REGISTER_TIMES
            vdEstSim3_ms.push_back(timeEstSim3);
#endif
            // f_time_pr << "Geo" << "" << timeGeoKF_ms.count() << endl;
            mpKeyFrameDB->add(mpCurrentKF);
            return true;
        }

        // TODO: This is only necessary if we use a minimun score for pick the best candidates

        // 这句并没有使用，作者把 orbslam2 里面通过 minScore 作为阀值筛选候选帧的策略抛弃了
        const vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

        /* float minScore = 1;
        for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++)
        {
            KeyFrame *pKF vpConnectedKeyFrames[i];
            if (pKF->isBad())
                continue;
            const DBoW2 : BowVector &BowVec = pKF->mBowVec;

            float score = mpORBVocabulary->score(CurrentBowvec, Bowvec);
            if (score < minScore)
                minScore = score;
        } */
        // ----------------------------------------

        // note：实际执行顺序 1
        // Step 4 若当前关键帧没有被检测到回环或融合, 则分别通过 bow 拿到当前帧最好的【三个】回环候选帧和融合候选帧
        vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
        // cout << "LC:bMergeDetectedInKF: " << bMergeDetectedInKF << " bLoopDetectedInKF: " << bLoopDetectedInKF << endl;

        if (!bMergeDetectedInKF || !bLoopDetectedInKF)
        {
            // Search in BoW
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartQuery = std::chrono::steady_clock::now();
#endif
            // 分别找到 3个 最佳的候选帧, 回环候选帧放在 vpLoopBowCand 中，融合候选帧放在 vpMergeBowCand 中
            mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3); // note：很重要，寻找闭环候选关键帧和融合候选关键帧！
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndQuery = std::chrono::steady_clock::now();

            double timeDataQuery = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndQuery - time_StartQuery).count();
            vdDataQuery_ms.push_back(timeDataQuery);
#endif
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_2 = std::chrono::steady_clock::now();
#endif
        // Check the BoW candidates if the geometric candidate list is empty
        // note：实际执行顺序 2
        // Step 4.1 若当前关键帧没有被检测到【回环】, 并且候选帧数量不为 0 ，则对回环候选帧进行论文中【第8页的2-5步】
        // case1：Loop candidates
        if (!bLoopDetectedInKF && !vpLoopBowCand.empty())
        {
            // mnLoopNumCoincidences 是成功几何验证的帧数，【超过 3 】就认为最终验证成功（mbLoopDetected=true），不超过则继续进行时序验证
            // mpLoopMatchedKF 最后成功匹配的候选关键帧
            mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs); // note：也很重要！
        }

        // Step 4.2 若当前关键帧没有被检测到【融合】，并且候选帧数量不为 0 ，则对融合候选帧进行论文中【第8页的2-5步】
        // case2：Merge candidates
        if (!bMergeDetectedInKF && !vpMergeBowCand.empty())
        {
            // mnLoopNumCoincidences 是成功几何验证的帧数，【超过 3】 就认为最终验证成功（mbMergeDetected = true），不超过继续进行时序验证
            mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_2 = std::chrono::steady_clock::now();

        timeEstSim3 += std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndEstSim3_2 - time_StartEstSim3_2).count();
        vdEstSim3_ms.push_back(timeEstSim3);
#endif
        // Step 5 根据结果确定有没有检测到共同区域
        // 把当前帧添加到关键帧数据库中
        mpKeyFrameDB->add(mpCurrentKF);

        // note：实际执行顺序 4，只要满足以下任意一种条件，就返回 true（如果检测到一种类型共同区域 ——> 返回 true）
        if (mbMergeDetected || mbLoopDetected)
        {
            return true;
        }

        // 如果没检测到则把当前关键帧 erase (不参与后续共同区域检测)
        mpCurrentKF->SetErase();

        // 标记当前关键帧不是当前在进行共同区域检测的帧
        mpCurrentKF->mbCurrentPlaceRecognition = false;

        return false;
    }

    // todo 对新进来的关键帧进行时序几何验证，同时继续优化之前估计的 Tcw
    /**
     * @brief 对新进来的关键帧进行时序几何验证，同时继续优化之前估计的 Tcw
     *
     * @param[in] pCurrentKF 当前关键帧
     * @param[in] pMatchedKF 候选帧
     * @param[out] gScw 世界坐标系在验证帧下的Sim3
     * @param[out] nNumProjMatches 记录匹配点的数量
     * @param[out] vpMPs 候选帧窗口内所有的地图点
     * @param[out] vpMatchedMPs 候选帧窗口内所有被匹配到的点
     * @return true 时序几何验证成功
     * @return false 时序几何验证失败
     */
    bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                     std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs)
    {
        set<MapPoint *> spAlreadyMatchedMPs;
        // 1. 重新选点
        // 把候选帧局部窗口内的地图点投向新进来的当前关键帧,看是否有足够的匹配点
        // TODO 清空vpMPs是不是有些多余？经过验证并不多余，点数有一定概率有轻微变化，但不大，这里可以做优化
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

        int nProjMatches = 30;
        int nProjOptMatches = 50;
        int nProjMatchesRep = 100;

        // 2.点数如果不符合返回false
        if (nNumProjMatches >= nProjMatches)
        {
            // Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ", Verbose::VERBOSITY_DEBUG);
            //  3.1 求得gScm 为OptimizeSim3接口准备数据
            Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
            g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
            g2o::Sim3 gScm = gScw * gSwm;
            Eigen::Matrix<double, 7, 7> mHessian7x7;

            // 单目情况下不锁定尺度
            bool bFixedScale = mbFixScale; // TODO CHECK; Solo para el monocular inertial
            // 如果是imu模式且未完成初始化,不锁定尺度
            if (mpTracker->mSensor == System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale = false;
            // 3.2 优化gScm，mp固定
            int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

            // Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);
            //  若匹配的数量大于一定的数目
            if (numOptMatches > nProjOptMatches)
            {
                //! bug, 以下gScw_estimation应该通过上述sim3优化后的位姿来更新。以下mScw应该改为 gscm * gswm^-1
                g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));

                // 再次通过优化后的Sim3搜索匹配点
                nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
                // 若果大于期望数目,接受这个结果
                if (nNumProjMatches >= nProjMatchesRep)
                {
                    gScw = gScw_estimation;
                    // 验证成功
                    return true;
                }
            }
        }
        // 验证失败
        return false;
    }

    // notice：是否检测到检测共同区域
    /**
     * @brief 实现论文 第8页的2-5步 中的一部分功能(对后面新进来的关键帧的验证没有放在这个函数里进行)
     * 1. 构造局部窗口
     * 2. Ransac 得到 Scm 的初始值
     * 3. guided matching refinement
     * 4. 利用地图中的共视关键帧验证(共视几何校验)
     *
     * @param[in] vpBowCand bow 给出的一些候选关键帧
     * @param[out] pMatchedKF2 最后成功匹配的候选关键帧
     * @param[out] pLastCurrentKF 用于记录当前关键帧为上一个关键帧(后续若仍需要时序几何校验需要记录此信息)
     * @param[out] g2oScw 候选关键帧世界坐标系到当前关键帧的 Sim3 变换
     * @param[out] nNumCoincidences 成功几何验证的帧数，【超过 3 】就认为几何验证成功，不超过继续进行时序验证
     * @param[out] vpMPs  所有地图点
     * @param[out] vpMatchedMPs 成功匹配的地图点
     * @return true 检测到一个合格的共同区域
     * @return false 没检测到一个合格的共同区域
     */
    bool LoopClosing::DetectCommonRegionsFromBoW(
        std::vector<KeyFrame *> &vpBowCand, KeyFrame *&pMatchedKF2, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
        int &nNumCoincidences, std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs)
    {
        // 一些后面会使用的阀值
        int nBoWMatches = 20;     // 最低 bow 匹配特征点数
        int nBoWInliers = 15;     // RANSAC 最低的匹配点数
        int nSim3Inliers = 20;    // sim3 最低内点数
        int nProjMatches = 50;    // 通过投影得到的匹配点数量最低阀值
        int nProjOptMatches = 80; // 通过更小的半径，更严的距离搜索到的匹配点数量

        /* if (mpTracker->mSensor == System : IMU_MONOCULAR || mpTracker->mSensor == System : IMU_STEREO)
        {
            nBoWMatches = 20;
            nBoWInliers = 15;
            nSim3Inliers = 20;
            nProjMatches = 35;
            nProjoptMatches = 50;
        } */

        // step 1. 获取当前帧的共视帧(在共同区域检测中应该避免当前关键帧的共视关键帧中)
        set<KeyFrame *> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

        // change 定义最佳共视关键帧的数量， 0.4 版本这里为 5
        int nNumCovisibles = 10;

        ORBmatcher matcherBoW(0.9, true); // 用于search by bow
        ORBmatcher matcher(0.75, true);   // 用与seach by projection
        int nNumGuideMatching = 0;        // 没有用到

        // Varibles to select the best numbe
        // 一些用于统计最优数据的变量,我们最后返回的是最佳的一个关键帧(几何校验匹配数最高的)
        KeyFrame *pBestMatchedKF;
        int nBestMatchesReproj = 0;
        int nBestNumCoindicendes = 0;
        g2o::Sim3 g2oBestScw;
        std::vector<MapPoint *> vpBestMapPoints;
        std::vector<MapPoint *> vpBestMatchedMapPoints;

        // bow 中候选关键帧的数量
        int numCandidates = vpBowCand.size();

        // 这三个变量是作者为了后面打印观察记录的信息，可以忽略
        vector<int> vnStage(numCandidates, 0);
        vector<int> vnMatchesStage(numCandidates, 0);
        int index = 0;

        // Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);
        // step 2. 对每个候选关键帧都进行详细的分析，包括进行 Sim3 计算和检验
        for (KeyFrame *pKFi : vpBowCand)
        {
            // cout << endl << "---------------------------------------" << endl;

            if (!pKFi || pKFi->isBad())
                continue;

            // std::cout << "KF candidate: " << pKFi->mnId << std::endl;

            // Current KF against KF with covisibles version
            // 2.1 获得候选关键帧的局部窗口 W_m
            // 拿到候选关键帧的10个最优共视帧
            std::vector<KeyFrame *> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
            if (vpCovKFi.empty())
            {
                std::cout << "Covisible list empty" << std::endl;
                vpCovKFi.push_back(pKFi);
            }
            else
            {
                // 再加上候选关键帧自己(这里操作比较迷,看起来只是为了把候选关键帧放到容器的第一顺位)
                vpCovKFi.push_back(vpCovKFi[0]);
                vpCovKFi[0] = pKFi;
            }

            // step 2.2：【1.0版本】将这步挪上来了，逻辑有些许变化，可以节省部分时间，主要意思没变
            // 好奇的可以去上个版本看看
            // 标记是否因为窗口内有当前关键帧的共视关键帧
            bool bAbortByNearKF = false;

            // 遍历窗口内的每个关键帧
            for (int j = 0; j < vpCovKFi.size(); ++j)
            {
                // 如果窗口中的帧是当前帧的共视帧则结束这个循环
                if (spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
                {
                    bAbortByNearKF = true;
                    // cout << "BoW:Candidate KF aborted by proximity" << endl;
                    break;
                }
            }
            if (bAbortByNearKF)
            {
                // std::cout << "Check BoW aborted because is close to the matched one " << std::endl;
                continue;
            }
            // std::cout << "Check BoW continue because is far to the matched one " << std::endl;

            // search by bow 返回的参数, 记录窗口Wm中每个关键帧有哪些点能在当前关键帧Ka中通过bow找到匹配点
            std::vector<std::vector<MapPoint *>> vvpMatchedMPs;
            vvpMatchedMPs.resize(vpCovKFi.size());

            // 记录整个窗口中有那些点能在Ka中通过bow找到匹配点(这个set是辅助容器,避免重复添加地图点)
            std::set<MapPoint *> spMatchedMPi;
            int numBoWMatches = 0;

            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的关键帧
            KeyFrame *pMostBoWMatchesKF = pKFi; // 这个很重要！

            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的数量
            int nMostBoWNumMatches = 0;

            // 下面两个变量是为了sim3 solver准备的
            // 1、记录窗口中的地图点能在当前关键帧中找到的匹配的点(数量的上限是当前关键帧地图点的数量)
            std::vector<MapPoint *> vpMatchedPoints = std::vector<MapPoint *>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
            // 2、记录上面的地图点分别对应窗口中的关键帧(数量的上限是当前关键帧地图点的数量)
            std::vector<KeyFrame *> vpKeyFrameMatchedMP = std::vector<KeyFrame *>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame *>(NULL));

            // 记录在W_km中有最多匹配点的帧的局部index, 这个后面没有用到
            int nIndexMostBoWMatchesKF = 0;

            // ! 以下循环中并没有重新赋值 pMostBoWMatchesKF ，一直是初始值：候选关键帧
            // 遍历窗口内 Wm 的每个关键帧
            // 2.3 通过 Bow 寻找候选帧窗口内的关键帧地图点与当前关键帧的匹配点
            for (int j = 0; j < vpCovKFi.size(); ++j)
            {
                if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                    continue;

                int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
                // cout << "BOW: " << num << " putative matches with KF " << vpCovKFi[j]->mnID << endl;
                if (num > nMostBoWNumMatches)
                {
                    nMostBoWNumMatches = num;
                    nIndexMostBoWMatchesKF = j;
                }
            }

            // 遍历窗口内的每个关键帧
            // 2.4 把窗口内的匹配点转换为 Sim3Solver 接口定义的格式
            for (int j = 0; j < vpCovKFi.size(); ++j)
            {
                // cout << "Matches: " << num << endl;

                // 遍历窗口内的某一个关键帧与当前关键帧由bow得到的匹配的地图点
                // 注意这里每个vvpMatchedMPs[j]的大小都是相等的且等于当前关键帧中的总地图点数量，详细请看searchByBow
                for (int k = 0; k < vvpMatchedMPs[j].size(); ++k)
                {
                    // 地图点指针
                    MapPoint *pMPi_j = vvpMatchedMPs[j][k];

                    // 如果指针为空或地图点被标记为bad,则跳过当前循环
                    if (!pMPi_j || pMPi_j->isBad())
                        continue;

                    // 窗口内不同关键帧与当前关键帧可能看到相同的3D点, 利用辅助容器避免重复添加
                    if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                    {
                        // 利用辅助容器记录添加过的点
                        spMatchedMPi.insert(pMPi_j);
                        // 统计窗口内有多少地图点能在当前关键中找到匹配
                        numBoWMatches++;

                        // 记录窗口中的地图点能在当前关键帧中找到的匹配的点
                        vpMatchedPoints[k] = pMPi_j;
                        // 记录上面的地图点分别对应窗口中的关键帧(数量的上限是当前关键帧地图点的数量)
                        vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                    }
                }
            }
            // 1. 前面统计了 vpCovKFi 中每个帧与当前帧匹配点的位置，可否用点数高的代替
            // 2. 有可能作者认为在 DetectNBestCandidates 已经找到共视关键帧中分数最多的了，所以这里不做判断直接使用原始的候选关键帧
            // pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

            // cout << "BoW:" << numBoWMatches << " independent putative matches" << endl;
            // 当窗口内的帧不是当前关键帧的相邻帧且匹配点足够多时
            // step 3. 利用RANSAC寻找候选关键帧窗口与当前关键帧的相对位姿T_cm的初始值(可能是Sim3)
            // nBoWMatches = 20; // 最低 bow 匹配特征点数
            // if (numBoWMatches >= nBoWMatches) // 原版本是这个
            if (!bAbortByNearKF && numBoWMatches >= nBoWMatches) // TODO pick a good threshold
            {
                cout << "-----------------------------" << endl;
                cout << "Geometric validation with " << numBoWMatches << endl;
                cout << "KFc:" << mpCurrentKF->mnId << "; KFm:" << pMostBoWMatchesKF->mnId << endl;

                // Geometric validation
                bool bFixedScale = mbFixScale;
                // 如果是单目带imu的模式且IMU初始化未完成第三阶段，则不固定scale
                if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                    bFixedScale = false;

                // 3.1 初始化 sim3 solver
                // Sim3Solver 的接口与 orbslam2 略有不同, 因为现在是 1-N 的对应关系
                Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
                // Sim3Solver Ransac 置信度 0.99，至少 20 个inliers 最多 300 次迭
                solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers, nBowInliers = 15

                bool bNoMore = false;
                vector<bool> vbInliers;
                int nInliers;
                bool bConverge = false;
                Eigen::Matrix4f mTcm;

                // 3.2 迭代到收敛
                while (!bConverge && !bNoMore)
                {
                    mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
                    // Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                }

                // cout < "Num inliers:" < nInliers < endl;
                // 3.3 Guide matching refinement: 利用初始的 Scm 信息，进行双向重投影，并非线性优化得到更精确的Scm。
                // ? 这里应该是 Tam 吧
                if (bConverge)
                {
                    // std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                    // Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                    //  Match by reprojection
                    vpCovKFi.clear();
                    // 拿到窗口内匹配最多的帧的最佳10个共视帧和它自己组成的窗口
                    vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                    vpCovKFi.push_back(pMostBoWMatchesKF);
                    // 这个后面没有用到
                    set<KeyFrame *> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                    // std::cout << "There are " << vpCovKFi.size() <<" near KFs" << std::endl;

                    // 辅助容器, 避免重复添加地图点
                    set<MapPoint *> spMapPoints;
                    // 这两个容器是 searchByProjection 定义的容器
                    // 记录窗口内地图点
                    vector<MapPoint *> vpMapPoints;
                    // 记录每个地图点对应的串口内的关键帧
                    vector<KeyFrame *> vpKeyFrames;
                    // vpMapPoints  {mp1, mp2, mp3, mp4}
                    // vpKeyFrames  {kf1, kf1, kf2, kf3}
                    // 遍历窗Wm内的所有关键帧
                    for (KeyFrame *pCovKFi : vpCovKFi)
                    {
                        // 遍历窗口内每个关键帧的所有地图点
                        for (MapPoint *pCovMPij : pCovKFi->GetMapPointMatches())
                        {
                            // 如果指针为空或者改地图点被标记为bad
                            if (!pCovMPij || pCovMPij->isBad())
                                continue;
                            // 避免重复添加
                            if (spMapPoints.find(pCovMPij) == spMapPoints.end())
                            {
                                spMapPoints.insert(pCovMPij);
                                vpMapPoints.push_back(pCovMPij);
                                vpKeyFrames.push_back(pCovKFi);
                            }
                        }
                    }

                    // std::cout << "There are " << vpKeyFrames.size() <<" KFs which view all the mappoints" << std::endl;

                    // 拿到solver 估计的 Scm初始值, 为后续的非线性优化做准备, 在这里 c 表示当前关键帧, m 表示回环/融合候选帧
                    g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(), solver.GetEstimatedTranslation().cast<double>(), (double)solver.GetEstimatedScale());
                    // 候选关键帧在其世界坐标系下的坐标
                    g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                    // 利用初始的Scm估计确定世界坐标系在当前相机中的位姿
                    g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                    // 准备用来SearchByProjection的位姿信息
                    Sophus::Sim3f mScw = Converter::toSophus(gScw);

                    // 记录最后searchByProjection的结果
                    vector<MapPoint *> vpMatchedMP;
                    vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
                    vector<KeyFrame *> vpMatchedKF;
                    vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame *>(NULL));
                    // 3.3.1 重新利用之前计算的mScw信息, 通过投影寻找更多的匹配点
                    int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                    // cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

                    // 如果拿到了足够多的匹配点, nProjMatches = 50
                    if (numProjMatches >= nProjMatches)
                    {
                        // Optimize Sim3 transformation with every matches
                        Eigen::Matrix<double, 7, 7> mHessian7x7;

                        bool bFixedScale = mbFixScale;
                        // 在imu模式下imu未完成第3阶段初始化就不固定scale
                        if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                            bFixedScale = false;
                        // 3.3.2 利用搜索到的更多的匹配点用Sim3优化投影误差得到的更好的 gScm
                        // pKFi是候选关键帧
                        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                        // 3.3.3 如果内点足够多,用更小的半径搜索匹配点,并且再次进行优化(p.s.这里与论文不符,并没有再次优化)
                        if (numOptMatches >= nSim3Inliers)
                        {
                            // 前面已经声明了这些变量了,无需再次声明
                            g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                            g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                            Sophus::Sim3f mScw = Converter::toSophus(gScw);

                            vector<MapPoint *> vpMatchedMP;
                            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
                            // 3.3.4 重新利用之前计算的mScw信息, 通过更小的半径和更严格的距离的投影寻找匹配点
                            // 5 : 半径的增益系数(对比之前下降了)---> 更小的半径, 1.0 , hamming distance 的阀值增益系数---> 允许更小的距离
                            int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                            // 当新的投影得到的内点数量大于nProjOptMatches=80时
                            if (numProjOptMatches >= nProjOptMatches)
                            {
                                /// 以下为调试信息
                                int max_x = -1, min_x = 1000000;
                                int max_y = -1, min_y = 1000000;
                                for (MapPoint *pMPi : vpMatchedMP)
                                {
                                    if (!pMPi || pMPi->isBad())
                                    {
                                        continue;
                                    }

                                    tuple<size_t, size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                    int index = get<0>(indexes);
                                    if (index >= 0)
                                    {
                                        int coord_x = pKFi->mvKeysUn[index].pt.x;
                                        if (coord_x < min_x)
                                        {
                                            min_x = coord_x;
                                        }
                                        if (coord_x > max_x)
                                        {
                                            max_x = coord_x;
                                        }
                                        int coord_y = pKFi->mvKeysUn[index].pt.y;
                                        if (coord_y < min_y)
                                        {
                                            min_y = coord_y;
                                        }
                                        if (coord_y > max_y)
                                        {
                                            max_y = coord_y;
                                        }
                                    }
                                }
                                // 调试完毕

                                // Boundary--------------------------------------------------------------------------------------------------------------------------------

                                // ps：共视几何校验的实现代码👇

                                // step 4. 用当前关键帧的相邻关键帧来验证前面得到的 Tam
                                int nNumKFs = 0; // 统计验证成功的关键帧数量

                                // vpMatchedMPs = vpMatchedMP;
                                // vpMPs = vpMapPoints;
                                //  Check the Sim3 transformation with the current KeyFrame covisibles

                                //  step 4.1 拿到用来验证的关键帧组(后称为【验证组】)，也就是当前关键帧的 5 个共视关键帧，nNumCovisibles = 5;
                                vector<KeyFrame *> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                                int j = 0;

                                // 遍历验证组，当有 3 个关键帧验证成功或遍历所有的关键帧后，结束循环
                                while (nNumKFs < 3 && j < vpCurrentCovKFs.size())
                                {
                                    // 拿出验证组中的 1 个关键帧
                                    KeyFrame *pKFj = vpCurrentCovKFs[j];

                                    // 为 DetectCommonRegionsFromLastKF 准备一个初始位姿, 这个用来进行 searchByProjection（引导搜索匹配）
                                    Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                    g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                                    // * 修改为👇：下面是书上的代码，上面是原来的代码（有错误好像）
                                    // cv::Mat mTjc = pKFj->GetPose() * mpCurrentKF->GetPoseInverse();
                                    // g2o::Sim3 gSjc(Converter::toMatrix3d(mTjc.rowRange(0, 3).colRange(0, 3)), Converter::toVector3d(mTjc.rowRange(0, 3).col(3)), 1.0);

                                    g2o::Sim3 gSjw = gSjc * gScw;
                                    int numProjMatches_j = 0;
                                    vector<MapPoint *> vpMatchedMPs_j;

                                    // step 4.2 几何校验函数。这个函数里面其实是个 searchByProjection : 通过之前计算的位姿转换地图点并通过投影搜索匹配点, 若大于一定数目，则成功验证一次
                                    bool bValid = DetectCommonRegionsFromLastKF(pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                    // 统计 valid 的帧的数量
                                    if (bValid)
                                    {
                                        Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                        Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                        Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                        Eigen::Vector3f vector_dist = Tc_cj.translation();

                                        nNumKFs++; // 统计成功校验的帧的数量
                                    }
                                    j++;
                                }

                                // Boundary--------------------------------------------------------------------------------------------------------------------------------

                                // 这里又是没有用的代码,只是记录一点信息,可能是为了方便打印检查
                                if (nNumKFs < 3)
                                {
                                    vnStage[index] = 8;
                                    vnMatchesStage[index] = nNumKFs;
                                }

                                // 记录第二次 searchByProjection 得到最多匹配点的关键帧的各种信息, 最后作为回环帧/融合帧
                                if (nBestMatchesReproj < numProjOptMatches)
                                {
                                    nBestMatchesReproj = numProjOptMatches; // 投影匹配的数量
                                    nBestNumCoindicendes = nNumKFs;         // 成功验证的帧数
                                    pBestMatchedKF = pMostBoWMatchesKF;     // 记录候选帧窗口内与当前关键帧相似度最高的帧
                                    g2oBestScw = gScw;                      // 记录最优的位姿(这个位姿是由Tam推到出来的 : Taw = Tam * Tmw,这里a表示c)
                                    vpBestMapPoints = vpMapPoints;          //  记录所有的地图点
                                    vpBestMatchedMapPoints = vpMatchedMP;   // 记录所有的地图点中被成功匹配的点
                                }
                            }
                        }
                    }
                }
                /*else
                {
                    Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
                }*/
            }
            index++;
        }

        // 如果成功找到了共同区域帧把记录的最优值存到输出变量中
        if (nBestMatchesReproj > 0)
        {
            pLastCurrentKF = mpCurrentKF;
            nNumCoincidences = nBestNumCoindicendes; // 成功几何验证的帧数
            pMatchedKF2 = pBestMatchedKF;
            pMatchedKF2->SetNotErase();
            g2oScw = g2oBestScw;
            vpMPs = vpBestMapPoints;
            vpMatchedMPs = vpBestMatchedMapPoints;

            // 如果有三个成功验证则 return ture
            return nNumCoincidences >= 3;
        }
        else
        {
            // 这里是一些无用的变量, 左值只用来打印检查
            int maxStage = -1;
            int maxMatched;
            for (int i = 0; i < vnStage.size(); ++i)
            {
                if (vnStage[i] > maxStage)
                {
                    maxStage = vnStage[i];
                    maxMatched = vnMatchesStage[i];
                }
            }
        }
        // 如果【少于 3 个】当前关键帧的共视关键帧验证了这个候选帧, 那么返回失败。注意, 这里的失败并不代表最终的验证失败, 后续会开启【时序校验】
        return false;
    }

    // notice 用来验证候选帧的函数
    /**
     * @brief 用来验证候选帧的函数, 这个函数的名字取的不好, 函数的本意是想利用候选帧的共视关键帧来验证候选帧,不如改叫做：DetectCommonRegionsFromCoVKF
     *
     * @param[in] pCurrentKF 当前关键帧
     * @param[in] pMatchedKF 候选帧
     * @param[in] gScw 世界坐标系在验证帧下的Sim3
     * @param[out] nNumProjMatches 最后匹配的数目
     * @param[out] vpMPs 候选帧的窗口内所有的地图点
     * @param[out] vpMatchedMPs 候选帧的窗口内被匹配到的地图点
     * @return true 验证成功
     * @return false 验证失败
     */
    bool LoopClosing::DetectCommonRegionsFromLastKF(
        KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
        std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs)
    {
        // 所有的匹配点
        set<MapPoint *> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
        // 通过Sim3变换后投影寻找匹配点
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

        int nProjMatches = 30;
        // 如果匹配点数目大于一定的阀值,则认为验证成功,返回ture
        if (nNumProjMatches >= nProjMatches)
        {
            return true;
        }

        return false;
    }

    /**
     * @brief 包装了一下searchByProjection, 把窗口内的所有地图点往当前关键帧上投影, 寻找匹配点
     *
     * @param[in] pCurrentKF 当前关键帧
     * @param[in] pMatchedKFw 候选帧
     * @param[in] g2oScw 世界坐标系在验证帧坐标系下的位姿
     * @param[in] spMatchedMPinOrigin 没有用上?
     * @param[in] vpMapPoints 候选帧及其共视关键帧组成的窗口里所有的地图点
     * @param[in] vpMatchedMapPoints 候选帧及其共视关键帧组成的窗口里所有被匹配上的地图点
     * @return int 匹配点的数量
     */
    int LoopClosing::FindMatchesByProjection(
        KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw, g2o::Sim3 &g2oScw,
        set<MapPoint *> &spMatchedMPinOrigin, vector<MapPoint *> &vpMapPoints,
        vector<MapPoint *> &vpMatchedMapPoints)
    {
        int nNumCovisibles = 10; // change 上个版本为5
        // 拿出候选帧的10个最好的共视关键帧
        vector<KeyFrame *> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
        int nInitialCov = vpCovKFm.size();
        // 把自己也加进去, 组成一个局部窗口
        vpCovKFm.push_back(pMatchedKFw);

        // 辅助容器,防止重复添加
        set<KeyFrame *> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
        // 拿出当前关键帧的共视关键帧
        set<KeyFrame *> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();

        // 1. 如果数量不够 扩充窗口
        if (nInitialCov < nNumCovisibles)
        {

            // 对于之前里的每个关键帧
            for (int i = 0; i < nInitialCov; ++i)
            {
                // 拿到共视关键帧
                vector<KeyFrame *> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
                int nInserted = 0;
                int j = 0;
                // 这个while循环统计的遍历后面都没有用到, 没有任何作用
                while (j < vpKFs.size() && nInserted < nNumCovisibles)
                {
                    // 如果没有被重复添加且不是当前关键帧的共视关键帧
                    if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                    {
                        spCheckKFs.insert(vpKFs[j]);
                        ++nInserted;

                        // 改成这样
                        vpCovKFm.push_back(vpKFs[j]);
                    }
                    ++j;
                }
                // 这里是原来的代码，这么写不太合适，会出现重复
                // 所以下面的插入可以改成放在if里面
                // 把每个帧的共视关键帧都加到窗口内
                // vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());  // 已放上面
            }
        }

        // 辅助容器, 防止地图点被重复添加
        set<MapPoint *> spMapPoints;

        // 清空这两个容器,重新进行searchByProjection
        vpMapPoints.clear();
        vpMatchedMapPoints.clear();
        // 2. 提取一堆mp
        // 对于窗口内每个关键帧
        for (KeyFrame *pKFi : vpCovKFm)
        {
            // 对于每个关键帧的地图点
            for (MapPoint *pMPij : pKFi->GetMapPointMatches())
            {
                // 如果指针不是空,且点不是bad
                if (!pMPij || pMPij->isBad())
                    continue;

                // 如果没有被重复添加
                if (spMapPoints.find(pMPij) == spMapPoints.end())
                {
                    spMapPoints.insert(pMPij);
                    vpMapPoints.push_back(pMPij);
                }
            }
        }

        Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
        ORBmatcher matcher(0.9, true);

        // 3. 初始化被匹配到的地图点容器, 匹配上限是当前帧的地图点数量
        vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint *>(NULL));
        // 把窗口中的点向当前关键帧投影, 搜索匹配点, 注意验证的时候用的搜索半径是最小的
        int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

        return num_matches;
    }

    // todo 作用：在检测到回环后进行修正优化位姿
    /**
     * @brief 相同地图检测到共同区域叫回环，不同地图叫融合，这个函数是在检测到回环后进行修正优化位姿
     */
    void LoopClosing::CorrectLoop()
    {
        // cout << "Loop detected!" << endl;

        // Step1. 结束局部地图线程、全局 BA线程，为闭环矫正做准备
        // 请求局部地图停止，防止在回环矫正时局部地图线程中 InsertKeyFrame 函数插入新的关键帧
        mpLocalMapper->RequestStop();
        mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

        // 如果正在进行全局 BA，丢弃它
        if (isRunningGBA())
        {
            cout << "Stoping Global Bundle Adjustment...";
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;
            // 记录全局 BA 次数
            mnFullBAIdx++;

            if (mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            cout << "  Done!!" << endl;
        }

        // 一直等到局部地图线程结束再继续
        while (!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // Ensure current keyframe is updated
        // cout << "Start updating connections" << endl;
        // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
        // Step 2. 根据共视关系更新当前关键帧与其它关键帧之间的连接关系
        // 因为之前闭环检测、计算 Sim3 中改变了该关键帧的地图点，所以需要更新
        mpCurrentKF->UpdateConnections();
        // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

        // Step 3. 通过位姿传播，得到 Sim3 优化后，与当前帧相连的关键帧的位姿，以及它们的地图点
        // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
        // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换

        // 取出当前关键帧及其共视关键帧，称为【当前关键帧组】
        mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        mvpCurrentConnectedKFs.push_back(mpCurrentKF);

        // std::cout << "Loop: number of connected KFs -> " + to_string(mvpCurrentConnectedKFs.size()) << std::endl;

        //  CorrectedSim3：存放闭环 g2o 优化后当前关键帧的共视关键帧的世界坐标系下 Sim3 变换
        //  NonCorrectedSim3：存放没有矫正的当前关键帧的共视关键帧的世界坐标系下 Sim3 变换
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // 先将 mpCurrentKF 的 Sim3 变换存入，认为是准的，所以固定不动
        CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
        // 当前关键帧到世界坐标系下的变换矩阵
        Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
        Sophus::SE3f Tcw = mpCurrentKF->GetPose();
        g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>(), 1.0);
        NonCorrectedSim3[mpCurrentKF] = g2oScw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(), mg2oLoopScw.translation() / mg2oLoopScw.scale());
        mpCurrentKF->SetPose(correctedTcw.cast<float>());

        Map *pLoopMap = mpCurrentKF->GetMap();

#ifdef REGISTER_TIMES
        /*KeyFrame* pKF = mpCurrentKF;
        int numKFinLoop = 0;
        while(pKF && pKF->mnId > mpLoopMatchedKF->mnId)
        {
            pKF = pKF->GetParent();
            numKFinLoop += 1;
        }
        vnLoopKFs.push_back(numKFinLoop);*/

        std::chrono::steady_clock::time_point time_StartFusion = std::chrono::steady_clock::now();
#endif
        // 对地图点操作
        {
            // Get Map Mutex
            unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

            const bool bImuInit = pLoopMap->isImuInitialized();
            // 3.1：通过mg2oLoopScw（认为是准的）来进行位姿传播，得到当前关键帧的共视关键帧的世界坐标系下Sim3 位姿（还没有修正）
            // 遍历"当前关键帧组""
            for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
            {
                KeyFrame *pKFi = *vit;

                if (pKFi != mpCurrentKF) // 跳过当前关键帧，因为当前关键帧的位姿已经在前面优化过了，在这里是参考基准
                {
                    // 得到当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的相对变换

                    Sophus::SE3f Tiw = pKFi->GetPose();
                    Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                    // * 修改为👇：修改上面原来的代码（有错误）---> 下面的代码
                    // cv::Mat Tic = mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse();

                    // g2oSic：当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                    // 这里是 non-correct, 所以 scale = 1.0

                    g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                    // * 修改为👇：修改上面原来的代码（有错误）---> 下面的代码
                    // g2o::Sim3 g2oSic(Converter::toMatrix3d(Tic.rowRange(0, 3).colRange(0, 3)), Converter::toVector3d(Tic.rowRange(0, 3).col(3)), 1.0);

                    // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                    g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;

                    // 存放闭环 g2o 优化后当前关键帧的共视关键帧的Sim3 位姿
                    CorrectedSim3[pKFi] = g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                    pKFi->SetPose(correctedTiw.cast<float>());

                    // Pose without correction
                    g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(), Tiw.translation().cast<double>(), 1.0);
                    NonCorrectedSim3[pKFi] = g2oSiw;
                }
            }

            // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
            // 3.2：得到矫正的当前关键帧的共视关键帧位姿后，修正这些关键帧的地图点
            // 遍历待矫正的共视关键帧（不包括当前关键帧）
            for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                g2o::Sim3 g2oCorrectedSiw = mit->second;
                g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                pKFi->SetPose(correctedTiw.cast<float>());*/

                vector<MapPoint *> vpMPsi = pKFi->GetMapPointMatches();

                // 遍历待矫正共视关键帧中的每一个地图点
                for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
                {
                    MapPoint *pMPi = vpMPsi[iMP];
                    if (!pMPi)
                        continue;
                    if (pMPi->isBad())
                        continue;
                    if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId) // 标记，防止重复矫正
                        continue;

                    // Project with non-corrected pose and project back with corrected pose
                    // 矫正过程本质上也是基于当前关键帧的优化后的位姿展开的
                    // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                    // map(P) 内部做了变换 R*P +t
                    // 下面变换是：eigP3Dw： world →g2oSiw→ i →g2oCorrectedSwi→ world
                    Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                    pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                    // 记录矫正该地图点的关键帧id，防止重复
                    pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                    // 记录该地图点所在的关键帧id
                    pMPi->mnCorrectedReference = pKFi->mnId;
                    // 因为地图点更新了，需要更新其平均观测方向以及观测距离范围
                    pMPi->UpdateNormalAndDepth();
                }

                // 这块减去了setpose

                // 修改速度
                // Correct velocity according to orientation correction
                if (bImuInit)
                {
                    Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation()).cast<float>();
                    pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
                }

                // Make sure connections are updated
                // 3.3 根据共视关系更新当前帧与其它关键帧之间的连接
                // 地图点的位置改变了,可能会引起共视关系\权值的改变
                pKFi->UpdateConnections();
            }
            // TODO Check this index increasement
            mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

            // Start Loop Fusion
            // Update matched map points and replace if duplicated
            // Step 4. 检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的进行替换或填补
            // mvpCurrentMatchedPoints 是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
            for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++)
            {
                if (mvpLoopMatchedMPs[i])
                {
                    // 取出同一个索引对应的两种地图点，决定是否要替换
                    // 匹配投影得到的地图点
                    MapPoint *pLoopMP = mvpLoopMatchedMPs[i];
                    // 原来的地图点
                    MapPoint *pCurMP = mpCurrentKF->GetMapPoint(i);

                    // 如果有重复的MapPoint，则用匹配的地图点代替现有的
                    // 因为匹配的地图点是经过一系列操作后比较精确的，现有的地图点很可能有累计误差
                    if (pCurMP)
                        pCurMP->Replace(pLoopMP);
                    else
                    {
                        // 如果当前帧没有该MapPoint，则直接添加
                        mpCurrentKF->AddMapPoint(pLoopMP, i);
                        pLoopMP->AddObservation(mpCurrentKF, i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }
            // cout << "LC: end replacing duplicated" << endl;
        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        // Step 5. 将闭环相连关键帧组mvpLoopMapPoints 投影到当前关键帧组中，进行匹配，融合，新增或替换当前关键帧组中KF的地图点
        // 因为 闭环相连关键帧组mvpLoopMapPoints 在地图中时间比较久经历了多次优化，认为是准确的
        // 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
        // CorrectedSim3：存放矫正后当前关键帧的共视关键帧，及其世界坐标系下Sim3 变换
        SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        // Step 6. 更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
        // LoopConnections：存储因为闭环地图点调整而新生成的连接关系
        map<KeyFrame *, set<KeyFrame *>> LoopConnections;

        // 6.1 遍历当前帧相连关键帧组（一级相连）
        for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;
            // 6.2 得到与当前帧相连关键帧的相连关键帧（二级相连）
            vector<KeyFrame *> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

            // Update connections. Detect new links.
            // 6.3 更新一级相连关键帧的连接关系(会把当前关键帧添加进去,因为地图点已经更新和替换了)
            pKFi->UpdateConnections();
            // 6.4 取出该帧更新后的连接关系
            LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
            // 6.5 从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
            {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            // 6.6 从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
            {
                LoopConnections[pKFi].erase(*vit2);
            }
        }

        // Optimize graph
        bool bFixedScale = mbFixScale;
        // TODO CHECK; Solo para el monocular inertial
        if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndFusion = std::chrono::steady_clock::now();

        double timeFusion = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndFusion - time_StartFusion).count();
        vdLoopFusion_ms.push_back(timeFusion);
#endif
        // cout << "Optimize essential graph" << endl;
        if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
        {
            Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
        }
        else
        {
            // cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
            //  Step 7. 进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
            Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
        }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndOpt = std::chrono::steady_clock::now();

        double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndOpt - time_EndFusion).count();
        vdLoopOptEss_ms.push_back(timeOptEss);
#endif

        mpAtlas->InformNewBigChange();

        // Add loop edge
        // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
        // 它在下一次的Essential Graph里面使用
        mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
        mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

        // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
        // 闭环地图没有imu初始化或者 仅有一个地图且内部关键帧<200时才执行全局BA，否则太慢
        if (!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))
        {
            // Step 9. 新建一个线程用于全局BA优化
            // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints

            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            mnCorrectionGBA = mnNumCorrection;

            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
        }

        // Loop closed. Release Local Mapping.
        mpLocalMapper->Release();

        mLastLoopKFid = mpCurrentKF->mnId; // TODO old varible, it is not use in the new algorithm
    }

    // note1 作用：【纯视觉】地图融合及优化！
    /**
     * @brief 纯视觉地图融合。在检测到成功验证的融合帧后进行
     * 1. 焊缝区域局部 BA
     * 2. Essential Graph BA
     * 融合两张地图为一张地图
     */
    void LoopClosing::MergeLocal()
    {
        Verbose::PrintMess("MERGE: Merge Visual detected!!!", Verbose::VERBOSITY_NORMAL);
        // mpTracker->SetStepByStep(true);

        // 窗口内共视关键帧的数量， 上个【0.4版本】是 15
        int numTemporalKFs = 25; // TODO （set by parameter）Temporal KFs in the local window if the map is inertial.

        // Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        //  建立本质图必须的量
        KeyFrame *pNewChild;
        KeyFrame *pNewParent;

        // 当前关键帧的窗口
        vector<KeyFrame *> vpLocalCurrentWindowKFs;
        // 候选关键帧的窗口
        vector<KeyFrame *> vpMergeConnectedKFs;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        // 记录是否把全局BA停下
        bool bRelaunchBA = false;

        Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
        //  If a Global Bundle Adjustment is running, abort it
        if (isRunningGBA())
        {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if (mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true; // 以后还会重新开启
        }

        Verbose::PrintMess("MERGE-VISUAL: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
        cout << "Request Stop Local Mapping" << endl;
        //  请求局部建图线程停止
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        // 等待局部建图工作停止
        while (!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }
        cout << "Local Map stopped" << endl;

        mpLocalMapper->EmptyQueue(); // ? 这里清理的是哪些？

        // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
        // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
        // 当前关键帧的共视关键帧和他们观测到的地图点会先被融合, 融合后的图会变成新的当前激活地图.
        // 之后, 所有当前地图的其他部分会被转换到当前地图中, 这样是为了保证tracking的实时性.

        // 当前关键帧地图的指针
        Map *pCurrentMap = mpCurrentKF->GetMap();
        // 融合关键帧地图的指针
        Map *pMergeMap = mpMergeMatchedKF->GetMap();

        // std::cout << "Merge local, Active map: " << pCurrentMap->GetId() << std::endl;
        // std::cout << "Merge local, Non-Active map: " << pMergeMap->GetId() << std::endl;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif

        // Ensure current keyframe is updated
        // 先保证当前关键帧的链接关系是最新的
        mpCurrentKF->UpdateConnections();

        // Step 1 构建当前关键帧和融合关键帧的局部窗口(关键帧+地图点)
        // Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
        // 当前关键帧的局部窗口(仅是辅助容器，防止重复添加)
        set<KeyFrame *> spLocalWindowKFs;
        // Get MPs in the welding area from the current map
        //  当前关键帧局部串口能观测到的所有地图点(仅是辅助容器，防止重复添加)
        set<MapPoint *> spLocalWindowMPs;

        // 这段代码只在 visual 状态下才会被使用，所以只会执行 else
        // Step 1.1 构造当前关键帧局部共视帧窗口
        if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) // TODO Check the correct initialization，纯视觉模式，这个 if 条件不会执行
        {
            KeyFrame *pKFi = mpCurrentKF;
            int nInserted = 0;
            while (pKFi && nInserted < numTemporalKFs)
            {
                spLocalWindowKFs.insert(pKFi);
                pKFi = mpCurrentKF->mPrevKF;
                nInserted++;

                set<MapPoint *> spMPi = pKFi->GetMapPoints();
                spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
            }

            pKFi = mpCurrentKF->mNextKF;
            while (pKFi) //! 这里会死循环,不过无所谓，这个外面的 if 永远不会执行
            {
                spLocalWindowKFs.insert(pKFi);

                set<MapPoint *> spMPi = pKFi->GetMapPoints();
                spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

                pKFi = mpCurrentKF->mNextKF;
            }
        }
        else
        {
            spLocalWindowKFs.insert(mpCurrentKF); // 把自己先加到窗口内
        }

        // 拿到当前关键帧的 numTemporalKFs = 15 个最佳共视关键帧
        vector<KeyFrame *> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        // 把当前帧的共视帧都加到窗口里
        spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        // * 1.0 版本新加的，将当前关键帧也添加进来
        spLocalWindowKFs.insert(mpCurrentKF);

        // 限制 while 循环次数 0.4 版本是 3
        const int nMaxTries = 5;
        int nNumTries = 0;

        // 如果窗口里的关键帧数量不够就再拉一些窗口里的关键帧的共视关键帧(二级共视关键帧)进来
        while (spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
        {
            vector<KeyFrame *> vpNewCovKFs;
            vpNewCovKFs.empty();

            // 遍历窗口内的每一个关键帧
            for (KeyFrame *pKFi : spLocalWindowKFs)
            {
                // 拿到一些二级共视关键帧，取 一级的一半
                vector<KeyFrame *> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);

                // 对于每个二级共视关键帧
                for (KeyFrame *pKFcov : vpKFiCov)
                {
                    // 如果指针不为空，且关键帧没有被标记为 bad，且没有被添加过则加到窗口内
                    if (pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                    {
                        vpNewCovKFs.push_back(pKFcov);
                    }
                }
            }
            // 用set记录，防止重复添加
            spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            // 递增循环次数
            nNumTries++;
        }
        // todo：----------------------------------------- 至此，找到了当前关键帧的 15 个共视关键帧了

        // Step 1.2 把当前关键帧窗口里关键帧观测到的地图点加进来
        for (KeyFrame *pKFi : spLocalWindowKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            set<MapPoint *> spMPs = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end()); // spLocalWindowMPs --- 存放当前帧及其 15 个共视帧的所有地图点
        }

        // std::cout << "[Merge]: Ma = " << to_string(pCurrentMap->GetId()) << "; #KFs = " << to_string(spLocalWindowKFs.size()) << "; #MPs = " << to_string(spLocalWindowMPs.size()) << std::endl;

        // Step 1.3 构造融合帧的共视帧窗口
        // 融合关键帧的共视关键帧们
        set<KeyFrame *> spMergeConnectedKFs;
        // 这段代码只在visual状态下才会被使用，所以只会执行else
        if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) // TODO Check the correct initialization
        {
            KeyFrame *pKFi = mpMergeMatchedKF;
            int nInserted = 0;
            while (pKFi && nInserted < numTemporalKFs / 2)
            {
                spMergeConnectedKFs.insert(pKFi);
                pKFi = mpCurrentKF->mPrevKF;
                nInserted++;
            }

            pKFi = mpMergeMatchedKF->mNextKF;
            while (pKFi && nInserted < numTemporalKFs)
            {
                spMergeConnectedKFs.insert(pKFi);
                pKFi = mpCurrentKF->mNextKF;
            }
        }
        else
        {
            // 先把融合关键帧自己添加到窗口内
            spMergeConnectedKFs.insert(mpMergeMatchedKF);
        }
        // 拿到融合关键帧最好的numTemporalKFs(25)个最佳共视关键帧
        vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
        // 记录循环次数
        nNumTries = 0;
        // 如果融合关键帧窗口里的关键帧不够就再拉一些窗口里的关键帧的共视帧进来(二级共视关键帧)
        while (spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
        {
            vector<KeyFrame *> vpNewCovKFs;
            // 遍历融合关键帧内的所有的一级共视关键帧
            for (KeyFrame *pKFi : spMergeConnectedKFs)
            {
                // 拿到一些二级共视关键帧
                vector<KeyFrame *> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
                // 对于每个二级共视关键帧
                for (KeyFrame *pKFcov : vpKFiCov)
                {
                    // 如果指针不为空,且关键帧没有被标记为bad,且没有被添加过则加到窗口内
                    if (pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                    {
                        vpNewCovKFs.push_back(pKFcov);
                    }
                }
            }
            // 用set记录,防止重复添加
            spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            // 递增循环次数
            nNumTries++;
        }

        // Step 1.4  把融合关键帧窗口里关键帧观测到的地图点加进来
        // 融合关键帧共视窗口内的所有地图点
        set<MapPoint *> spMapPointMerge;
        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            set<MapPoint *> vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        }

        // 把融合关键帧窗口内的地图点加到待融合的向量中
        vector<MapPoint *> vpCheckFuseMapPoint;
        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        // 把spMapPointMerge拷贝到vpCheckFuseMapPoint里
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // std::cout << "[Merge]: Mm = " << to_string(pMergeMap->GetId()) << "; #KFs = " << to_string(spMergeConnectedKFs.size()) << "; #MPs = " << to_string(spMapPointMerge.size()) << std::endl;

        // Step 2 根据之前的Sim3初始值, 记录当前帧窗口内关键帧，地图点的矫正前的值，和矫正后的初始值

        // Step 2.1 先计算当前关键帧矫正前的值，和矫正后的初始值
        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();

        // cv::Mat Twc = mpCurrentKF->GetPoseInverse();

        // cv::Mat Rwc = Twc.rowRange(0, 3).rolRange(0, 3);
        // cv::Mat twc = Twc.rowRange(0, 3).rol(3);

        // 记录没有融合矫正之前的Swc和Scw
        g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
        g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();

        // 拿到之前通过Sim3(见 NewDetectCommonRegion)计算得到的当前关键帧融合矫正之后的初始位姿
        // mg2oMergeScw 存放的是融合关键帧所在的世界坐标系到当前帧的 Sim3 位姿
        g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; // TODO Check the transformation

        // 记录当前关键帧窗口内所有关键帧融合矫正之前的位姿，和融合矫正之后的初始位姿
        KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
        // g2oNonCorrectedScw 是当前关键帧世界坐标系下的
        // g2oCorrectedScw 是融合关键帧所在的世界坐标系下的
        vCorrectedSim3[mpCurrentKF] = g2oCorrectedScw;
        vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

#ifdef REGISTER_TIMES
        vnMergeKFs.push_back(spLocalWindowKFs.size() + spMergeConnectedKFs.size());
        vnMergeMPs.push_back(spLocalWindowMPs.size() + spMapPointMerge.size());
#endif

        // Step 2.2 通过当前关键帧融合矫正前后的位姿，把当前关键帧的共视窗口里面剩余的关键帧矫正前后的位姿都给填写上
        // 对于每个当前关键帧共视窗口里的关键帧
        for (KeyFrame *pKFi : spLocalWindowKFs)
        {
            // 跳过空的指针，或者坏的关键帧
            if (!pKFi || pKFi->isBad())
            {
                Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
                continue;
            }

            if (pKFi->GetMap() != pCurrentMap)
                Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

            // 确保这些共视关键帧在当前地图下
            // 保存第i个共视关键帧融合矫正后的初始位姿
            g2o::Sim3 g2oCorrectedSiw;

            if (pKFi != mpCurrentKF)
            {
                // 先记录下融合矫正前的位姿
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                // Pose without correction
                vNonCorrectedSim3[pKFi] = g2oSiw;

                // 根据链式法则,利用当前关键帧和第i个共视关键帧的位姿关系,以及当前关键帧矫正后的初始位姿,推得第i个共视关键帧的矫正后的初始位姿
                Sophus::SE3d Tic = Tiw * Twc;
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2oCorrectedSiw = g2oSic * mg2oMergeScw;
                vCorrectedSim3[pKFi] = g2oCorrectedSiw;
            }
            else
            {
                g2oCorrectedSiw = g2oCorrectedScw;
            }

            // 这一句没有什么作用,下面又被覆盖了
            pKFi->mTcwMerge = pKFi->GetPose();

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // 根据上面计算得到的融合矫正后的初始位姿(Sim3),更新窗口内每个关键帧的mTcwMerge(矫正后的初始位姿)
            double s = g2oCorrectedSiw.scale();
            pKFi->mfScale = s;

            // s*Riw * Pw + tiw = Pi  此时Pi在i坐标系下的坐标，尺度保留的是原来的
            // Riw * Pw + tiw/s = Pi/s 此时Pi/s在i坐标系下的坐标，尺度是最新的的，所以Rt要这么保留
            Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s); // 修正尺度

            // 赋值得到的矫正后的初始位姿
            pKFi->mTcwMerge = correctedTiw.cast<float>();

            // !纯视觉模式，以下代码不执行
            if (pCurrentMap->isImuInitialized())
            {
                Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
            }

            // TODO DEBUG to know which are the KFs that had been moved to the other map
        }

        int numPointsWithCorrection = 0;

        // for(MapPoint* pMPi : spLocalWindowMPs)
        //  Step 2.3 记录所有地图点矫正前的位置，和矫正后的初始值
        //  对于每个窗口内的地图点，之前用的for循环，改成迭代器方便删点
        set<MapPoint *>::iterator itMP = spLocalWindowMPs.begin();
        while (itMP != spLocalWindowMPs.end())
        {
            MapPoint *pMPi = *itMP;
            // 不好的点去掉，1.0新加
            if (!pMPi || pMPi->isBad())
            {
                itMP = spLocalWindowMPs.erase(itMP);
                continue;
            }

            // 拿到参考关键帧，删掉不存在的，1.0新加
            KeyFrame *pKFref = pMPi->GetReferenceKeyFrame();
            if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
            {
                itMP = spLocalWindowMPs.erase(itMP);
                numPointsWithCorrection++;
                continue;
            }

            // 拿到计算好的矫正后参考关键帧的初始位姿
            g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            // 拿到矫正前的参考关键帧的位姿
            g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            // Project with non-corrected pose and project back with corrected pose
            // 先把3D点转换到参考关键帧矫正前的坐标系中
            Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
            // 再转换到矫正后的初始坐标系中
            Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            // 计算旋转部分的变化
            Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            // 矫正后3d点的初始位置
            pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
            // 用旋转部分的变化更新计算3D点normal的新值
            pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

            itMP++;
        }
        /*if(numPointsWithCorrection>0)
        {
            std::cout << "[Merge]: " << std::to_string(numPointsWithCorrection) << " points removed from Ma due to its reference KF is not in welding area" << std::endl;
            std::cout << "[Merge]: Ma has " << std::to_string(spLocalWindowMPs.size()) << " points" << std::endl;
        }*/

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 3 以新（当前帧所在地图）地图换旧（融合帧所在地图）地图，包括【关键帧及地图点关联地图】的以新换旧、【地图集】的以新换旧;
        {
            // 当前地图会被更新，老地图中的重复地图点会被剔除
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);     // We remove the Kfs and MPs in the merged area from the old map

            // std::cout << "Merge local window: " << spLocalWindowKFs.size() << std::endl;
            // std::cout << "[Merge]: init merging maps " << std::endl;

            // 对于当前关键帧共视窗口内的每一个关键帧
            // Step 3.1 更新当前关键帧共视窗口内的每一个关键帧信息
            for (KeyFrame *pKFi : spLocalWindowKFs)
            {
                if (!pKFi || pKFi->isBad())
                {
                    // std::cout << "Bad KF in correction" << std::endl;
                    continue;
                }

                // std::cout << "KF id: " << pKFi->mnId << std::endl;

                //  记录融合矫正前的位姿
                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                // 把这个关键帧的位姿，设置为融合矫正后的初始位姿
                pKFi->SetPose(pKFi->mTcwMerge);

                // Make sure connections are updated
                // 把这个关键帧的地图，设置为融合帧所在的地图
                pKFi->UpdateMap(pMergeMap);

                // 记录这个关键帧是被哪个当前关键帧融合的
                pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;

                // 把这个关键帧所有权给到融合帧所在地图里
                pMergeMap->AddKeyFrame(pKFi);

                // 把这个关键帧从当前活跃地图中删掉
                pCurrentMap->EraseKeyFrame(pKFi);

                // 下面是没用的代码，因为是 IMU 模式下的
                if (pCurrentMap->isImuInitialized())
                {
                    pKFi->SetVelocity(pKFi->mVwbMerge);
                }
            }

            // Step 3.2 更新当前关键帧共视帧窗口所能观测到的地图点的信息（将当前关键帧共视帧窗口所能观测到的地图点添加到融合帧所在的地图中）
            // 把所有地图点的所有权给到老图里面
            // 对于每个当前关键帧共视窗口内的所有地图点
            for (MapPoint *pMPi : spLocalWindowMPs)
            {
                if (!pMPi || pMPi->isBad())
                    continue;

                // 把地图点的位置，设置成融合矫正之后的位置
                pMPi->SetWorldPos(pMPi->mPosMerge);

                // 把地图点 normal ，设置成融合矫正之后的法向量
                pMPi->SetNormalVector(pMPi->mNormalVectorMerge);

                // 把3D（地图点）点所在的地图，设置成融合帧所在的地图
                pMPi->UpdateMap(pMergeMap);

                // 把地图点添加进融合帧所在的地图
                pMergeMap->AddMapPoint(pMPi);

                // 把地图点从当前活跃地图中删掉
                pCurrentMap->EraseMapPoint(pMPi);
            }

            // Step 3.3 更新剩余信息，如 Altas 和融合帧所在地图的信息
            // 在 Altas 中把当前地图休眠，重新激活旧地图（融合帧所在地图）
            mpAtlas->ChangeMap(pMergeMap);

            // 当前地图的信息都到融合帧所在地图里去了，可以设置为 bad
            mpAtlas->SetMapBad(pCurrentMap);

            // 记录地图变化次数
            pMergeMap->IncreaseChangeIndex();

            // TODO for debug
            pMergeMap->ChangeId(pCurrentMap->GetId());

            // std::cout << "[Merge]: merging maps finished" << std::endl;
        }

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 4 融合新旧地图的生成树
        // Rebuild the essential graph in the local window（重新构造 essential graph 的相关信息）

        // 设置当前地图的第一个关键帧不再是第一次生成树了
        pCurrentMap->GetOriginKF()->SetFirstConnection(false);

        // 将当前帧 mpCurrentKF 的父节点命名为 pNewChild
        pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
        // mpCurrentKF 的别名：pNewParent
        pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)

        // 把当前帧的父关键帧，更换为【融合帧】
        mpCurrentKF->ChangeParent(mpMergeMatchedKF);

        // mpMergeMatchedKF 表示待融合地图中与当前关键帧对应上的帧
        // 因为整个待融合地图要融入到当前地图里，为了避免有两个父节点，mpMergeMatchedKF 的原始父节点变成了它的子节点，而当前关键帧成了 mpMergeMatchedKF 的父节点
        // 同理，为了避免 mpMergeMatchedKF 原始父节点（现在已成它的子节点）有两个父节点，需要向上一直改到待融合地图最顶端的父节点

        // 从当前关键帧开始--反向--遍历整个地图
        while (pNewChild)
        {
            // new parent (old child) 不再是 new child (old parent) 的 child 了
            pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop（删除父子关系）

            // 记录原先老 parent 的 parent, 用于后续遍历
            KeyFrame *pOldParent = pNewChild->GetParent(); // 将 pNewChild 的父关键帧命名为 pOldParent

            // 把 new parent 设置为 new child 的 parent (父子关系互换)
            pNewChild->ChangeParent(pNewParent);

            // 指针赋值, 用于遍历下一组父子关键帧
            pNewParent = pNewChild;
            pNewChild = pOldParent;
        }

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Update the connections between the local window
        //  更新融合帧局部的连接关系
        mpMergeMatchedKF->UpdateConnections();
        // cout << "MERGE-VISUAL: Essential graph rebuilded" << endl;

        // 重新拿到融合帧局部的共视帧窗窗口
        vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
        // ! bug，这里前面已经做过了，所以这里重复添加了两遍点
        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // Project MapPoints observed in the neighborhood of the merge keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        // std::cout << "[Merge]: start fuse points" << std::endl;

        // Step 5 把融合关键帧的共视窗口里的地图点投到当前关键帧的共视窗口里，把重复的点融合掉(以旧换新)
        SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);
        // std::cout << "[Merge]: fuse points finished" << std::endl;

        // Update connectivity
        // Step 6 因为融合导致地图点变化。需要更新关键帧中图的连接关系
        // 更新当前关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi : spLocalWindowKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }
        // 更新融合关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }

        // std::cout << "[Merge]: Start welding bundle adjustment" << std::endl;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartWeldingBA = std::chrono::steady_clock::now();

        double timeMergeMaps = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_StartWeldingBA - time_StartMerge).count();
        vdMergeMaps_ms.push_back(timeMergeMaps);
#endif

        bool bStop = false;
        // 为Local BA的接口, 把set转为vector
        // Step 7 在缝合(Welding)区域进行local BA
        vpLocalCurrentWindowKFs.clear(); // 当前关键帧的窗口
        vpMergeConnectedKFs.clear();     // 融合关键帧的窗口
        std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
        std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
        if (mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD)
        {
            //! 没有用的代码
            Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop, pCurrentMap, vCorrectedSim3);
        }
        else
        {
            // 运行的 welding BA , 优化所有的当前关键帧共视窗口里的关键帧和地图点, 固定所有融合帧共视窗口里的帧
            Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs, &bStop);
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndWeldingBA = std::chrono::steady_clock::now();

        double timeWeldingBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndWeldingBA - time_StartWeldingBA).count();
        vdWeldingBA_ms.push_back(timeWeldingBA);
#endif
        // std::cout << "[Merge]: Welding bundle adjustment finished" << std::endl;

        // Loop closed. Release Local Mapping.
        // 在 welding BA 之后，我们就可以继续建图了。先做局部窗口的 BA，然后放开局部建图，再做地图剩下部分的矫正、本质图优化，可以提高实时性
        mpLocalMapper->Release();

        Verbose::PrintMess("MERGE: Finish the LBA", Verbose::VERBOSITY_DEBUG);

        // Step 8 在当前帧整个剩下的地图中（局部窗口外，认为没那么紧急处理）对位姿和地图点进行矫正传播
        // Update the non critical area from the current map to the merged map
        //  把前面优位姿优化的结果传递到地图中其他的关键帧
        vector<KeyFrame *> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
        vector<MapPoint *> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

        if (vpCurrentMapKFs.size() == 0)
        {
        }
        else
        {
            if (mpTracker->mSensor == System::MONOCULAR)
            {
                // 锁住当前地图
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

                // Step 8.1 拿到当前帧所在地图里的所有关键帧, 重复之前的过程, 不过这里是对于地图里剩余的所有的关键帧
                for (KeyFrame *pKFi : vpCurrentMapKFs)
                {
                    if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }

                    g2o::Sim3 g2oCorrectedSiw;

                    Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                    // Pose without correction
                    vNonCorrectedSim3[pKFi] = g2oSiw;

                    Sophus::SE3d Tic = Tiw * Twc;
                    g2o::Sim3 g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
                    g2oCorrectedSiw = g2oSim * mg2oMergeScw;
                    vCorrectedSim3[pKFi] = g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    double s = g2oCorrectedSiw.scale();

                    pKFi->mfScale = s;

                    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

                    pKFi->mTcwBefMerge = pKFi->GetPose();
                    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                    pKFi->SetPose(correctedTiw.cast<float>());

                    if (pCurrentMap->isImuInitialized())
                    {
                        Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                        pKFi->SetVelocity(Rcor * pKFi->GetVelocity()); // TODO: should add here scale s
                    }
                }
                for (MapPoint *pMPi : vpCurrentMapMPs)
                {
                    if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pCurrentMap)
                        continue;

                    KeyFrame *pKFref = pMPi->GetReferenceKeyFrame();
                    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                    // Project with non-corrected pose and project back with corrected pose
                    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                    Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
                    pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

                    pMPi->UpdateNormalAndDepth();
                }
            }

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            // Optimize graph (and update the loop position for each element form the begining to the end)
            // Step 8.2 本质图优化
            if (mpTracker->mSensor != System::MONOCULAR)
            {
                // 固定 : 所有融合帧共视窗口内的关键帧 + 所有当前关键帧共视窗口内的关键帧
                // 优化:  当前关键帧所在地图里的所有关键帧(除了当前关键帧共视窗口内的关键帧) + 当前地图里的所有地图点
                Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
            }

            {
                // Get Merge Map Mutex
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
                unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);     // We remove the Kfs and MPs in the merged area from the old map

                // std::cout << "Merge outside KFs: " << vpCurrentMapKFs.size() << std::endl;
                //  确保融合后信息被更新
                for (KeyFrame *pKFi : vpCurrentMapKFs)
                {
                    if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }
                    // std::cout << "KF id: " << pKFi->mnId << std::endl;

                    // Make sure connections are updated
                    // 当前关键帧所在地图更新为融合地图，并建立关联。并从当前地图中删掉
                    pKFi->UpdateMap(pMergeMap);
                    pMergeMap->AddKeyFrame(pKFi);
                    pCurrentMap->EraseKeyFrame(pKFi);
                }

                for (MapPoint *pMPi : vpCurrentMapMPs)
                {
                    if (!pMPi || pMPi->isBad())
                        continue;

                    pMPi->UpdateMap(pMergeMap);
                    pMergeMap->AddMapPoint(pMPi);
                    pCurrentMap->EraseMapPoint(pMPi);
                }
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndOptEss = std::chrono::steady_clock::now();

        double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndOptEss - time_EndWeldingBA).count();
        vdMergeOptEss_ms.push_back(timeOptEss);
#endif

        // Essential graph 优化后可以重新开始局部建图了
        mpLocalMapper->Release();

        // 如果之前停掉了全局的 BA，就开启全局 BA
        // 这里没有 IMU，所以 isImuInitialized 一定是 false，所以第二个条件（当前地图关键帧数量小于 200 且地图只有一个）一定是 true
        // 全局的BA，后面一串的判断都为true
        // !pCurrentMap->isImuInitialized()一定是true
        // Step 9 全局BA
        if (bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)))
        {
            // Launch a new thread to perform Global Bundle Adjustment
            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            // 执行全局BA
            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pMergeMap, mpCurrentKF->mnId);
        }

        // 添加融合边(这里不是参与优化的边,只是记录)
        mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
        mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

        pCurrentMap->IncreaseChangeIndex();
        pMergeMap->IncreaseChangeIndex();

        // altas 移除不好的地图
        mpAtlas->RemoveBadMaps();
    }

    // note2 作用：【视觉 + IMU】地图融合及优化！
    void LoopClosing::MergeLocal2()
    {
        cout << "Merge detected!!!!" << endl;

        //  没用上
        int numTemporalKFs = 11; // TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

        // Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        // 用来重新构造 Essential Graph
        KeyFrame *pNewChild;
        KeyFrame *pNewParent;

        // 没用上
        vector<KeyFrame *> vpLocalCurrentWindowKFs;
        vector<KeyFrame *> vpMergeConnectedKFs;

        // 记录用初始 Sim3 计算出来的当前关键帧局部共视帧窗口内的所有关键帧【矫正前】的值和【矫正后】的初始值
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // NonCorrectedSim3[mpCurrentKF] = mg2oLoopScw;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        // 记录要不要重新进行全局 ba
        bool bRelaunchBA = false;

        // cout << "Check Full Bundle Adjustment" << endl;

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        //  Step 1 如果正在进行全局 BA，停掉它
        if (isRunningGBA())
        {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if (mpThreadGBA)
            {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true;
        }

        // cout << "Request Stop Local Mapping" << endl;

        // 暂停局部建图线程，直到完全停止
        mpLocalMapper->RequestStop();

        // 等待直到完全停掉
        while (!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // cout << "Local Map stopped" << endl;

        // 当前关键帧地图的指针
        Map *pCurrentMap = mpCurrentKF->GetMap();

        // 融合关键帧地图的指针
        Map *pMergeMap = mpMergeMatchedKF->GetMap();

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 2 利用前面计算的坐标系变换位姿，把整个当前地图（关键帧及地图点）变换到融合帧所在地图中
        {
            // 把当前关键帧所在的地图位姿，带到融合关键帧所在的地图中
            // mSold_new = gSw2w1 记录的是当前关键帧世界坐标系到融合关键帧世界坐标系的变换
            float s_on = mSold_new.scale();
            Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

            // * 下面两行代码是书上的用法，把变换矩阵 T 拆开成 R 和 t 了
            // cv::Mat R_on = Converter::toCvMat(mSold_new.rotation().toRotationMatrix());
            // cv::Mat t_on = Converter::toCvMat(mSold_new.translation());

            // 锁住 altas 是为了更新地图
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

            // cout << "KFs before empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;

            // 清空队列里还没来得及处理的关键帧
            mpLocalMapper->EmptyQueue();
            // cout << "KFs after empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            // cout << "updating active map to merge reference" << endl;

            // cout << "curr merge KF id: " << mpCurrentKF->mnId << endl;

            // cout << "curr tracking KF id: " << mpTracker->GetLastKeyFrame()->mnId << endl;

            // 是否将尺度更新的标志
            bool bScaleVel = false;
            if (s_on != 1) // ? 判断浮点数和 1 严格相等是不是不合适？
                bScaleVel = true;

            // 利用 mSold_new 位姿把整个当前地图中的关键帧和地图点变换到--融合帧所在地图--的坐标系下
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);

            // 将尺度更新到普通帧位姿
            mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());

            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        }

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 3 如果当前地图 IMU 没有完全初始化，则帮助 IMU 快速优化，并强制设置为 IMU 已经完成初始化
        // 反正都要融合了，这里就拔苗助长完成 IMU 优化，回头直接全部放到融合地图里就好了
        const int numKFnew = pCurrentMap->KeyFramesInMap();

        if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD) && !pCurrentMap->GetIniertialBA2())
        {
            // 进入 if 语句，表示地图中，IMU 还没有完全初始化
            Eigen::Vector3d bg, ba;
            bg << 0., 0., 0.; // ? 是不是有点 low 啊？这个写法？
            ba << 0., 0., 0.;

            // 优化当前地图中的零偏参数 bg、ba
            Optimizer::InertialOptimization(pCurrentMap, bg, ba);
            IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

            // 用优化得到的 bias（零偏） 更新普通帧位姿
            mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

            // 强制设置 IMU 已经完成初始化
            pCurrentMap->SetIniertialBA2();
            pCurrentMap->SetIniertialBA1();
            pCurrentMap->SetImuInitialized();
        }

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        // Load KFs and MPs from merge map

        // cout << "updating current map" << endl;

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 4 地图以旧换新。把融合帧所在地图里的关键帧和地图点从原地图里删掉，变更为当前关键帧所在地图。
        {
            // 地图加互斥锁，这里会停止跟踪线程！
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate);     // We remove the Kfs and MPs in the merged area from the old map

            // 取出融合帧所在地图的所有关键帧和地图点
            vector<KeyFrame *> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
            vector<MapPoint *> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

            // 遍历每个融合帧所在地图的关键帧
            for (KeyFrame *pKFi : vpMergeMapKFs)
            {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
                {
                    continue;
                }

                // Make sure connections are updated
                // 把该关键帧从融合帧所在地图中删掉，加入到当前的地图中
                pKFi->UpdateMap(pCurrentMap);
                pCurrentMap->AddKeyFrame(pKFi);
                pMergeMap->EraseKeyFrame(pKFi);
            }

            // 遍历每个融合帧所在地图的地图点
            for (MapPoint *pMPi : vpMergeMapMPs)
            {
                if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                    continue;

                // 把地图点添加到当前帧所在地图中，从融合帧所在地图删掉
                pMPi->UpdateMap(pCurrentMap);
                pCurrentMap->AddMapPoint(pMPi);
                pMergeMap->EraseMapPoint(pMPi);
            }
            // ? BUG! pMergeMap 没有设置为 BAD
            // ? 应该加入如下代码吧？
            // ? mpAtlas->SetMapBad(pMergeMap);

            // Save non corrected poses (already merged maps)
            // 存下所有关键帧在融合矫正之前的位姿
            vector<KeyFrame *> vpKFs = pCurrentMap->GetAllKeyFrames();
            for (KeyFrame *pKFi : vpKFs)
            {
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                // * 上面两句虽然没报错，但是书里也是用的 cv::Mat 类型的，即下面四行代码👇
                // cv::Mat Tiw = pKFi->GetPose();
                // cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
                // cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
                // g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);

                NonCorrectedSim3[pKFi] = g2oSiw;
            }
        }

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        // cout << "end updating current map" << endl;

        // Critical zone
        // bool good = pCurrentMap->CheckEssentialGraph();

        /*if(!good)
            cout << "BAD ESSENTIAL GRAPH!!" << endl;*/

        // cout << "Update essential graph" << endl;

        // mpCurrentKF->UpdateConnections(); // to put at false mbFirstConnection

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        // Step 5 融合新旧地图的生成树
        pMergeMap->GetOriginKF()->SetFirstConnection(false);

        // 将父节点命名为 pNewChild
        pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF

        // 记别名为 pNewParent
        pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)

        // 将融合关键帧父节点更换为当前帧
        mpMergeMatchedKF->ChangeParent(mpCurrentKF);

        // 开始--反向--遍历整个地图
        while (pNewChild)
        {
            // 删除父子关系
            pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop

            // 将 pNewChild 的父关键帧命名为 pOldParent
            KeyFrame *pOldParent = pNewChild->GetParent();
            pNewChild->ChangeParent(pNewParent); // 父子关系互换
            pNewParent = pNewChild;              // 指针赋值，用于遍历下一组父子关键帧
            pNewChild = pOldParent;
        }

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        // cout << "end update essential graph" << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 1!!" << endl;*/

        // cout << "Update relationship between KFs" << endl;

        vector<MapPoint *> vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
        vector<KeyFrame *> vpCurrentConnectedKFs;

        // 下面都是为后续 SearchAndFuse 准备数据👇

        // todo1：拿出融合帧的局部窗口, 确保最后是(1+5), 1: 融合帧自己 5: 5个共视关键帧
        mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
        vector<KeyFrame *> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
        if (mvpMergeConnectedKFs.size() > 6)
            mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6, mvpMergeConnectedKFs.end());

        /*
        mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
        */

        // todo2：拿出当前关键帧的局部窗口, 确保最后是(1+5), 1: 融合帧自己 5: 5个共视关键帧
        mpCurrentKF->UpdateConnections();
        vpCurrentConnectedKFs.push_back(mpCurrentKF);

        /*
        vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.push_back(mpCurrentKF);
        */

        aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
        if (vpCurrentConnectedKFs.size() > 6)
            vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6, vpCurrentConnectedKFs.end());

        // 取出所有融合帧局部窗口的地图点，设置上限为【1000】
        set<MapPoint *> spMapPointMerge;
        for (KeyFrame *pKFi : mvpMergeConnectedKFs)
        {
            set<MapPoint *> vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
            if (spMapPointMerge.size() > 1000)
                break;
        }

        /*
        cout << "vpCurrentConnectedKFs.size() " << vpCurrentConnectedKFs.size() << endl;
        cout << "mvpMergeConnectedKFs.size() " << mvpMergeConnectedKFs.size() << endl;
        cout << "spMapPointMerge.size() " << spMapPointMerge.size() << endl;
        */

        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // cout << "Finished to update relationship between KFs" << endl;

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*
        good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 2!!" << endl;
        */

        // cout << "start SearchAndFuse" << endl;

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        //  Step 6 把融合关键帧的共视窗口里的地图点投到当前关键帧的共视窗口里，把重复的点融合掉（以旧换新）
        SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

        // cout << "end SearchAndFuse" << endl;

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 3!!" << endl;

        cout << "Init to update connections" << endl;*/

        // 1.更新【当前关键帧】共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi : vpCurrentConnectedKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }

        // 2.更新【融合关键帧】共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi : mvpMergeConnectedKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateConnections();
        }

        // cout << "end update connections" << endl;

        // cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 4!!" << endl;*/

        // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
        if (numKFnew < 10)
        {
            mpLocalMapper->Release();
            return;
        }

        /*good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 5!!" << endl;*/

        // Perform BA
        bool bStopFlag = false;
        KeyFrame *pCurrKF = mpTracker->GetLastKeyFrame();

        // cout << "start MergeInertialBA" << endl;

        // Boundary--------------------------------------------------------------------------------------------------------------------------------

        //  Step 7 针对缝合区域的窗口内进行进行 welding（熔接） BA 优化
        Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);

        // cout << "end MergeInertialBA" << endl;

        /*
        good = pCurrentMap->CheckEssentialGraph();
        if(!good)
            cout << "BAD ESSENTIAL GRAPH 6!!" << endl;
        */

        // 释放局部建图线程
        mpLocalMapper->Release();

        return;
    }

    // 1.0 版本新的调试函数，暂时跳过
    void LoopClosing::CheckObservations(set<KeyFrame *> &spKFsMap1, set<KeyFrame *> &spKFsMap2)
    {
        cout << "----------------------" << endl;
        for (KeyFrame *pKFi1 : spKFsMap1)
        {
            map<KeyFrame *, int> mMatchedMP;
            set<MapPoint *> spMPs = pKFi1->GetMapPoints();

            for (MapPoint *pMPij : spMPs)
            {
                if (!pMPij || pMPij->isBad())
                {
                    continue;
                }

                map<KeyFrame *, tuple<int, int>> mMPijObs = pMPij->GetObservations();
                for (KeyFrame *pKFi2 : spKFsMap2)
                {
                    if (mMPijObs.find(pKFi2) != mMPijObs.end())
                    {
                        if (mMatchedMP.find(pKFi2) != mMatchedMP.end())
                        {
                            mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                        }
                        else
                        {
                            mMatchedMP[pKFi2] = 1;
                        }
                    }
                }
            }

            if (mMatchedMP.size() == 0)
            {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
            }
            else
            {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size() << " KF from the other map" << endl;
                for (pair<KeyFrame *, int> matchedKF : mMatchedMP)
                {
                    cout << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
                }
            }
        }
        cout << "----------------------" << endl;
    }

    // todo 查找对应 MP 与融合
    /**
     * @brief 查找对应MP与融合
     * @param CorrectedPosesMap 关键帧及对应的pose
     * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
     */
    void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint *> &vpMapPoints)
    {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        // 遍历每个关键帧
        // cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" << endl;
        // cout << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;
        for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++)
        {
            int num_replaces = 0;
            KeyFrame *pKFi = mit->first;
            Map *pMap = pKFi->GetMap();

            g2o::Sim3 g2oScw = mit->second;
            Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));

            // 新点表示pKFi对应的点，老点表示pKFi对应的回环点
            // 将vpMapPoints投到pKF里面看看有没有匹配的MP，如果没有直接添加，如果有，暂时将老点放入至vpReplacePoints
            // vpReplacePoints下标表示第n个vpMapPoints，存放着新点，可以直接找到对应信息
            int numFused = matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            // 更新点
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++)
            {
                // vpReplacePoints如果存在新点，则替换成老点，这里注意如果老点已经在新点对应的kf中
                // 也就是之前某次matcher.Fuse 把老点放入到新的关键帧中，下次遍历时，如果老点已经在被代替点的对应的某一个关键帧内
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep)
                {

                    num_replaces += 1;
                    // 替换掉较新的
                    pRep->Replace(vpMapPoints[i]);
                }
            }

            total_replaces += num_replaces;
        }
        // cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
    }

    /**
     * @brief 查找对应MP与融合，与上个函数类似
     * @param vConectedKFs 当前地图的当前关键帧及5个共视关键帧
     * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
     */
    void LoopClosing::SearchAndFuse(const vector<KeyFrame *> &vConectedKFs, vector<MapPoint *> &vpMapPoints)
    {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        // cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
        // cout << "FUSE-POSE: Intially there are " << vConectedKFs.size() << " KFs" << endl;
        for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++)
        {
            int num_replaces = 0;
            KeyFrame *pKF = (*mit);
            Map *pMap = pKF->GetMap();
            Sophus::SE3f Tcw = pKF->GetPose();
            Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
            Scw.setScale(1.f);
            /*std::cout << "These should be zeros: " <<
                Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
                Scw.translation() - Tcw.translation() << std::endl <<
                Scw.scale() - 1.f << std::endl;*/
            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));
            matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++)
            {
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep)
                {
                    num_replaces += 1;
                    pRep->Replace(vpMapPoints[i]);
                }
            }
            /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << " MPs fused" << endl;
            total_replaces += num_replaces;*/
        }
        // cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
    }

    // todo 由外部线程调用，请求复位当前线程
    void LoopClosing::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(5000);
        }
    }

    void LoopClosing::RequestResetActiveMap(Map *pMap)
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetActiveMapRequested = true;
            mpMapToReset = pMap;
        }

        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetActiveMapRequested)
                    break;
            }
            usleep(3000);
        }
    }

    // todo 当前线程调用，检查是否有外部线程请求复位当前线程，如果有的话就复位回环检测线程
    void LoopClosing::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        // 如果有来自于外部的线程的复位请求,那么就复位当前线程
        if (mbResetRequested)
        {
            cout << "Loop closer reset requested..." << endl;
            // 清空参与和进行回环检测的关键帧队列
            mlpLoopKeyFrameQueue.clear();
            // 上一次没有和任何关键帧形成闭环关系
            mLastLoopKFid = 0; // TODO old variable, it is not use in the new algorithm
            // 复位请求标志复位
            mbResetRequested = false;
            mbResetActiveMapRequested = false;
        }
        else if (mbResetActiveMapRequested)
        {

            for (list<KeyFrame *>::const_iterator it = mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
            {
                KeyFrame *pKFi = *it;
                if (pKFi->GetMap() == mpMapToReset)
                {
                    it = mlpLoopKeyFrameQueue.erase(it);
                }
                else
                    ++it;
            }

            mLastLoopKFid = mpAtlas->GetLastInitKFid(); // TODO old variable, it is not use in the new algorithm
            mbResetActiveMapRequested = false;
        }
    }

    /**
     * @brief MergeLocal CorrectLoop 中调用
     * @param pActiveMap 当前地图
     * @param nLoopKF 检测到回环成功的关键帧，不是与之匹配的老关键帧
     */
    void LoopClosing::RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF)
    {
        Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();

        nFGBA_exec += 1;

        vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
        vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

        // imu 初始化成功才返回true，只要一阶段成功就为true
        const bool bImuInit = pActiveMap->isImuInitialized();

        if (!bImuInit)
            Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
        else
            // 仅有一个地图且内部关键帧<200，并且IMU完成了第一阶段初始化后才会进行下面
            Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndGBA = std::chrono::steady_clock::now();

        double timeGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndGBA - time_StartFGBA).count();
        vdGBA_ms.push_back(timeGBA);

        if (mbStopGBA)
        {
            nFGBA_abort += 1;
        }
#endif
        // 记录GBA已经迭代次数,用来检查全局BA过程是否是因为意外结束的
        int idx = mnFullBAIdx;
        // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

        // Update all MapPoints and KeyFrames
        // Local Mapping was active during BA, that means that there might be new keyframes
        // not included in the Global BA and they are not consistent with the updated map.
        // We need to propagate the correction through the spanning tree
        {
            unique_lock<mutex> lock(mMutexGBA);
            if (idx != mnFullBAIdx)
                return;

            if (!bImuInit && pActiveMap->isImuInitialized())
                return;

            if (!mbStopGBA)
            {
                Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
                Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

                mpLocalMapper->RequestStop();
                // Wait until Local Mapping has effectively stopped

                while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                {
                    usleep(1000);
                }

                // Get Map Mutex
                unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                // cout << "LC: Update Map Mutex adquired" << endl;

                // pActiveMap->PrintEssentialGraph();
                //  Correct keyframes starting at map first keyframe
                list<KeyFrame *> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(), pActiveMap->mvpKeyFrameOrigins.end());

                // 通过树的方式更新未参与全局优化的关键帧，一个关键帧与其父节点的共视点数最多，所以选其作为参考帧
                while (!lpKFtoCheck.empty())
                {
                    KeyFrame *pKF = lpKFtoCheck.front();
                    const set<KeyFrame *> sChilds = pKF->GetChilds();
                    // cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                    // cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                    Sophus::SE3f Twc = pKF->GetPoseInverse();
                    // cout << "Twc: " << Twc << endl;
                    // cout << "GBA: Correct KeyFrames" << endl;
                    //  广度优先搜索
                    for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
                    {
                        KeyFrame *pChild = *sit;
                        if (!pChild || pChild->isBad())
                            continue;

                        // 专门处理没有参与优化的新关键帧
                        if (pChild->mnBAGlobalForKF != nLoopKF)
                        {
                            // cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                            // cout << " child id: " << pChild->mnId << endl;
                            Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                            // cout << "Child pose: " << Tchildc << endl;
                            // cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;

                            Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                            if (pChild->isVelocitySet())
                            {
                                pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                            }
                            else
                                Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);

                            // cout << "Child bias: " << pChild->GetImuBias() << endl;
                            pChild->mBiasGBA = pChild->GetImuBias();

                            pChild->mnBAGlobalForKF = nLoopKF; // 标记成更新过的
                        }
                        lpKFtoCheck.push_back(pChild);
                    }

                    // cout << "-------Update pose" << endl;
                    pKF->mTcwBefGBA = pKF->GetPose();
                    // cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                    pKF->SetPose(pKF->mTcwGBA);
                    /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                    cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                    double dist = cv::norm(trasl);
                    cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                    double desvX = 0;
                    double desvY = 0;
                    double desvZ = 0;
                    if(pKF->mbHasHessian)
                    {
                        cv::Mat hessianInv = pKF->mHessianPose.inv();

                        double covX = hessianInv.at<double>(3,3);
                        desvX = std::sqrt(covX);
                        double covY = hessianInv.at<double>(4,4);
                        desvY = std::sqrt(covY);
                        double covZ = hessianInv.at<double>(5,5);
                        desvZ = std::sqrt(covZ);
                        pKF->mbHasHessian = false;
                    }
                    if(dist > 1)
                    {
                        cout << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                        cout << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                        cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                        string strNameFile = pKF->mNameFile;
                        cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

                        cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

                        vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
                        int num_MPs = 0;
                        for(int i=0; i<vpMapPointsKF.size(); ++i)
                        {
                            if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                            {
                                continue;
                            }
                            num_MPs += 1;
                            string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                            cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                            cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                        }
                        cout << "--It has " << num_MPs << " MPs matched in the map" << endl;

                        string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                        cv::imwrite(namefile, imLeft);
                    }*/

                    if (pKF->bImu)
                    {
                        // cout << "-------Update inertial values" << endl;
                        pKF->mVwbBefGBA = pKF->GetVelocity();
                        // if (pKF->mVwbGBA.empty())
                        //     Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                        // assert(!pKF->mVwbGBA.empty());
                        pKF->SetVelocity(pKF->mVwbGBA);
                        pKF->SetNewBias(pKF->mBiasGBA);
                    }

                    lpKFtoCheck.pop_front();
                }

                // cout << "GBA: Correct MapPoints" << endl;
                //  Correct MapPoints
                //  更新mp点
                const vector<MapPoint *> vpMPs = pActiveMap->GetAllMapPoints();

                for (size_t i = 0; i < vpMPs.size(); i++)
                {
                    MapPoint *pMP = vpMPs[i];

                    if (pMP->isBad())
                        continue;

                    // NOTICE 并不是所有的地图点都会直接参与到全局BA优化中,但是大部分的地图点需要根据全局BA优化后的结果来重新纠正自己的位姿
                    // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
                    if (pMP->mnBAGlobalForKF == nLoopKF)
                    {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    }
                    else // 如故这个地图点并没有直接参与到全局BA优化的过程中,那么就使用器参考关键帧的新位姿来优化自己的位姿
                    {
                        // Update according to the correction of its reference keyframe
                        // 说明这个关键帧，在前面的过程中也没有因为“当前关键帧”得到全局BA优化
                        //? 可是,为什么会出现这种情况呢? 难道是因为这个地图点的参考关键帧设置成为了bad?
                        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                        if (pRefKF->mnBAGlobalForKF != nLoopKF)
                            continue;

                        /*if(pRefKF->mTcwBefGBA.empty())
                            continue;*/

                        // Map to non-corrected camera
                        // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        // 转换到其参考关键帧相机坐标系下的坐标
                        Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                        // Backproject using corrected camera
                        // 然后使用已经纠正过的参考关键帧的位姿,再将该地图点变换到世界坐标系下
                        pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                    }
                }

                pActiveMap->InformNewBigChange();
                pActiveMap->IncreaseChangeIndex();

                // TODO Check this update
                // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

                mpLocalMapper->Release();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();

                double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndUpdateMap - time_EndGBA).count();
                vdUpdateMap_ms.push_back(timeUpdateMap);

                double timeFGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndUpdateMap - time_StartFGBA).count();
                vdFGBATotal_ms.push_back(timeFGBA);
#endif
                Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
            }

            mbFinishedGBA = true;
            mbRunningGBA = false;
        }
    }

    // todo 由外部线程调用，请求终止当前线程
    void LoopClosing::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        // cout << "LC: Finish requested" << endl;
        mbFinishRequested = true;
    }

    // todo 当前线程调用，查看是否有外部线程请求当前线程
    bool LoopClosing::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    // todo 由当前线程调用，执行完成该函数之后线程主函数退出，线程销毁
    void LoopClosing::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    // todo 由外部线程调用，判断当前回环检测线程是否已经正确终止了
    bool LoopClosing::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

} // namespace ORB_SLAM
