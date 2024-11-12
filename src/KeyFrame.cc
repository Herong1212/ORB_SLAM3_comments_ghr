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

#include "KeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include <mutex>

namespace ORB_SLAM3
{

    long unsigned int KeyFrame::nNextId = 0;

    KeyFrame::KeyFrame()
        : mnFrameId(0), mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
          mfGridElementWidthInv(0), mfGridElementHeightInv(0),
          mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
          mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
          fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
          mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint>>(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint>>(NULL)),
          mvuRight(static_cast<vector<float>>(NULL)), mvDepth(static_cast<vector<float>>(NULL)), mnScaleLevels(0), mfScaleFactor(0),
          mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0), mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
          mnMaxY(0), mPrevKF(static_cast<KeyFrame *>(NULL)), mNextKF(static_cast<KeyFrame *>(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
          mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0),
          NLeft(0), NRight(0), mnNumberOfOpt(0), mbHasVelocity(false)
    {
    }

    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB)
        : bImu(pMap->isImuInitialized()), mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
          mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
          mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
          mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
          fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
          mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
          mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
          mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
          mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
          mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
          mnMaxY(F.mnMaxY), mK_(F.mK_), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
          mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
          mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
          mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
          mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
          mvLeftToRightMatch(F.mvLeftToRightMatch), mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
          mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0), mbHasVelocity(false)
    {
        mnId = nNextId++;

        // 根据指定的普通帧, 初始化用于加速匹配的网格对象信息; 其实就把每个网格中有的特征点的索引复制过来
        mGrid.resize(mnGridCols);
        if (F.Nleft != -1)
            mGridRight.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++)
        {
            mGrid[i].resize(mnGridRows);
            if (F.Nleft != -1)
                mGridRight[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
            {
                mGrid[i][j] = F.mGrid[i][j];
                if (F.Nleft != -1)
                {
                    mGridRight[i][j] = F.mGridRight[i][j];
                }
            }
        }

        if (!F.HasVelocity())
        {
            mVw.setZero();
            mbHasVelocity = false;
        }
        else
        {
            mVw = F.GetVelocity();
            mbHasVelocity = true;
        }

        mImuBias = F.mImuBias;
        SetPose(F.GetPose());

        mnOriginMapId = pMap->GetId();
    }

    // note：函数的主要功能：利用词汇表（BoW 词袋模型）将图像的 ORB 特征描述子（mDescriptors）转换为词袋向量（mBowVec）和特征向量（mFeatVec），用于后续的图像匹配和定位。
    void KeyFrame::ComputeBoW()
    {
        // 只有当【词袋向量】或者【节点和特征序号的特征向量】为空的时候执行词袋向量的计算。这可以避免重复计算，提高效率。
        if (mBowVec.empty() || mFeatVec.empty())
        {
            // 将描述子 mDescriptors 转换为 DBOW 要求的输入格式，即把 mDescriptors pushback 到 currentDesc 中
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);

            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            // 将当前特征点的描述子转换成词袋向量 mBowVec 以及节点与索引的特征向量 mFeatVec
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    // 设置当前关键帧的位姿
    void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
    {
        unique_lock<mutex> lock(mMutexPose);

        mTcw = Tcw;
        mRcw = mTcw.rotationMatrix();
        mTwc = mTcw.inverse();
        mRwc = mTwc.rotationMatrix();

        if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
        {
            mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
        }
    }

    void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
    {
        unique_lock<mutex> lock(mMutexPose);
        mVw = Vw;
        mbHasVelocity = true;
    }

    // 获取位姿
    Sophus::SE3f KeyFrame::GetPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTcw;
    }

    // 获取位姿的逆
    Sophus::SE3f KeyFrame::GetPoseInverse()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTwc;
    }

    // 获取(左目)相机的中心在世界坐标系下的坐标
    Eigen::Vector3f KeyFrame::GetCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTwc.translation();
    }

    Eigen::Vector3f KeyFrame::GetImuPosition()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mOwb;
    }

    Eigen::Matrix3f KeyFrame::GetImuRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return (mTwc * mImuCalib.mTcb).rotationMatrix();
    }

    Sophus::SE3f KeyFrame::GetImuPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTwc * mImuCalib.mTcb;
    }

    Eigen::Matrix3f KeyFrame::GetRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mRcw;
    }

    Eigen::Vector3f KeyFrame::GetTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTcw.translation();
    }

    Eigen::Vector3f KeyFrame::GetVelocity()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mVw;
    }

    bool KeyFrame::isVelocitySet()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mbHasVelocity;
    }

    // 为关键帧之间添加或更新连接
    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
    {
        {
            // 如果被占用就一直等着,这个添加连接的操作不能够被放弃
            unique_lock<mutex> lock(mMutexConnections);

            // 判断当前关键帧是否已经和其他的关键帧创建了联系
            // std::map::count函数只可能返回0或1两种情况

            // count函数返回0，mConnectedKeyFrameWeights中没有pKF，之前没有连接
            if (!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight) // 之前连接的权重不一样，更新
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        // 如果添加了更新的连接关系就要更新一下,主要是重新进行排序
        UpdateBestCovisibles();
    }

    /**
     * @brief 按照权重对连接的关键帧进行排序
     *
     * 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
     */
    void KeyFrame::UpdateBestCovisibles()
    {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的3D点数放在前面，利于排序
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        // 按照权重进行排序（默认是从小到大）
        sort(vPairs.begin(), vPairs.end());

        // 为什么要用链表保存？因为插入和删除操作方便，只需要修改上一节点位置，不需要移动其他元素
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
        {
            if (!vPairs[i].second->isBad())
            {
                // push_front 后变成从大到小
                lKFs.push_front(vPairs[i].second);
                lWs.push_front(vPairs[i].first);
            }
        }

        // 权重从大到小
        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    // 得到与该关键帧连接（>15个共视地图点）的关键帧(没有排序的)
    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);
        return s;
    }

    // 得到与该关键帧连接的关键帧(已按权值排序)
    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    // 得到与该关键帧连接的前N个关键帧(已按权值排序)
    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
        unique_lock<mutex> lock(mMutexConnections);
        // 如果不够达到的数目就直接吧现在所有的关键帧都返回了
        if ((int)mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
    }

    // 得到与该关键帧连接的权重大于等于w的关键帧
    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w)
    {
        unique_lock<mutex> lock(mMutexConnections);

        // 如果没有和当前关键帧连接的关键帧
        if (mvpOrderedConnectedKeyFrames.empty())
        {
            return vector<KeyFrame *>();
        }

        // 从mvOrderedWeights找出第一个大于w的那个迭代器
        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);

        // 如果没有找到(最大的权重也比给定的阈值小)
        if (it == mvOrderedWeights.end() && mvOrderedWeights.back() < w)
        {
            return vector<KeyFrame *>();
        }
        else
        {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    // 得到该关键帧与pKF的权重
    int KeyFrame::GetWeight(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexConnections);

        // 没有连接的话权重也就是共视点个数就是0
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    int KeyFrame::GetNumberMPs()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        int numberMPs = 0;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++)
        {
            if (!mvpMapPoints[i])
                continue;
            numberMPs++;
        }
        return numberMPs;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }

    /**
     * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,将该地图点置为NULL
     *
     * @param[in] idx   地图点在该关键帧中的id
     */
    void KeyFrame::EraseMapPointMatch(const int &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP)
    {
        tuple<size_t, size_t> indexes = pMP->GetIndexInKeyFrame(this);
        size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        if (leftIndex != -1)
            mvpMapPoints[leftIndex] = static_cast<MapPoint *>(NULL);
        if (rightIndex != -1)
            mvpMapPoints[rightIndex] = static_cast<MapPoint *>(NULL);
    }

    // 地图点的替换
    void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint *pMP)
    {
        mvpMapPoints[idx] = pMP;
    }

    // 获取当前关键帧中的所有地图点
    set<MapPoint *> KeyFrame::GetMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++)
        {
            // 判断是否被删除了
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            // 如果是没有来得及删除的坏点也要进行这一步
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    // 关键帧中，大于等于最少观测数目minObs的MapPoints的数量.这些特征点被认为追踪到了
    int KeyFrame::TrackedMapPoints(const int &minObs)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        // 是否检查数目
        const bool bCheckObs = minObs > 0;
        // N是当前帧中特征点的个数
        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (bCheckObs)
                    {
                        // 满足输入阈值要求的地图点计数加1
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    // 获取当前关键帧的具体的地图点
    vector<MapPoint *> KeyFrame::GetMapPointMatches()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    // 获取当前关键帧的具体的某个地图点
    MapPoint *KeyFrame::GetMapPoint(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    /*
     * 更新图的连接
     *
     * 1. 首先获得该关键帧的所有 MapPoint 点，统计观测到这些 3d 点的每个关键帧与其它所有关键帧之间的共视程度
     *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共 3d 点的个数。
     * 2. 并且该权重必须大于【一个阈值】，如果没有超过该阈值的权重，那么就只保留--权重最大--的边（与其它关键帧的共视程度比较高）
     * 3. 对这些连接按照权重--从大到小--进行排序，以方便将来的处理
     *    更新完 covisibility 图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
     */
    // todo 作用：每次新建关键帧时也需要新建和它相连关键帧的关系。当地图点的关键帧发生变化时，需要更新和它相连关键帧的联系👇
    void KeyFrame::UpdateConnections(bool upParent)
    {
        // ps 在没有执行这个函数之前，关键帧只和 MapPoints 之间有连接关系，这个函数可以更新关键帧之间的连接关系

        map<KeyFrame *, int> KFcounter; // 关键帧---权重，权重为其它关键帧与当前关键帧共视地图点的个数，也称为【共视程度】
        vector<MapPoint *> vpMP;

        {
            // 获得该关键帧的所有 3D 点
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints; // 将该关键帧的所有地图点放入 vpMP 容器中
        }

        // For all map points in keyframe check in which other keyframes are they seen.  Increase counter for those keyframes.
        // Step 1 通过地图点被关键帧观测来间接统计关键帧之间的共视程度
        // 统计每一个地图点都有多少关键帧与当前关键帧存在共视关系，统计结果放在 KFcounter
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            // 对于每一个 MapPoint 点，observations 记录了可以观测到该 MapPoint 的所有关键帧
            map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                if (mit->first->mnId == mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                    continue;
                // 这里的操作非常精彩！👇
                // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对；
                // mit->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数；
                // 所以最后 KFcounter 第一个参数表示某个关键帧，第二个参数表示该关键帧看到了多少当前帧的地图点，也就是【共视程度】；
                KFcounter[mit->first]++;
            }
        }

        // 没有共视关系，直接退出
        // This should not happen
        if (KFcounter.empty())
            return;

        // If the counter is greater than threshold add connection
        // In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0; // 记录最高的共视程度

        KeyFrame *pKFmax = NULL;

        // 至少有【15个】共视地图点
        int th = 15;

        // vPairs 记录与其它关键帧共视帧数【大于 th】的关键帧
        // pair<int, KeyFrame *> 将关键帧的权重写在前面，关键帧写在后面方便后面排序；
        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(KFcounter.size());
        if (!upParent)
            cout << "UPDATE_CONN: current KF " << mnId << endl;

        // Step 2 找到对应权重最大的关键帧（共视程度最高的关键帧）
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++)
        {
            if (!upParent)
                cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;

            if (mit->second > nmax)
            {
                nmax = mit->second;
                pKFmax = mit->first;
            }

            // 建立共视关系至少需要大于等于 th 个共视地图点
            if (mit->second >= th)
            {
                // 对应权重需要大于阈值，对这些关键帧建立连接
                vPairs.push_back(make_pair(mit->second, mit->first));

                // 对方关键帧也要添加这个信息
                // 更新 KFcounter 中该关键帧的 mConnectedKeyFrameWeights
                // 更新其它 KeyFrame 的 mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        //  Step 3 如果没有连接到关键（超过阈值的权重），则对权重最大的关键帧建立连接
        if (vPairs.empty())
        {
            // 如果每个关键帧与它共视的关键帧的个数都少于 th，那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
            // 这是对之前 th 这个阈值可能过高的一个补丁
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }

        //  Step 4 对共视程度比较高的关键帧对更新连接关系及权重（从大到小）
        // vPairs 里存的都是相互共视程度比较高的关键帧和共视权重，接下来--由大到小--进行排序
        sort(vPairs.begin(), vPairs.end()); // sort 函数默认--升序--排列

        // ? 将排序后的结果分别组织成为两种数据类型
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++)
        {
            // push_front 后变成了从大到小顺序(就是重新编写排序规则而已，写个 myCompare() 函数也可以是实现)
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            // 更新当前帧与其它关键帧的连接权重
            // ? bug 这里直接赋值，会把小于阈值的共视关系也放入 mConnectedKeyFrameWeights，会增加计算量
            // ? 但后续主要用 mvpOrderedConnectedKeyFrames 来取共视帧，对结果没影响
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            // Step 5 更新生成树的连接
            if (mbFirstConnection && mnId != mpMap->GetInitKFid())
            {
                // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
                mpParent = mvpOrderedConnectedKeyFrames.front();
                // 建立双向连接关系，将当前关键帧作为其子关键帧
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }
        }
    }

    // 添加子关键帧（即和子关键帧具有最大共视关系的关键帧就是当前关键帧）
    void KeyFrame::AddChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    // 删除某个子关键帧
    void KeyFrame::EraseChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    // 改变当前关键帧的父关键帧
    void KeyFrame::ChangeParent(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        if (pKF == this)
        {
            cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
            throw std::invalid_argument("The parent and child can not be the same");
        }

        mpParent = pKF;
        pKF->AddChild(this);
    }

    // 获取当前关键帧的子关键帧
    set<KeyFrame *> KeyFrame::GetChilds()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    // 获取当前关键帧的父关键帧
    KeyFrame *KeyFrame::GetParent()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    // 判断某个关键帧是否是当前关键帧的子关键帧
    bool KeyFrame::hasChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::SetFirstConnection(bool bFirst)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbFirstConnection = bFirst;
    }

    // 给当前关键帧添加回环边，回环边连接了形成闭环关系的关键帧
    void KeyFrame::AddLoopEdge(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    // 获取和当前关键帧形成闭环关系的关键帧
    set<KeyFrame *> KeyFrame::GetLoopEdges()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::AddMergeEdge(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspMergeEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetMergeEdges()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspMergeEdges;
    }

    // 设置当前关键帧不要在优化的过程中被删除. 由【回环检测线程】调用
    void KeyFrame::SetNotErase()
    {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    /**
     * @brief 删除当前的这个关键帧,表示不进行回环检测过程;由回环检测线程调用
     *
     */
    void KeyFrame::SetErase()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            // 如果当前关键帧和其他的关键帧没有形成回环关系,那么就删吧
            if (mspLoopEdges.empty())
            {
                mbNotErase = false;
            }
        }

        // mbToBeErased：删除之前记录的想要删但时机不合适没有删除的帧
        if (mbToBeErased)
        {
            SetBadFlag();
        }
    }

    /**
     * @brief 真正地执行删除关键帧的操作
     * 需要删除的是该关键帧和其他所有帧、地图点之间的连接关系
     *
     * mbNotErase作用：表示要删除该关键帧及其连接关系但是这个关键帧有可能正在回环检测或者计算sim3操作，这时候虽然这个关键帧冗余，但是却不能删除，
     * 仅设置mbNotErase为true，这时候调用setbadflag函数时，不会将这个关键帧删除，只会把mbTobeErase变成true，代表这个关键帧可以删除但不到时候,先记下来以后处理。
     * 在闭环线程里调用 SetErase()会根据mbToBeErased 来删除之前可以删除还没删除的帧。
     */
    void KeyFrame::SetBadFlag()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            // 初始关键帧不能删除
            if (mnId == mpMap->GetInitKFid())
            {
                return;
            }
            else if (mbNotErase)
            {
                // mbNotErase表示不应该删除，于是把mbToBeErased置为true，假装已经删除，其实没有删除
                mbToBeErased = true;
                return;
            }
        }

        // Step 2 遍历所有和当前关键帧共视的关键帧，删除他们与当前关键帧的联系
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
        {
            mit->first->EraseConnection(this);
        }

        // Step 3 遍历每一个当前关键帧的地图点，删除每一个地图点和当前关键帧的联系
        for (size_t i = 0; i < mvpMapPoints.size(); i++)
        {
            if (mvpMapPoints[i])
            {
                mvpMapPoints[i]->EraseObservation(this); // 让与自己有联系的MapPoint删除与自己的联系
            }
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            // 清空自己与其它关键帧之间的联系
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            // Step 4 更新生成树，主要是处理好父子关键帧，不然会造成整个关键帧维护的图断裂，或者混乱，不能够为后端提供较好的初值
            // 子关键帧候选父关键帧
            set<KeyFrame *> sParentCandidates;
            // 将当前帧的父关键帧放入候选父关键帧
            if (mpParent)
                sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            // 如果这个关键帧有自己的子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
            while (!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                // Step 4.1 遍历每一个子关键帧，让它们更新它们指向的父关键帧
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++)
                {
                    KeyFrame *pKF = *sit;
                    // 跳过无效的子关键帧
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    // Step 4.2 子关键帧遍历每一个与它共视的关键帧
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++)
                    {
                        // sParentCandidates 中刚开始存的是“爷爷”
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++)
                        {
                            if (vpConnected[i]->mnId == (*spcit)->mnId)
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                // 寻找并更新权值最大的那个共视关系
                                if (w > max)
                                {
                                    pC = pKF;            // 子关键帧
                                    pP = vpConnected[i]; // 目前和子关键帧具有最大权值的关键帧（将来的父关键帧）
                                    max = w;             // 这个最大的权值
                                    bContinue = true;    // 说明子节点找到了可以作为其新父关键帧的帧
                                }
                            }
                        }
                    }
                }

                // Step 4.4 如果在上面的过程中找到了新的父节点
                // 下面代码应该放到遍历子关键帧循环中?
                // 回答：不需要！这里while循环还没退出，会使用更新的sParentCandidates
                if (bContinue)
                {
                    // 因为父节点死了，并且子节点找到了新的父节点，就把它更新为自己的父节点
                    pC->ChangeParent(pP);
                    // 因为子节点找到了新的父节点并更新了父节点，那么该子节点升级，作为其它子节点的备选父节点
                    sParentCandidates.insert(pC);
                    // 该子节点处理完毕，删掉
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            // Step 4.5 如果还有子节点没有找到新的父节点
            if (!mspChildrens.empty())
            {
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++)
                {
                    // 直接把父节点的父节点作为自己的父节点 即对于这些子节点来说,他们的新的父节点其实就是自己的爷爷节点
                    (*sit)->ChangeParent(mpParent);
                }
            }

            if (mpParent)
            {
                mpParent->EraseChild(this);
                // 如果当前的关键帧要被删除的话就要计算这个,表示原父关键帧到当前关键帧的位姿变换
                // 注意在这个删除的过程中,其实并没有将当前关键帧中存储的父关键帧的指针删除掉
                mTcp = mTcw * mpParent->GetPoseInverse();
            }
            // 标记当前关键帧已经死了
            mbBad = true;
        }

        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    // 返回当前关键帧是否已经完蛋了
    bool KeyFrame::isBad()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    // 删除当前关键帧和指定关键帧之间的共视关系
    void KeyFrame::EraseConnection(KeyFrame *pKF)
    {
        // 其实这个应该表示是否真的是有共视关系
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        // 如果是真的有共视关系,那么删除之后就要更新共视关系
        if (bUpdate)
            UpdateBestCovisibles();
    }

    // 获取某个特征点的邻域中的特征点id,其实这个和 Frame.cc 中的那个函数基本上都是一致的; r为边长（半径）
    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        // 计算要搜索的cell的范围
        float factorX = r;
        float factorY = r;

        // floor向下取整，mfGridElementWidthInv 为每个像素占多少个格子
        const int nMinCellX = max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        // ceil向上取整
        const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        // 遍历每个cell,取出其中每个cell中的点,并且每个点都要计算是否在邻域内
        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                               : (!bRight)   ? mvKeys[vCell[j]]
                                                             : mvKeysRight[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    // 判断某个点是否在当前关键帧的图像中
    bool KeyFrame::IsInImage(const float &x, const float &y) const
    {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    /**
     * @brief 在双目和RGBD情况下将特征点反投影到空间中得到世界坐标系下三维点
     *
     * @param[in] i                         第i个特征点
     * @return Eigen::Vector3f              返回世界坐标系下三维点
     */
    bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            // 由2维图像反投影到相机坐标系
            // 双目中mvDepth是在ComputeStereoMatches函数中求取的，rgbd中是直接测量的
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Eigen::Vector3f x3Dc(x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            x3D = mRwc * x3Dc + mTwc.translation();
            return true;
        }
        else
            return false;
    }

    // Compute Scene Depth (q=2 median). Used in monocular. 评估当前关键帧场景深度，q=2表示中值. 只是在单目情况下才会使用
    // 其实过程就是对当前关键帧下所有地图点的深度进行从小到大排序,返回距离头部其中1/q处的深度值作为当前场景的平均深度
    float KeyFrame::ComputeSceneMedianDepth(const int q)
    {
        if (N == 0)
            return -1.0;

        vector<MapPoint *> vpMapPoints;
        Eigen::Matrix3f Rcw;
        Eigen::Vector3f tcw;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            tcw = mTcw.translation();
            Rcw = mRcw;
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
        float zcw = tcw(2);
        // 遍历每一个地图点,计算并保存其在当前关键帧下的深度
        for (int i = 0; i < N; i++)
        {
            if (mvpMapPoints[i])
            {
                MapPoint *pMP = mvpMapPoints[i];
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw; // (R*x3Dw+t)的第三行，即z
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }

    void KeyFrame::SetNewBias(const IMU::Bias &b)
    {
        unique_lock<mutex> lock(mMutexPose);
        mImuBias = b;
        if (mpImuPreintegrated)
            mpImuPreintegrated->SetNewBias(b);
    }

    Eigen::Vector3f KeyFrame::GetGyroBias()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
    }

    Eigen::Vector3f KeyFrame::GetAccBias()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
    }

    IMU::Bias KeyFrame::GetImuBias()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mImuBias;
    }

    Map *KeyFrame::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void KeyFrame::UpdateMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void KeyFrame::PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP, set<GeometricCamera *> &spCam)
    {
        // Save the id of each MapPoint in this KF, there can be null pointer in the vector
        mvBackupMapPointsId.clear();
        mvBackupMapPointsId.reserve(N);
        for (int i = 0; i < N; ++i)
        {

            if (mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
                mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
            else // If the element is null his value is -1 because all the id are positives
                mvBackupMapPointsId.push_back(-1);
        }
        // Save the id of each connected KF with it weight
        mBackupConnectedKeyFrameIdWeights.clear();
        for (std::map<KeyFrame *, int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
        {
            if (spKF.find(it->first) != spKF.end())
                mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
        }

        // Save the parent id
        mBackupParentId = -1;
        if (mpParent && spKF.find(mpParent) != spKF.end())
            mBackupParentId = mpParent->mnId;

        // Save the id of the childrens KF
        mvBackupChildrensId.clear();
        mvBackupChildrensId.reserve(mspChildrens.size());
        for (KeyFrame *pKFi : mspChildrens)
        {
            if (spKF.find(pKFi) != spKF.end())
                mvBackupChildrensId.push_back(pKFi->mnId);
        }

        // Save the id of the loop edge KF
        mvBackupLoopEdgesId.clear();
        mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
        for (KeyFrame *pKFi : mspLoopEdges)
        {
            if (spKF.find(pKFi) != spKF.end())
                mvBackupLoopEdgesId.push_back(pKFi->mnId);
        }

        // Save the id of the merge edge KF
        mvBackupMergeEdgesId.clear();
        mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
        for (KeyFrame *pKFi : mspMergeEdges)
        {
            if (spKF.find(pKFi) != spKF.end())
                mvBackupMergeEdgesId.push_back(pKFi->mnId);
        }

        // Camera data
        mnBackupIdCamera = -1;
        if (mpCamera && spCam.find(mpCamera) != spCam.end())
            mnBackupIdCamera = mpCamera->GetId();

        mnBackupIdCamera2 = -1;
        if (mpCamera2 && spCam.find(mpCamera2) != spCam.end())
            mnBackupIdCamera2 = mpCamera2->GetId();

        // Inertial data
        mBackupPrevKFId = -1;
        if (mPrevKF && spKF.find(mPrevKF) != spKF.end())
            mBackupPrevKFId = mPrevKF->mnId;

        mBackupNextKFId = -1;
        if (mNextKF && spKF.find(mNextKF) != spKF.end())
            mBackupNextKFId = mNextKF->mnId;

        if (mpImuPreintegrated)
            mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
    }

    void KeyFrame::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid, map<unsigned int, GeometricCamera *> &mpCamId)
    {
        // Rebuild the empty variables

        // Pose
        SetPose(mTcw);

        mTrl = mTlr.inverse();

        // Reference reconstruction
        // Each MapPoint sight from this KeyFrame
        mvpMapPoints.clear();
        mvpMapPoints.resize(N);
        for (int i = 0; i < N; ++i)
        {
            if (mvBackupMapPointsId[i] != -1)
                mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
            else
                mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }

        // Conected KeyFrames with him weight
        mConnectedKeyFrameWeights.clear();
        for (map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
             it != end; ++it)
        {
            KeyFrame *pKFi = mpKFid[it->first];
            mConnectedKeyFrameWeights[pKFi] = it->second;
        }

        // Restore parent KeyFrame
        if (mBackupParentId >= 0)
            mpParent = mpKFid[mBackupParentId];

        // KeyFrame childrens
        mspChildrens.clear();
        for (vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it != end; ++it)
        {
            mspChildrens.insert(mpKFid[*it]);
        }

        // Loop edge KeyFrame
        mspLoopEdges.clear();
        for (vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
        {
            mspLoopEdges.insert(mpKFid[*it]);
        }

        // Merge edge KeyFrame
        mspMergeEdges.clear();
        for (vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
        {
            mspMergeEdges.insert(mpKFid[*it]);
        }

        // Camera data
        if (mnBackupIdCamera >= 0)
        {
            mpCamera = mpCamId[mnBackupIdCamera];
        }
        else
        {
            cout << "ERROR: There is not a main camera in KF " << mnId << endl;
        }
        if (mnBackupIdCamera2 >= 0)
        {
            mpCamera2 = mpCamId[mnBackupIdCamera2];
        }

        // Inertial data
        if (mBackupPrevKFId != -1)
        {
            mPrevKF = mpKFid[mBackupPrevKFId];
        }
        if (mBackupNextKFId != -1)
        {
            mNextKF = mpKFid[mBackupNextKFId];
        }
        mpImuPreintegrated = &mBackupImuPreintegrated;

        // Remove all backup container
        mvBackupMapPointsId.clear();
        mBackupConnectedKeyFrameIdWeights.clear();
        mvBackupChildrensId.clear();
        mvBackupLoopEdgesId.clear();

        UpdateBestCovisibles();
    }

    bool KeyFrame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v)
    {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
        float &PcX = Pc(0);
        float &PcY = Pc(1);
        float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
        {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        // cout << "c";

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        float x = (u - cx) * invfx;
        float y = (v - cy) * invfy;
        float r2 = x * x + y * y;
        float k1 = mDistCoef.at<float>(0);
        float k2 = mDistCoef.at<float>(1);
        float p1 = mDistCoef.at<float>(2);
        float p2 = mDistCoef.at<float>(3);
        float k3 = 0;
        if (mDistCoef.total() == 5)
        {
            k3 = mDistCoef.at<float>(4);
        }

        // Radial distorsion
        float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        float u_distort = x_distort * fx + cx;
        float v_distort = y_distort * fy + cy;

        u = u_distort;
        v = v_distort;

        kp = cv::Point2f(u, v);

        return true;
    }

    bool KeyFrame::ProjectPointUnDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v)
    {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
        float &PcX = Pc(0);
        float &PcY = Pc(1);
        float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
        {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        kp = cv::Point2f(u, v);

        return true;
    }

    Sophus::SE3f KeyFrame::GetRelativePoseTrl()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTrl;
    }

    Sophus::SE3f KeyFrame::GetRelativePoseTlr()
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTlr;
    }

    Sophus::SE3<float> KeyFrame::GetRightPose()
    {
        unique_lock<mutex> lock(mMutexPose);

        return mTrl * mTcw;
    }

    Sophus::SE3<float> KeyFrame::GetRightPoseInverse()
    {
        unique_lock<mutex> lock(mMutexPose);

        return mTwc * mTlr;
    }

    Eigen::Vector3f KeyFrame::GetRightCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);

        return (mTwc * mTlr).translation();
    }

    Eigen::Matrix<float, 3, 3> KeyFrame::GetRightRotation()
    {
        unique_lock<mutex> lock(mMutexPose);

        return (mTrl.so3() * mTcw.so3()).matrix();
    }

    Eigen::Vector3f KeyFrame::GetRightTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return (mTrl * mTcw).translation();
    }

    void KeyFrame::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        mpORBvocabulary = pORBVoc;
    }

    void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
    {
        mpKeyFrameDB = pKFDB;
    }

} // namespace ORB_SLAM
