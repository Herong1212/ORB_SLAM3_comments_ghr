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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace ORB_SLAM3
{

    // 构造函数
    KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) : mpVoc(&voc)
    {
        mvInvertedFile.resize(voc.size());
    }

    /**
     * @brief 数据库中有新的关键帧，根据关键帧的词袋向量更新数据库的倒排索引
     *
     * @param[in] pKF   新添加到数据库中的关键帧
     */
    void KeyFrameDatabase::add(KeyFrame *pKF)
    {
        // 线程锁
        unique_lock<mutex> lock(mMutex);

        // 为每一个 word 添加该 KeyFrame【对该关键帧词袋向量中的每一个单词更新倒排索引】
        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            mvInvertedFile[vit->first].push_back(pKF);
    }

    /**
     * @brief 关键帧被删除后，更新数据库的倒排索引
     *
     * @param[in] pKF   删除的关键帧
     */
    void KeyFrameDatabase::erase(KeyFrame *pKF)
    {
        // 线程锁，保护共享数据
        unique_lock<mutex> lock(mMutex);

        // 每一个 KeyFrame 包含多个 words，遍历倒排索引中的这些单词，然后在单词对应的关键帧列表中删除该 KeyFrame
        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            // 取出包含该单词的所有关键帧列表
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            // 如果包含待删除的关键帧，则把该关键帧从列表里删除
            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                if (pKF == *lit)
                {
                    lKFs.erase(lit);
                    break;
                }
            }
        }
    }

    // 清空关键帧数据库
    void KeyFrameDatabase::clear()
    {
        mvInvertedFile.clear();
        mvInvertedFile.resize(mpVoc->size());
    }

    void KeyFrameDatabase::clearMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutex);

        // Erase elements in the Inverse File for the entry
        for (std::vector<list<KeyFrame *>>::iterator vit = mvInvertedFile.begin(), vend = mvInvertedFile.end(); vit != vend; vit++)
        {
            // List of keyframes that share the word
            list<KeyFrame *> &lKFs = *vit;

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend;)
            {
                KeyFrame *pKFi = *lit;
                if (pMap == pKFi->GetMap())
                {
                    lit = lKFs.erase(lit);
                    // Dont delete the KF because the class Map clean all the KF when it is destroyed
                }
                else
                {
                    ++lit;
                }
            }
        }
    }

    // note1：闭环检测的第一阶段 -- 寻找闭环候选关键帧
    /**
     * @brief 在闭环检测中找到与该关键帧可能闭环的关键帧（注意不和当前帧连接）
     * Step 1：找出和当前帧具有公共单词的所有关键帧，不包括与当前帧连接（也就是共视）的关键帧
     * Step 2：只和具有共同单词较多的（最大数目的80%以上）关键帧进行相似度计算
     * Step 3：计算上述候选帧对应的共视关键帧组的总得分，只取最高组得分75%以上的组
     * Step 4：得到上述组中分数最高的关键帧作为闭环候选关键帧
     * @param[in] pKF               需要闭环检测的关键帧
     * @param[in] minScore          候选闭环关键帧帧和当前关键帧的 BoW 相似度至少要大于 minScore
     * @return vector<KeyFrame*>    闭环候选关键帧
     */
    vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, float minScore)
    {
        // 取出与当前关键帧相连（>15个共视地图点）的所有关键帧，这些相连关键帧都是局部相连，在闭环检测的时候将被剔除
        // 相连关键帧定义见 KeyFrame::UpdateConnections()
        set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();

        // 用于保存可能与当前关键帧形成闭环的候选帧（只要有相同的 word，且不属于局部相连（共视）帧）
        list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        // Step 1：找出和当前帧具有公共单词的所有关键帧，不包括与当前帧连接（也就是共视）的关键帧
        {
            unique_lock<mutex> lock(mMutex);

            // words是检测图像是否匹配的枢纽，遍历该 pKF 的每一个word
            // mBowVec 内部实际存储的是std::map<WordId, WordValue>
            // WordId 和 WordValue 表示Word在叶子中的id 和权重
            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                // 提取所有包含该 word 的 KeyFrame，即找到和当前关键帧有共同视觉词汇的关键帧
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                // 然后对这些关键帧展开遍历
                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    // 遍历这些和当前关键帧有共同视觉词汇的关键帧，如果不属于与当前关键帧相连的关键帧，就把这些关键帧加入到一个vector中，叫做lKFsSharingWords
                    KeyFrame *pKFi = *lit;
                    if (pKFi->GetMap() == pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
                    {
                        if (pKFi->mnLoopQuery != pKF->mnId)
                        {
                            // 还没有标记为pKF的闭环候选帧
                            pKFi->mnLoopWords = 0;
                            // 和当前关键帧共视的话不作为闭环候选帧
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                // 没有共视就标记作为闭环候选关键帧，放到lKFsSharingWords里
                                pKFi->mnLoopQuery = pKF->mnId;
                                lKFsSharingWords.push_back(pKFi);
                            }
                        }
                        pKFi->mnLoopWords++; // 记录pKFi与pKF具有相同word的个数
                    }
                }
            }
        }
        // 如果没有关键帧和这个关键帧具有相同的单词,那么就返回空
        if (lKFsSharingWords.empty())
            return vector<KeyFrame *>();

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        // Step 2：统计上述所有闭环候选帧中与当前帧具有共同单词最多的单词数，用来决定相对阈值
        // 在lKFsSharingWords中有一些关键帧，与当前关键帧有共同的视觉词汇，最多的数目为maxCommonWords，再乘0.8为minCommonWords
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnLoopWords > maxCommonWords)
                maxCommonWords = (*lit)->mnLoopWords;
        }

        // 确定最小公共单词数为最大公共单词数目的0.8倍
        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        // 在有共同视觉词汇的关键帧中，找到词袋向量与关键帧的词袋向量比较的score大于minScore的关键帧，保留其score，共同放入list<pair<float,KeyFrame*> >lScoreAndMatch
        // Step 3：遍历上述所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            // pKF只和具有共同单词较多（大于minCommonWords）的关键帧进行比较
            if (pKFi->mnLoopWords > minCommonWords)
            {
                nscores++;

                // 用mBowVec来计算两者的相似度得分
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                if (si >= minScore) // minScore是寻找闭环的参数，是词袋向量之间比较得到的分值。
                    lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        // 如果没有超过指定相似度阈值的，那么也就直接跳过去
        if (lScoreAndMatch.empty())
            return vector<KeyFrame *>();

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = minScore;

        // Lets now accumulate score by covisibility
        // 在与当前关键帧有共同的视觉词汇，且词袋之比值评分大于minScore的关键帧中，继续挑选，通过共视关系，选出10个共视的最好的关键帧，然后继续积累score值，找到一个比较好的，用作阈值的标杆
        // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
        // Step 4：计算上述候选帧对应的共视关键帧组的总得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first; // 该组最高分数
            float accScore = it->first;  // 该组累计得分
            KeyFrame *pBestKF = pKFi;    // 该组最高分数对应的关键帧
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;

                // 只有pKF2也在闭环候选帧中，且公共单词数超过最小要求，才能贡献分数
                if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
                {
                    accScore += pKF2->mLoopScore;
                    // 统计得到组里分数最高的关键帧
                    if (pKF2->mLoopScore > bestScore)
                    {
                        pBestKF = pKF2;
                        bestScore = pKF2->mLoopScore;
                    }
                }
            }

            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            // 记录所有组中组得分最高的组，用于确定相对阈值
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        // 所有组中最高得分的0.75倍，作为最低阈值
        float minScoreToRetain = 0.75f * bestAccScore;

        set<KeyFrame *> spAlreadyAddedKF;
        vector<KeyFrame *> vpLoopCandidates;
        vpLoopCandidates.reserve(lAccScoreAndMatch.size());

        // Step 5：只取组得分大于阈值的组，得到组中分数最高的关键帧作为闭环候选关键帧
        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            if (it->first > minScoreToRetain) // 卡一个阈值
            {
                KeyFrame *pKFi = it->second;

                // spAlreadyAddedKF 是为了防止重复添加
                if (!spAlreadyAddedKF.count(pKFi)) // 没有重复出现过的
                {
                    vpLoopCandidates.push_back(pKFi); // 得到最后的闭环候选帧的集合
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpLoopCandidates;
    }

    void KeyFrameDatabase::DetectCandidates(KeyFrame *pKF, float minScore, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand)
    {
        set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        list<KeyFrame *> lKFsSharingWordsLoop, lKFsSharingWordsMerge;

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (pKFi->GetMap() == pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
                    {
                        if (pKFi->mnLoopQuery != pKF->mnId)
                        {
                            pKFi->mnLoopWords = 0;
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                pKFi->mnLoopQuery = pKF->mnId;
                                lKFsSharingWordsLoop.push_back(pKFi);
                            }
                        }
                        pKFi->mnLoopWords++;
                    }
                    else if (!pKFi->GetMap()->IsBad())
                    {
                        if (pKFi->mnMergeQuery != pKF->mnId)
                        {
                            pKFi->mnMergeWords = 0;
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                pKFi->mnMergeQuery = pKF->mnId;
                                lKFsSharingWordsMerge.push_back(pKFi);
                            }
                        }
                        pKFi->mnMergeWords++;
                    }
                }
            }
        }

        if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
            return;

        if (!lKFsSharingWordsLoop.empty())
        {
            list<pair<float, KeyFrame *>> lScoreAndMatch;

            // Only compare against those keyframes that share enough words
            int maxCommonWords = 0;
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++)
            {
                if ((*lit)->mnLoopWords > maxCommonWords)
                    maxCommonWords = (*lit)->mnLoopWords;
            }

            int minCommonWords = maxCommonWords * 0.8f;

            int nscores = 0;

            // Compute similarity score. Retain the matches whose score is higher than minScore
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                if (pKFi->mnLoopWords > minCommonWords)
                {
                    nscores++;

                    float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                    pKFi->mLoopScore = si;
                    if (si >= minScore)
                        lScoreAndMatch.push_back(make_pair(si, pKFi));
                }
            }

            if (!lScoreAndMatch.empty())
            {
                list<pair<float, KeyFrame *>> lAccScoreAndMatch;
                float bestAccScore = minScore;

                // Lets now accumulate score by covisibility
                for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
                {
                    KeyFrame *pKFi = it->second;
                    vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                    float bestScore = it->first;
                    float accScore = it->first;
                    KeyFrame *pBestKF = pKFi;
                    for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
                    {
                        KeyFrame *pKF2 = *vit;
                        if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
                        {
                            accScore += pKF2->mLoopScore;
                            if (pKF2->mLoopScore > bestScore)
                            {
                                pBestKF = pKF2;
                                bestScore = pKF2->mLoopScore;
                            }
                        }
                    }

                    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                    if (accScore > bestAccScore)
                        bestAccScore = accScore;
                }

                // Return all those keyframes with a score higher than 0.75*bestScore
                float minScoreToRetain = 0.75f * bestAccScore;

                set<KeyFrame *> spAlreadyAddedKF;
                vpLoopCand.reserve(lAccScoreAndMatch.size());

                for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
                {
                    if (it->first > minScoreToRetain)
                    {
                        KeyFrame *pKFi = it->second;
                        if (!spAlreadyAddedKF.count(pKFi))
                        {
                            vpLoopCand.push_back(pKFi);
                            spAlreadyAddedKF.insert(pKFi);
                        }
                    }
                }
            }
        }

        if (!lKFsSharingWordsMerge.empty())
        {
            // cout << "BoW candidates: " << lKFsSharingWordsMerge.size() << endl;
            list<pair<float, KeyFrame *>> lScoreAndMatch;

            // Only compare against those keyframes that share enough words
            int maxCommonWords = 0;
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++)
            {
                if ((*lit)->mnMergeWords > maxCommonWords)
                    maxCommonWords = (*lit)->mnMergeWords;
            }
            // cout << "Max common words: " << maxCommonWords << endl;

            int minCommonWords = maxCommonWords * 0.8f;

            int nscores = 0;

            // Compute similarity score. Retain the matches whose score is higher than minScore
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                if (pKFi->mnMergeWords > minCommonWords)
                {
                    nscores++;

                    float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
                    // cout << "KF score: " << si << endl;

                    pKFi->mMergeScore = si;
                    if (si >= minScore)
                        lScoreAndMatch.push_back(make_pair(si, pKFi));
                }
            }
            // cout << "BoW candidates2: " << lScoreAndMatch.size() << endl;

            if (!lScoreAndMatch.empty())
            {
                list<pair<float, KeyFrame *>> lAccScoreAndMatch;
                float bestAccScore = minScore;

                // Lets now accumulate score by covisibility
                for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
                {
                    KeyFrame *pKFi = it->second;
                    vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                    float bestScore = it->first;
                    float accScore = it->first;
                    KeyFrame *pBestKF = pKFi;
                    for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
                    {
                        KeyFrame *pKF2 = *vit;
                        if (pKF2->mnMergeQuery == pKF->mnId && pKF2->mnMergeWords > minCommonWords)
                        {
                            accScore += pKF2->mMergeScore;
                            if (pKF2->mMergeScore > bestScore)
                            {
                                pBestKF = pKF2;
                                bestScore = pKF2->mMergeScore;
                            }
                        }
                    }

                    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                    if (accScore > bestAccScore)
                        bestAccScore = accScore;
                }

                // Return all those keyframes with a score higher than 0.75*bestScore
                float minScoreToRetain = 0.75f * bestAccScore;

                // cout << "Min score to retain: " << minScoreToRetain << endl;

                set<KeyFrame *> spAlreadyAddedKF;
                vpMergeCand.reserve(lAccScoreAndMatch.size());

                for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
                {
                    if (it->first > minScoreToRetain)
                    {
                        KeyFrame *pKFi = it->second;
                        if (!spAlreadyAddedKF.count(pKFi))
                        {
                            vpMergeCand.push_back(pKFi);
                            spAlreadyAddedKF.insert(pKFi);
                        }
                    }
                }
                // cout << "Candidates: " << vpMergeCand.size() << endl;
            }
        }

        //----
        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                pKFi->mnLoopQuery = -1;
                pKFi->mnMergeQuery = -1;
            }
        }
    }

    void KeyFrameDatabase::DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nMinWords)
    {
        list<KeyFrame *> lKFsSharingWords;
        set<KeyFrame *> spConnectedKF;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            spConnectedKF = pKF->GetConnectedKeyFrames();

            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (spConnectedKF.find(pKFi) != spConnectedKF.end())
                    {
                        continue;
                    }
                    if (pKFi->mnPlaceRecognitionQuery != pKF->mnId)
                    {
                        pKFi->mnPlaceRecognitionWords = 0;
                        pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnPlaceRecognitionWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
                maxCommonWords = (*lit)->mnPlaceRecognitionWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        if (minCommonWords < nMinWords)
        {
            minCommonWords = nMinWords;
        }

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mnPlaceRecognitionWords > minCommonWords)
            {
                nscores++;
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
                pKFi->mPlaceRecognitionScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (lScoreAndMatch.empty())
            return;

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                if (pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                    continue;

                accScore += pKF2->mPlaceRecognitionScore;
                if (pKF2->mPlaceRecognitionScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mPlaceRecognitionScore;
                }
            }
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.75f * bestAccScore;
        set<KeyFrame *> spAlreadyAddedKF;
        vpLoopCand.reserve(lAccScoreAndMatch.size());
        vpMergeCand.reserve(lAccScoreAndMatch.size());
        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    if (pKF->GetMap() == pKFi->GetMap())
                    {
                        vpLoopCand.push_back(pKFi);
                    }
                    else
                    {
                        vpMergeCand.push_back(pKFi);
                    }
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }
    }

    bool compFirst(const pair<float, KeyFrame *> &a, const pair<float, KeyFrame *> &b)
    {
        return a.first > b.first;
    }

    // todo 作用：根据公共单词，使用【1】个相对阈值，来寻找【闭环候选关键帧】（寻找到的候选关键帧限制数量 为 3 个，统一称为 Km）
    /**
     * @brief 找到 N （ = 3 ) 个融合候选帧，N （ = 3 ) 个回环候选帧
     *
     * @param[in] pKF 当前关键帧(我们要寻找这个关键帧的【回环候选帧】和【融合候选帧】)
     * @param[out] vpLoopCand 记录找到的回环候选帧，对应闭环操作
     * @param[out] vpMergeCand 记录找到的融合候选帧，对应地图融合操作
     * @param[in] nNumCandidates 期望的候选数目, 即回环和候选分别应该有多少个（论文中好像是 3 个吧）
     */
    void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nNumCandidates)
    {

        // 用于保存可能与当前关键帧形成闭环的候选帧（只要有相同的 word，且不属于局部相连（共视）帧）
        list<KeyFrame *> lKFsSharingWords;

        // set<KeyFrame *> spInsertedKFsSharing;
        // 当前关键帧的共视关键帧(避免将当前关键帧的共视关键帧加入回环检测)
        set<KeyFrame *> spConnectedKF;

        // Step 1：统计与当前关键帧有相同单词的所有关键帧，不包括与当前帧连接的关键帧
        {
            unique_lock<mutex> lock(mMutex);

            // 拿到当前关键帧的共视关键帧（取出与当前关键帧相连（ >15个共视地图点）的所有关键帧，这些相连关键帧都是局部相连，在闭环检测的时候将被剔除）
            spConnectedKF = pKF->GetConnectedKeyFrames();

            // words 是检测图像是否匹配的枢纽，遍历该 pKF 的词袋向量的每一个 word
            // mBowVec 内部实际存储的是 std::map<WordId, WordValue>
            // WordId 和 WordValue 表示 Word 在叶子中的 id 和权重
            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                // 拿到当前单词的逆向索引(提取所有有当前单词的关键帧)
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                // 遍历每个有该单词的关键帧
                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    /*if(spConnectedKF.find(pKFi) != spConnectedKF.end())
                    {
                        continue;
                    }*/

                    // 如果此关键帧没有被当前关键帧访问过(防止重复添加)
                    if (pKFi->mnPlaceRecognitionQuery != pKF->mnId)
                    {
                        // 初始化公共单词数为 0
                        pKFi->mnPlaceRecognitionWords = 0;

                        // 如果该关键帧不是当前关键帧的共视关键帧
                        // * 这一步很重要，就是判断那些绿色的点不是那些蓝色的点
                        if (!spConnectedKF.count(pKFi))
                        {
                            // 标记该关键帧被当前关键帧访问到（也就是有公共单词）
                            pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                            // 把当前关键帧添加到有公共单词的关键帧列表中
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    // 递增该关键帧与当前关键帧的公共单词数
                    pKFi->mnPlaceRecognitionWords++;

                    /*if(spInsertedKFsSharing.find(pKFi) == spInsertedKFsSharing.end())
                    {
                        lKFsSharingWords.push_back(pKFi);
                        spInsertedKFsSharing.insert(pKFi);
                    }*/
                }
            }
        }
        // 如果没有关键帧和这个关键帧具有相同的单词, 那么就返回空
        if (lKFsSharingWords.empty())
            return;

        // * ---------------------------------------- lKFsSharingWords 里都是存放的第一步筛选出来的【与当前关键帧有相同单词的所有关键帧】了！

        // Step 2 统计上述所有候选帧中与当前关键帧的【公共单词数最多】的单词数 maxCommonWords, 用来决定相对阈值，并筛选
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
                maxCommonWords = (*lit)->mnPlaceRecognitionWords;
        }

        // * 阈值：取 0.8 倍为阀值（确定最小公共单词数为最大公共单词数目的 0.8 倍）
        int minCommonWords = maxCommonWords * 0.8f;

        // 这里的 pair 是 <相似度, 候选关键帧的指针> : 记录所有大于 minCommonWords 的候选帧与当前关键帧的相似度
        list<pair<float, KeyFrame *>> lScoreAndMatch;

        // ? 只是个统计变量, 貌似没有用到
        int nscores = 0;

        // 对所有大于 minCommonWords 的候选帧计算相似度

        // 遍历所有有公共单词的候选帧
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            // 如果当前帧的公共单词数大于 minCommonWords
            if (pKFi->mnPlaceRecognitionWords > minCommonWords)
            {
                nscores++; // 未使用
                // 计算相似度
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
                // 记录该候选帧与当前帧的相似度
                pKFi->mPlaceRecognitionScore = si;
                // 把符合要求的关键帧放到容器里, 每个元素是 <相似度, 候选帧的指针>
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        // 如果没有超过指定相似度阈值的，那么也就直接跳过去，表示没有符合上述条件的关键帧
        if (lScoreAndMatch.empty())
            return;

        // * ---------------------------------------- 至此，lScoreAndMatch 里都是存放的第二步筛选出来的【与当前关键帧的【公共单词数最多】的关键帧们】了！

        // Step 3 : 用小组得分排序得到 top3 总分里最高分的关键帧, 作为候选帧
        // 统计以【组】为单位的累计相似度和组内相似度最高的关键帧, 每个 pair 为 <小组总相似度, 组内相似度最高的关键帧指针>
        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        // 变量所有被 lScoreAndMatch 记录的 pair <相似度,候选关键帧>
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            // 候选关键帧
            KeyFrame *pKFi = it->second;
            // 与候选关键帧共视关系最好的【10个】关键帧
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            // 初始化最大相似度为该候选关键帧自己的相似度
            float bestScore = it->first;

            // 初始化小组累计得分为改候选关键帧自己的相似度
            float accScore = bestScore;

            // 初始化组内相似度最高的帧为该候选关键帧本身
            KeyFrame *pBestKF = pKFi;

            // 遍历与当前关键帧共视关系最好的【10帧】
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                // 如果该关键帧没有被当前关键帧访问过(也就是没有公共单词)则跳过
                if (pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                    continue;

                // 累加小组总分
                accScore += pKF2->mPlaceRecognitionScore;

                // 如果大于组内最高分,则更新当前最高分记录
                if (pKF2->mPlaceRecognitionScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mPlaceRecognitionScore;
                }
            }

            // 统计以组为单位的累计相似度和组内相似度最高的关键帧, 每个 pair 为<小组总相似度, 组内相似度最高的关键帧指针>
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));

            // 统计最高得分, 这个 bestAccSocre 没有用到
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // cout << "Amount of candidates: " << lAccScoreAndMatch.size() << endl;

        //  按相似度从大到小排序
        lAccScoreAndMatch.sort(compFirst);

        // 最后返回的变量, 记录回环的候选帧
        vpLoopCand.reserve(nNumCandidates);
        // 最后返回的变量, 记录融合候选帧
        vpMergeCand.reserve(nNumCandidates);

        // 避免重复添加
        set<KeyFrame *> spAlreadyAddedKF;
        // cout << "Candidates in score order " << endl;

        // for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)

        int i = 0;
        list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin();

        // 遍历 lAccScoreAndMatch 中所有的 pair, 每个 pair 为<小组总相似度, 组内相似度最高的关键帧指针>，nNumCandidates 默认为【3】
        while (i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates))
        {
            // cout << "Accum score: " << it->first << endl;

            // 拿到候选关键帧的指针
            KeyFrame *pKFi = it->second;
            if (pKFi->isBad())
                continue;

            // 如果没有被重复添加
            if (!spAlreadyAddedKF.count(pKFi))
            {
                // NOTE 闭环和地图融合的区别👇 --- 非常重要！

                // case1：如果候选帧 pKFi 与当前关键帧 pKF 【在同一个地图】里, 且候选者数量还不足够
                if (pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates)
                {
                    // 添加到【回环候选帧】里
                    vpLoopCand.push_back(pKFi);
                }
                // case2：如果候选帧 pKFi 与当前关键帧 pKF 【不在同一个地图】里, 且候选者数量还不足够, 且候选者所在地图不是 bad （因为此时用的是非活跃地图，可能会被删掉或者其它操作，所以需要看下是不是有效的）
                else if (pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
                {
                    // 添加到【融合候选帧】里
                    vpMergeCand.push_back(pKFi);
                }

                // 防止重复添加
                spAlreadyAddedKF.insert(pKFi);
            }
            i++;
            it++;
        }

        // -----------------------

        // Return all those keyframes with a score higher than 0.75*bestScore
        /* float minScoreToRetain = 0.75f * bestAccScore;
        set<KeyFrame> spAlreadyAddedKF;
        vpLoopCand.reserve(lAccScoreAndMatch.size());
        vpMergeCand.reserve(lAccScoreAndMatch.size());

        for (list<pair<float, KeyFrame *>> : iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    if (pKF->GetMap() == pKFi->GetMap())
                    {
                        vpLoopCand.push_back(pKFi);
                    }
                    else
                    {
                        vpMergeCand.push_back(pKFi);
                    }
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        } */
    }

    // note：搜索重定位候选关键帧
    /**
     * @brief 在重定位中找到与该关键帧相似的候选关键帧
     *
     * @param F 需要重定位的帧
     * @param pMap
     * @return vector<KeyFrame *> 相似的候选关键帧组
     */
    vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map *pMap)
    {
        list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (pKFi->mnRelocQuery != F->mnId)
                    {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return vector<KeyFrame *>();

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnRelocWords > maxCommonWords)
                maxCommonWords = (*lit)->mnRelocWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mnRelocWords > minCommonWords)
            {
                nscores++;
                float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
                pKFi->mRelocScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (lScoreAndMatch.empty())
            return vector<KeyFrame *>();

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                if (pKF2->mnRelocQuery != F->mnId)
                    continue;

                accScore += pKF2->mRelocScore;
                if (pKF2->mRelocScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mRelocScore;
                }
            }
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.75f * bestAccScore;
        set<KeyFrame *> spAlreadyAddedKF;
        vector<KeyFrame *> vpRelocCandidates;
        vpRelocCandidates.reserve(lAccScoreAndMatch.size());
        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;
                if (pKFi->GetMap() != pMap)
                    continue;
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    vpRelocCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpRelocCandidates;
    }

    void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        ORBVocabulary **ptr;
        ptr = (ORBVocabulary **)(&mpVoc);
        *ptr = pORBVoc;

        mvInvertedFile.clear();
        mvInvertedFile.resize(mpVoc->size());
    }

} // namespace ORB_SLAM
