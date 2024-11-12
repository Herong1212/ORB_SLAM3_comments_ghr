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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace ORB_SLAM3
{
    class GeometricCamera
    {

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnType;
            ar & mvParameters;
        }

    public:
        GeometricCamera() {}
        GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}
        ~GeometricCamera() {}

        // 投影函数
        virtual cv::Point2f project(const cv::Point3f &p3D) = 0; // 纯虚函数
        virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) = 0;
        virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) = 0;
        virtual Eigen::Vector2f projectMat(const cv::Point3f &p3D) = 0;

        // 返回像素点的不确定性，返回的都是 1.0f
        virtual float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) = 0; // 优化时调用

        // 反投影
        virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) = 0;
        virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;

        // 投影雅可比
        virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D) = 0;

        // 反投影雅可比
        // ? 这里并没有使用

        // 三角化恢复三维点，主要在单目初始化中使用（SLAM2中是在单目初始化那个函数中实现的）
        virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const std::vector<int> &vMatches12,
                                             Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) = 0;

        // 返回 K 矩阵
        virtual cv::Mat toK() = 0;
        virtual Eigen::Matrix3f toK_() = 0;

        // 极限约束，三角化 MP 时使用，局部建图线程中调用
        virtual bool epipolarConstrain(GeometricCamera *otherCamera, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel, const float unc) = 0;

        // 获取与设置相机内参数
        float getParameter(const int i) { return mvParameters[i]; }
        void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

        // 参数的 size
        size_t size() { return mvParameters.size(); }

        // 此处声明了但后面没有实际使用
        virtual bool matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, GeometricCamera *pOther,
                                         Sophus::SE3f &Tcw1, Sophus::SE3f &Tcw2,
                                         const float sigmaLevel1, const float sigmaLevel2,
                                         Eigen::Vector3f &x3Dtriangulated) = 0;

        // 关于 ID 与 type 的获取函数，但是代码中没有使用
        unsigned int GetId() { return mnId; }
        unsigned int GetType() { return mnType; }

        const static unsigned int CAM_PINHOLE = 0;
        const static unsigned int CAM_FISHEYE = 1; // 区分模式用的

        static long unsigned int nNextId;

    protected:
        std::vector<float> mvParameters; // 存放参数，包括 K 的参数与畸变

        unsigned int mnId;

        unsigned int mnType;
    };
}

#endif // CAMERAMODELS_GEOMETRICCAMERA_H
