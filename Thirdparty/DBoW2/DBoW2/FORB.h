/**
 * File: FORB.h
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_F_ORB__
#define __D_T_F_ORB__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2
{

  /// Functions to manipulate ORB descriptors
  class FORB : protected FClass
  {
  public:
    /// Descriptor type
    typedef cv::Mat TDescriptor; // CV_8U

    /// Pointer to a single descriptor
    typedef const TDescriptor *pDescriptor;

    /// Descriptor length (in bytes)
    static const int L;

    /**
     * Calculates the mean value of a set of descriptors
     * @param descriptors
     * @param mean mean descriptor
     */
    static void meanValue(const std::vector<pDescriptor> &descriptors,
                          TDescriptor &mean);

    // todo 计算两个描述子之间的距离
    static int distance(const TDescriptor &a, const TDescriptor &b);

    /**
     * Returns a string version of the descriptor
     * @param a descriptor
     * @return string version
     */
    static std::string toString(const TDescriptor &a);

    /**
     * Returns a descriptor from a string
     * @param a descriptor
     * @param s string version
     */
    static void fromString(TDescriptor &a, const std::string &s);

    /**
     * Returns a mat with the descriptors in float format
     * @param descriptors
     * @param mat (out) NxL 32F matrix
     */
    static void toMat32F(const std::vector<TDescriptor> &descriptors,
                         cv::Mat &mat);

    static void toMat8U(const std::vector<TDescriptor> &descriptors,
                        cv::Mat &mat);

    // note 从二进制文件中加载描述符
    static void loadORBDescriptors(const std::string &filename, std::vector<TDescriptor> &descriptors);

    // note 将描述符写入二进制文件
    static void saveORBDescriptors(const std::string &filename, const std::vector<TDescriptor> &descriptors);
  };

} // namespace DBoW2

#endif