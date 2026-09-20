#ifndef REPROJECTION_H
#define REPROJECTION_H

#include <Eigen/Dense>

// 相机内参结构体
struct CameraIntrinsics {
    double fx, fy, cx, cy;
};

// 三维世界坐标点
struct Point3D {
    double x, y, z;
};

// 二维像素坐标点
struct Point2D {
    double u, v;
};

/**
 * @brief 重投影计算：世界点 -> 像素坐标
 * @param pw 世界坐标系下的3D点
 * @param K 相机内参
 * @param R 旋转矩阵 (3x3)
 * @param t 平移向量 (3x1)
 * @return 像素坐标 (u, v)
 * @throws std::runtime_error 如果 Zc <= 0 (非正深度)
 */
Point2D projectPoint(const Point3D& pw, const CameraIntrinsics& K,
                     const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/**
 * @brief 计算两个像素点之间的欧氏距离
 */
double computePixelDistance(const Point2D& p1, const Point2D& p2);

#endif // REPROJECTION_H