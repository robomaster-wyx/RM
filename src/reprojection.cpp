#include "reprojection.h"
#include <cmath>
#include <stdexcept>

Point2D projectPoint(const Point3D& pw, const CameraIntrinsics& K,
                     const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    // 1. 世界坐标转相机坐标: Pc = R * Pw + t
    Eigen::Vector3d Pw(pw.x, pw.y, pw.z);
    Eigen::Vector3d Pc = R * Pw + t;

    // 2. ⚠️ 核心考点：处理非正深度
    if (Pc.z() <= 0.0) {
        throw std::runtime_error("错误：该点位于相机后方或中心，无法投影 (Zc <= 0)");
    }

    // 3. 理想针孔模型投影: u = fx * Xc / Zc + cx, v = fy * Yc / Zc + cy
    double u = K.fx * (Pc.x() / Pc.z()) + K.cx;
    double v = K.fy * (Pc.y() / Pc.z()) + K.cy;

    return {u, v};
}

double computePixelDistance(const Point2D& p1, const Point2D& p2) {
    double du = p1.u - p2.u;
    double dv = p1.v - p2.v;
    return std::sqrt(du * du + dv * dv);
}