#include <iostream>
#include <iomanip>
#include "reprojection.h"

int main() {
    // ================= 1. 构造测试数据 =================
    
    // 假设相机内参 (以像素为单位)
    CameraIntrinsics K = {800.0, 800.0, 640.0, 360.0};

    // 假设外参：世界到相机的变换
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // 单位旋转矩阵（简化测试）
    Eigen::Vector3d t(0.0, 0.0, 1.0);                // 相机在原点前方1米处

    // 构造一个世界坐标点 Pw (位于相机正前方)
    Point3D Pw = {0.1, 0.1, 1.0};

    // 自行构造一个对应的观测点 (u_obs, v_obs)，用于计算误差
    // 理论计算：Pc = (0.1, 0.1, 2.0)，u = 800*(0.1/2) + 640 = 680, v = 800*(0.1/2) + 360 = 400
    // 故意稍微偏移一点来模拟误差：设为 (682, 398)
    Point2D observedPoint = {682.0, 398.0}; 

    // ================= 2. 正常计算与误差验证 =================
    try {
        std::cout << "--- 正常投影测试 ---" << std::endl;
        Point2D projected = projectPoint(Pw, K, R, t);
        
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "计算得到的像素坐标: (" << projected.u << ", " << projected.v << ")" << std::endl;
        std::cout << "观测到的像素坐标:   (" << observedPoint.u << ", " << observedPoint.v << ")" << std::endl;

        double error = computePixelDistance(projected, observedPoint);
        std::cout << "像素欧氏距离误差: " << error << " pixels" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "发生异常: " << e.what() << std::endl;
    }

    // ================= 3. 异常情况测试 (必做) =================
    std::cout << "\n--- 非正深度异常测试 ---" << std::endl;
    // 构造一个位于相机后方的点 (Zw = -2.0，Zc 将小于等于 0)
    Point3D Pw_behind = {0.1, 0.1, -2.0}; 
    
    try {
        Point2D invalidPoint = projectPoint(Pw_behind, K, R, t);
        // 如果没抛出异常，输出错误提示
        std::cout << "警告：Zc <= 0 但没有捕获到异常！" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "成功捕获异常: " << e.what() << std::endl;
    }

    return 0;
}