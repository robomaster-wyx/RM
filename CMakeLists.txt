cmake_minimum_required(VERSION 3.10)
project(RM_Reprojection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ⚠️ 就是这两行，告诉 CMake 去寻找 Eigen
find_package(Eigen3 REQUIRED)

# 包含头文件目录（这里把 Eigen 的路径也加进来）
include_directories(${EIGEN3_INCLUDE_DIR} include)

add_executable(reprojection_test src/main.cpp src/reprojection.cpp)

# ⚠️ 链接 Eigen 库
target_link_libraries(reprojection_test Eigen3::Eigen)