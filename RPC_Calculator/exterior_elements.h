#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <eigen3/Eigen/Dense>

class ExteriorElements
{
public:
    /**
     * @brief 构造函数
     *
     * @param attDataPath 姿态数据文件路径
     * @param gpsDataPath gps轨道数据文件路径
     * @param imgTimePath 扫描行时间标签文件
     */
    ExteriorElements(const std::string& attDataPath, const std::string& gpsDataPath, const std::string& imgTimePath);

    /**
     * @brief 加载扫描行时间标签文件
     *
     * @param filePath 姿态数据文件路径
     * @param imageTime 存放读入的时间标签文件数组 RelLine Time deltaTime
     */
    void load_imageTime(const std::string& filePath, std::vector<std::tuple<int, double, double>>& imageTime);

    /**
     * @brief 加载gps轨道数据文件
     *
     * @param filePath gps轨道数据文件路径
     * @param gpsData 存放读入的gps数据文件数组 timeCode dateTime PX PY PZ VX VY VZ
     */
    void load_gpsData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData);

    /**
     * @brief 加载姿态数据文件
     *
     * @param filePath 姿态数据文件路径
     * @param gpsData 存放读入的姿态数据文件数组 timeCode dateTime roll pitch yaw roll_velocity pitch_velocity yaw_velocity q1 q2 q3 q4
     */
    void load_attData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData);

    /**
     * @brief 加载姿态数据文件
     *
     * @param filePath 姿态数据文件路径
     * @param gpsData 存放读入的姿态数据文件数组 timeCode dateTime roll pitch yaw roll_velocity pitch_velocity yaw_velocity q1 q2 q3 q4
     */
    void transform_time(const std::vector<std::tuple<int, double, double>>& imageTime);

    /**
     * @brief 内插姿态数据得到成像时刻四元数
     *
     * @param imageTime 成像时刻时间标签
     * @param attData 姿态数据
     */
    void att_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData);

    /**
     * @brief 内插轨道数据得到成像时刻卫星位置
     *
     * @param imageTime 成像时刻时间标签
     * @param gpsData 轨道数据
     */
    void gps_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData);

public:
    // 成像时刻姿态 q1 q2 q3 q4
    std::vector<Eigen::Vector4d> att;
    // 成像时刻位置 PX PY PZ
    std::vector<Eigen::Vector3d> gps;
    // 成像时刻UTC
    std::vector<std::tuple<int, int, int, int, int, double>> transformed_time;
};

