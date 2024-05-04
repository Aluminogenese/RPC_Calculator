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
     * @brief ���캯��
     *
     * @param attDataPath ��̬�����ļ�·��
     * @param gpsDataPath gps��������ļ�·��
     * @param imgTimePath ɨ����ʱ���ǩ�ļ�
     */
    ExteriorElements(const std::string& attDataPath, const std::string& gpsDataPath, const std::string& imgTimePath);

    /**
     * @brief ����ɨ����ʱ���ǩ�ļ�
     *
     * @param filePath ��̬�����ļ�·��
     * @param imageTime ��Ŷ����ʱ���ǩ�ļ����� RelLine Time deltaTime
     */
    void load_imageTime(const std::string& filePath, std::vector<std::tuple<int, double, double>>& imageTime);

    /**
     * @brief ����gps��������ļ�
     *
     * @param filePath gps��������ļ�·��
     * @param gpsData ��Ŷ����gps�����ļ����� timeCode dateTime PX PY PZ VX VY VZ
     */
    void load_gpsData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData);

    /**
     * @brief ������̬�����ļ�
     *
     * @param filePath ��̬�����ļ�·��
     * @param gpsData ��Ŷ������̬�����ļ����� timeCode dateTime roll pitch yaw roll_velocity pitch_velocity yaw_velocity q1 q2 q3 q4
     */
    void load_attData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData);

    /**
     * @brief ������̬�����ļ�
     *
     * @param filePath ��̬�����ļ�·��
     * @param gpsData ��Ŷ������̬�����ļ����� timeCode dateTime roll pitch yaw roll_velocity pitch_velocity yaw_velocity q1 q2 q3 q4
     */
    void transform_time(const std::vector<std::tuple<int, double, double>>& imageTime);

    /**
     * @brief �ڲ���̬���ݵõ�����ʱ����Ԫ��
     *
     * @param imageTime ����ʱ��ʱ���ǩ
     * @param attData ��̬����
     */
    void att_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData);

    /**
     * @brief �ڲ������ݵõ�����ʱ������λ��
     *
     * @param imageTime ����ʱ��ʱ���ǩ
     * @param gpsData �������
     */
    void gps_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData);

public:
    // ����ʱ����̬ q1 q2 q3 q4
    std::vector<Eigen::Vector4d> att;
    // ����ʱ��λ�� PX PY PZ
    std::vector<Eigen::Vector3d> gps;
    // ����ʱ��UTC
    std::vector<std::tuple<int, int, int, int, int, double>> transformed_time;
};

