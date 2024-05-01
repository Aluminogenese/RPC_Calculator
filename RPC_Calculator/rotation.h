#pragma once
#define _USE_MATH_DEFINES 
#include<iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>

#include <eigen3/Eigen/Dense>

class Rotation
{
public:
	/**
	 * @brief ���캯��
	 *
	 * @param RuPath ƫ�þ����ļ�·��
	 * @param directPath ��Ԫָ����ļ�·��
	 * @param Rj2wPath J2000��WGS84����ϵ��ת�����ļ�·��
	 * @param att ��̬���ݣ���Ԫ����q1,q2,q3,q4��
	 */
	Rotation(const std::string& RuPath,
		const std::string& directPath,
		const std::string& Rj2wPath,
		std::vector<Eigen::Vector4d>& att);

	/**
	 * @brief ������Ԫָ����ļ�
	 *
	 * @param filePath ��Ԫָ����ļ�·��
	 * @param directData ��Ŷ������Ԫָ����ļ����� ��ԪID ����ָ��� �ع�ָ���
	 */
	void load_direct_data(const std::string& filePath,
		std::vector<std::tuple<int, double, double>>& directData);

	/**
	 * @brief ��ƫ�þ�������������ϵ�����Ǳ�������ϵ��ת����
	 *
	 * @param filePath ƫ�þ����ļ�·��
	 */
	void calculate_Rc2b(const std::string& filePath);

	/**
	 * @brief ����̬���ݣ���λ��Ԫ�������㱾������ϵ��J2000����ϵ��ת����
	 *
	 * @param att ��̬���ݣ���λ��Ԫ����
	 */
	void calculate_Rb2j(const std::vector<Eigen::Vector4d>& att);

	/**
	 * @brief ���ļ�����J2000����ϵ��WGS84����ϵ��ת����
	 *
	 * @param filePath �ļ�·��
	 */
	void calculate_Rj2w(const std::string& filePath);

	/**
	 * @brief ����Ԫָ��Ǽ�����������tan��y,tan��x,-1)
	 *
	 * @param directData ��Ԫָ�������
	 */
	void get_ux(const std::vector<std::tuple<int, double, double>>& directData);

	/**
	 * @brief �����η���
	 *
	 * @param a,b,b ���η���ϵ��
	 * @return true:�н⣬false���޽�
	 */
	bool solve_quadratic(const double& a, const double& b, const double& c, double& x);

	/**
	 * @brief ����ϸ�ģ�ͼ���������Ƶ�
	 *
	 * @param h ����߳�
	 * @param gps_XYZ GNSS������λ����wgs84����
	 * @param XYZ ������������Ƶ�wgs84����
	 * @param i ��iɨ����
	 * @param j ��iɨ�����ϵ�j�����
	 */
	void calculate_XYZ(const double& h, const Eigen::Vector3d& gps_XYZ, Eigen::Vector3d& XYZ, const int& i, const int& j);

	/**
	 * @brief wgs84����ת�������
	 *
	 * @param XYZ ��������wgs84����
	 * @param BLH ����õ��ĵ�Ĵ������
	 */
	//void convert_wgs84_to_geodetic(double x, double y, double z, double& lat, double& lon, double& alt);
	void convert_wgs84_to_geodetic(const std::vector<Eigen::Vector3d>& XYZ, std::vector<Eigen::Vector3d>& BLH);

	/**
	 * @brief �������תwgs84����
	 *
	 * @param BLH �������Ĵ������
	 * @param XYZ ����õ��ĵ��wgs84����
	 */
	void convert_geodetic_to_wgs84(const std::vector<Eigen::Vector3d>& BLH, std::vector<Eigen::Vector3d>& XYZ);

public:
	// �������ϵ����������ϵ��ת����
	Eigen::Matrix3d R_c2b;
	// ��������ϵ��J2000����ϵ��ת����
	std::vector<Eigen::Matrix3d> R_b2j;
	// J2000����ϵ��WGS-84����ϵ
	std::vector<Eigen::Matrix3d> R_j2w;
	std::vector<Eigen::Vector3d> ux;
};

