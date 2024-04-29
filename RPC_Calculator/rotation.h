#pragma once
#include<iostream>
#include <string>
#include <vector>
#include <fstream>

#include <eigen3/Eigen/Dense>
class Rotation
{
public:
	Rotation(const std::string& Rupath, const std::string directPath, const std::string& rotPath);
	void load_direct_data(const std::string& filePath, std::vector<std::tuple<int, double, double>>& directData);
	void calculate_Rc2b(const std::string& filePath, Eigen::Matrix3d& R_c2b);
	void calculate_Rb2j(const std::vector<std::tuple<double, double, double, double>>& att, std::vector<Eigen::Matrix3d>& R_b2j);
	void load_Rj2w(const std::string& rotPath, std::vector<Eigen::Matrix3d>& R_j2w);
	void get_ux(const std::vector<std::tuple<int, double, double>>& directData, std::vector<Eigen::Vector3d>& ux);
	void result(const double& a, const double& b, const double& c);
	void get_M();
	void getXYZ();
	void XYZ2BLH();
	void XYZ2BLH2();
	void BLH2XYZ();

	// �������ϵ����������ϵ��ת����
	Eigen::Matrix3d R_c2b;
	// ��������ϵ��J2000����ϵ��ת����
	std::vector<Eigen::Matrix3d> R_b2j;
	// J2000����ϵ��WGS-84����ϵ
	std::vector<Eigen::Matrix3d> R_j2w;
	std::vector<Eigen::Vector3d> ux;
};

