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
	Rotation(const std::string& RuPath,
		const std::string& directPath,
		const std::string& rotPath,
		std::vector<Eigen::Vector4d>& att);

	void load_direct_data(const std::string& filePath,
		std::vector<std::tuple<int, double, double>>& directData);

	void calculate_Rc2b(const std::string& filePath);

	void calculate_Rb2j(const std::vector<Eigen::Vector4d>& att);

	void calculate_Rj2w(const std::string& filePath);

	void get_ux(const std::vector<std::tuple<int, double, double>>& directData);

	bool solve_quadratic(const double& a, const double& b, const double& c, double& x);

	void calculate_XYZ(const double& h, const Eigen::Vector3d& gps_XYZ, Eigen::Vector3d& XYZ, const int& i, const int& j);

	//void convert_wgs84_to_geodetic(double x, double y, double z, double& lat, double& lon, double& alt);
	void convert_wgs84_to_geodetic(const std::vector<Eigen::Vector3d>& XYZ, std::vector<Eigen::Vector3d>& BLH);

	void convert_geodetic_to_wgs84(double lat, double lon, double alt, double& x, double& y, double& z);

	// 相机坐标系到本体坐标系旋转矩阵
	Eigen::Matrix3d R_c2b;
	// 本体坐标系到J2000坐标系旋转矩阵
	std::vector<Eigen::Matrix3d> R_b2j;
	// J2000坐标系到WGS-84坐标系
	std::vector<Eigen::Matrix3d> R_j2w;
	std::vector<Eigen::Vector3d> ux;
	//std::vector<Eigen::Vector3d> gps_XYZ;
	std::vector<Eigen::Vector3d> XYZ;
};

