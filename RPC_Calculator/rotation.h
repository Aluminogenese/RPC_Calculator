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
	 * @brief 构造函数
	 *
	 * @param RuPath 偏置矩阵文件路径
	 * @param directPath 像元指向角文件路径
	 * @param Rj2wPath J2000与WGS84坐标系旋转矩阵文件路径
	 * @param att 姿态数据（四元数：q1,q2,q3,q4）
	 */
	Rotation(const std::string& RuPath,
		const std::string& directPath,
		const std::string& Rj2wPath,
		std::vector<Eigen::Vector4d>& att);

	/**
	 * @brief 加载像元指向角文件
	 *
	 * @param filePath 像元指向角文件路径
	 * @param directData 存放读入的像元指向角文件数组 像元ID 垂轨指向角 沿轨指向角
	 */
	void load_direct_data(const std::string& filePath,
		std::vector<std::tuple<int, double, double>>& directData);

	/**
	 * @brief 由偏置矩阵计算相机坐标系到卫星本体坐标系旋转矩阵
	 *
	 * @param filePath 偏置矩阵文件路径
	 */
	void calculate_Rc2b(const std::string& filePath);

	/**
	 * @brief 由姿态数据（单位四元数）计算本体坐标系到J2000坐标系旋转矩阵
	 *
	 * @param att 姿态数据（单位四元数）
	 */
	void calculate_Rb2j(const std::vector<Eigen::Vector4d>& att);

	/**
	 * @brief 从文件导入J2000坐标系到WGS84坐标系旋转矩阵
	 *
	 * @param filePath 文件路径
	 */
	void calculate_Rj2w(const std::string& filePath);

	/**
	 * @brief 由像元指向角计算视向量（tanφy,tanφx,-1)
	 *
	 * @param directData 像元指向角数据
	 */
	void get_ux(const std::vector<std::tuple<int, double, double>>& directData);

	/**
	 * @brief 求解二次方程
	 *
	 * @param a,b,b 二次方程系数
	 * @return true:有解，false：无解
	 */
	bool solve_quadratic(const double& a, const double& b, const double& c, double& x);

	/**
	 * @brief 相机严格模型计算虚拟控制点
	 *
	 * @param h 虚拟高程
	 * @param gps_XYZ GNSS天线相位中心wgs84坐标
	 * @param XYZ 待计算虚拟控制点wgs84坐标
	 * @param i 第i扫描行
	 * @param j 第i扫描行上第j个像点
	 */
	void calculate_XYZ(const double& h, const Eigen::Vector3d& gps_XYZ, Eigen::Vector3d& XYZ, const int& i, const int& j);

	/**
	 * @brief wgs84坐标转大地坐标
	 *
	 * @param XYZ 待计算点的wgs84坐标
	 * @param BLH 计算得到的点的大地坐标
	 */
	//void convert_wgs84_to_geodetic(double x, double y, double z, double& lat, double& lon, double& alt);
	void convert_wgs84_to_geodetic(const std::vector<Eigen::Vector3d>& XYZ, std::vector<Eigen::Vector3d>& BLH);

	/**
	 * @brief 大地坐标转wgs84坐标
	 *
	 * @param BLH 待计算点的大地坐标
	 * @param XYZ 计算得到的点的wgs84坐标
	 */
	void convert_geodetic_to_wgs84(const std::vector<Eigen::Vector3d>& BLH, std::vector<Eigen::Vector3d>& XYZ);

public:
	// 相机坐标系到本体坐标系旋转矩阵
	Eigen::Matrix3d R_c2b;
	// 本体坐标系到J2000坐标系旋转矩阵
	std::vector<Eigen::Matrix3d> R_b2j;
	// J2000坐标系到WGS-84坐标系
	std::vector<Eigen::Matrix3d> R_j2w;
	std::vector<Eigen::Vector3d> ux;
};

