#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

class ClassRPC
{
public:
	/**
	 * @brief 构造函数
	 */
	ClassRPC(){
		image_centroid(Eigen::Vector2d::Zero());
		control_centroid(Eigen::Vector3d::Zero());
		image_scale(Eigen::Vector2d::Zero());
		control_scale(Eigen::Vector3d::Zero());
		a = Eigen::VectorXd::Ones(20);
		b = Eigen::VectorXd::Ones(20);
		c = Eigen::VectorXd::Ones(20);
		d = Eigen::VectorXd::Ones(20);
	};
	/**
	 * @brief 标准化坐标
	 *
	 * @param image_points 原始像点坐标
	 * @param control_points 原始物方坐标
	 * @param image_barycentric 标准化后像点坐标
	 * @param control_barycentric 标准化后物方坐标
	 */
	void barycentric(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points,
		std::vector<Eigen::Vector2d>& image_barycentric, std::vector<Eigen::Vector3d>& control_barycentric);

	/**
	 * @brief 计算RPC参数
	 *
	 * @param image_points 像点坐标
	 * @param control_points 物方坐标
	 */
	void calculate_RPC(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points);

	/**
	 * @brief 保存RPC参数
	 *
	 * @param filePath 文件路径
	 */
	void save_RPCs(const std::string& filePath);

public:
	// 像方标准化平移参数
	Eigen::Vector2d image_centroid;
	// 物方标准化平移参数
	Eigen::Vector3d control_centroid;
	// 像方标准化比例参数
	Eigen::Vector2d image_scale;
	//物方标准化比例参数
	Eigen::Vector3d control_scale;
	//RPC参数
	Eigen::VectorXd a;// Line_Num
	Eigen::VectorXd b;// Line_Den
	Eigen::VectorXd c;// Sample_Num
	Eigen::VectorXd d;// Sample_Den
};

