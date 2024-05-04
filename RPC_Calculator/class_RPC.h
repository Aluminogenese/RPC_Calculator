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
		ground_centroid(Eigen::Vector3d::Zero());
		image_scale(Eigen::Vector2d::Zero());
		ground_scale(Eigen::Vector3d::Zero());
		a = Eigen::VectorXd::Ones(20);
		b = Eigen::VectorXd::Ones(20);
		c = Eigen::VectorXd::Ones(20);
		d = Eigen::VectorXd::Ones(20);
	};

	/**
	 * @brief 计算标准化参数
	 *
	 * @param image_points 原始像点坐标
	 * @param ground_points 原始物方坐标
	 * @param image_norm 标准化后像点坐标
	 * @param ground_norm 标准化后物方坐标
	 */
	void calculate_norm_param(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief 坐标标准化
	 *
	 * @param image_points 原始像点坐标
	 * @param ground_points 原始物方坐标
	 * @param image_norm 标准化后像点坐标
	 * @param ground_norm 标准化后物方坐标
	 */
	void normalize(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points,
		std::vector<Eigen::Vector2d>& image_norm, std::vector<Eigen::Vector3d>& ground_norm);

	/**
	 * @brief 去标准化
	 * 
	 * @param image_norm 标准像点坐标
	 * @param ground_norm 标准物方坐标
	 * @param image_points 去标准化像点坐标
	 * @param ground_points 去标准化物方坐标
	 */
	void de_normalize(const std::vector<Eigen::Vector2d>& image_norm, std::vector<Eigen::Vector2d>& image_points);

	/**
	 * @brief 计算RPC参数
	 *
	 * @param image_points 像点坐标
	 * @param ground_points 物方坐标
	 */
	void calculate_RPCs(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief 保存RPC参数
	 *
	 * @param filePath 文件路径
	 */
	void save_RPCs(const std::string& filePath);

	/**
	 * @brief 利用RPC参数计算像点坐标
	 *
	 * @param image_points 计算得到的像点坐标
	 * @param ground_points 地面点坐标
	 */
	void calculate_imgPt_from_RPCs(std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief 利用严格模型计算像点坐标
	 *
	 * @param ground_point 地面点坐标
	 * @param gps GPS天线相位中心坐标
	 * @param R_j2w J2000到WGS-84旋转矩阵
	 * @param R_b2j 本体到J2000旋转矩阵
	 * @image_point 计算得到的像点坐标
	 */
	void calculate_imagePt(const Eigen::Vector3d& ground_point, const Eigen::Vector3d& gps, const Eigen::Matrix3d& R_j2w, const Eigen::Matrix3d& R_b2j, Eigen::Vector2d& image_point);

	/**
	 * @brief 窗口二分法最佳扫描线搜索获取检查点
	 *
	 * @param check_points 生成的检查格网点物方坐标
	 * @param gps GPS天线相位中心坐标
	 * @param R_j2w J2000到WGS-84旋转矩阵
	 * @param R_b2j 本体到J2000旋转矩阵
	 * @image_points 计算得到的检查点像方坐标
	 * @param ground_points 计算得到的检查点物方坐标
	 */
	void generate_checkPts(const std::vector<Eigen::Vector3d>& check_points,
		const std::vector<Eigen::Vector3d>& gps,
		const std::vector<Eigen::Matrix3d>& R_j2w,
		const std::vector<Eigen::Matrix3d>& R_b2j,
		const std::vector<Eigen::Vector3d>& ux,
		std::vector<Eigen::Vector2d>& image_points,
		std::vector<Eigen::Vector3d>& ground_points);
public:
	// 像方标准化平移参数
	Eigen::Vector2d image_centroid;
	// 物方标准化平移参数
	Eigen::Vector3d ground_centroid;
	// 像方标准化比例参数
	Eigen::Vector2d image_scale;
	//物方标准化比例参数
	Eigen::Vector3d ground_scale;
	//RPC参数
	Eigen::VectorXd a;// Line_Num
	Eigen::VectorXd b;// Line_Den
	Eigen::VectorXd c;// Sample_Num
	Eigen::VectorXd d;// Sample_Den
};

