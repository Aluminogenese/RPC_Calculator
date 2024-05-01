#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

class ClassRPC
{
public:
	/**
	 * @brief ���캯��
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
	 * @brief ��׼������
	 *
	 * @param image_points ԭʼ�������
	 * @param control_points ԭʼ�﷽����
	 * @param image_barycentric ��׼�����������
	 * @param control_barycentric ��׼�����﷽����
	 */
	void barycentric(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points,
		std::vector<Eigen::Vector2d>& image_barycentric, std::vector<Eigen::Vector3d>& control_barycentric);

	/**
	 * @brief ����RPC����
	 *
	 * @param image_points �������
	 * @param control_points �﷽����
	 */
	void calculate_RPC(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points);

	/**
	 * @brief ����RPC����
	 *
	 * @param filePath �ļ�·��
	 */
	void save_RPCs(const std::string& filePath);

public:
	// �񷽱�׼��ƽ�Ʋ���
	Eigen::Vector2d image_centroid;
	// �﷽��׼��ƽ�Ʋ���
	Eigen::Vector3d control_centroid;
	// �񷽱�׼����������
	Eigen::Vector2d image_scale;
	//�﷽��׼����������
	Eigen::Vector3d control_scale;
	//RPC����
	Eigen::VectorXd a;// Line_Num
	Eigen::VectorXd b;// Line_Den
	Eigen::VectorXd c;// Sample_Num
	Eigen::VectorXd d;// Sample_Den
};

