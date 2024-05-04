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
		ground_centroid(Eigen::Vector3d::Zero());
		image_scale(Eigen::Vector2d::Zero());
		ground_scale(Eigen::Vector3d::Zero());
		a = Eigen::VectorXd::Ones(20);
		b = Eigen::VectorXd::Ones(20);
		c = Eigen::VectorXd::Ones(20);
		d = Eigen::VectorXd::Ones(20);
	};

	/**
	 * @brief �����׼������
	 *
	 * @param image_points ԭʼ�������
	 * @param ground_points ԭʼ�﷽����
	 * @param image_norm ��׼�����������
	 * @param ground_norm ��׼�����﷽����
	 */
	void calculate_norm_param(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief �����׼��
	 *
	 * @param image_points ԭʼ�������
	 * @param ground_points ԭʼ�﷽����
	 * @param image_norm ��׼�����������
	 * @param ground_norm ��׼�����﷽����
	 */
	void normalize(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points,
		std::vector<Eigen::Vector2d>& image_norm, std::vector<Eigen::Vector3d>& ground_norm);

	/**
	 * @brief ȥ��׼��
	 * 
	 * @param image_norm ��׼�������
	 * @param ground_norm ��׼�﷽����
	 * @param image_points ȥ��׼���������
	 * @param ground_points ȥ��׼���﷽����
	 */
	void de_normalize(const std::vector<Eigen::Vector2d>& image_norm, std::vector<Eigen::Vector2d>& image_points);

	/**
	 * @brief ����RPC����
	 *
	 * @param image_points �������
	 * @param ground_points �﷽����
	 */
	void calculate_RPCs(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief ����RPC����
	 *
	 * @param filePath �ļ�·��
	 */
	void save_RPCs(const std::string& filePath);

	/**
	 * @brief ����RPC���������������
	 *
	 * @param image_points ����õ����������
	 * @param ground_points ���������
	 */
	void calculate_imgPt_from_RPCs(std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points);

	/**
	 * @brief �����ϸ�ģ�ͼ����������
	 *
	 * @param ground_point ���������
	 * @param gps GPS������λ��������
	 * @param R_j2w J2000��WGS-84��ת����
	 * @param R_b2j ���嵽J2000��ת����
	 * @image_point ����õ����������
	 */
	void calculate_imagePt(const Eigen::Vector3d& ground_point, const Eigen::Vector3d& gps, const Eigen::Matrix3d& R_j2w, const Eigen::Matrix3d& R_b2j, Eigen::Vector2d& image_point);

	/**
	 * @brief ���ڶ��ַ����ɨ����������ȡ����
	 *
	 * @param check_points ���ɵļ��������﷽����
	 * @param gps GPS������λ��������
	 * @param R_j2w J2000��WGS-84��ת����
	 * @param R_b2j ���嵽J2000��ת����
	 * @image_points ����õ��ļ���������
	 * @param ground_points ����õ��ļ����﷽����
	 */
	void generate_checkPts(const std::vector<Eigen::Vector3d>& check_points,
		const std::vector<Eigen::Vector3d>& gps,
		const std::vector<Eigen::Matrix3d>& R_j2w,
		const std::vector<Eigen::Matrix3d>& R_b2j,
		const std::vector<Eigen::Vector3d>& ux,
		std::vector<Eigen::Vector2d>& image_points,
		std::vector<Eigen::Vector3d>& ground_points);
public:
	// �񷽱�׼��ƽ�Ʋ���
	Eigen::Vector2d image_centroid;
	// �﷽��׼��ƽ�Ʋ���
	Eigen::Vector3d ground_centroid;
	// �񷽱�׼����������
	Eigen::Vector2d image_scale;
	//�﷽��׼����������
	Eigen::Vector3d ground_scale;
	//RPC����
	Eigen::VectorXd a;// Line_Num
	Eigen::VectorXd b;// Line_Den
	Eigen::VectorXd c;// Sample_Num
	Eigen::VectorXd d;// Sample_Den
};

