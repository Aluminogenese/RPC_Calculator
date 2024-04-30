#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <eigen3/Eigen/Dense>

class ClassRPC
{
public:
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
	void barycentric(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points, std::vector<Eigen::Vector2d>& image_barycentric, std::vector<Eigen::Vector3d>& control_barycentric);
	void calculate_RPC(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points);
	void save_RPCs(const std::string& filePath);
public:
	Eigen::Vector2d image_centroid;
	Eigen::Vector3d control_centroid;

	Eigen::Vector2d image_scale;
	Eigen::Vector3d control_scale;

	Eigen::VectorXd a;
	Eigen::VectorXd b;
	Eigen::VectorXd c;
	Eigen::VectorXd d;
};

