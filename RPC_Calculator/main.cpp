#include <iostream>
#include "exterior_elements.h"
#include "rotation.h"
#include "class_RPC.h"

int main() {
	// 参数设定
	int check_grid_m = 20;
	int check_grid_n = 20;
	int ctrl_grid_m = 10;
	int ctrl_grid_n = 10;
	int ctrl_layers = 6;
	int img_col = 8192;
	int img_row = 5378;
	int iter_time = 20;
	double max_height = 947;
	double min_height = 41;
	float threshold = 0.0015;
	// 外方位元素
	std::string att_path = "../data/DX_ZY3_NAD_att.txt";
	std::string gps_path = "../data/DX_ZY3_NAD_gps.txt";
	std::string image_time_path = "../data/DX_ZY3_NAD_imagingTime.txt";
	ExteriorElements ex_element(att_path, gps_path, image_time_path);
	// 旋转矩阵
	std::string direct_path = "../data/NAD.cbr";
	std::string Ru_path = "../data/NAD.txt";
	std::string Rj2w_path = "../data/rot.txt";
	Rotation rotation_hanler(Ru_path, direct_path, Rj2w_path, ex_element.att);

	std::ofstream b2j_file("../inter_result/b2j.txt");
	for (const auto& b2j : rotation_hanler.R_b2j) {
		b2j_file << b2j << "\n";
	}
	b2j_file.close();
	std::ofstream j2w_file("../inter_result/j2w.txt");
	for (const auto& j2w : rotation_hanler.R_j2w) {
		j2w_file << j2w << "\n";
	}
	j2w_file.close();
	std::ofstream ux_file("../inter_result/ux.txt");
	for (const auto& ux_ : rotation_hanler.ux) {
		ux_file << ux_ << "\n";
	}
	ux_file.close();
	// 像方划分格网
	std::ofstream img_file("../inter_result/img_points.txt");
	std::vector<Eigen::Vector2d> image_points;
	for (int j = 0; j <= ctrl_grid_n; ++j) {
		for (int i = 0; i <= ctrl_grid_m; ++i) {
			image_points.push_back(Eigen::Vector2d((img_row - 1) / ctrl_grid_n * j, (img_col - 1) / ctrl_grid_m * i));
			img_file << (img_row - 1) / ctrl_grid_n * j << " " << (img_col - 1) / ctrl_grid_m * i << "\n";
		}
	}
	img_file.close();
	std::vector<Eigen::Vector2d> extent_image_points;
	for (int i = 0; i < ctrl_layers; i++) {
		extent_image_points.insert(extent_image_points.end(), image_points.begin(), image_points.end());
	}
	// 物方格网点
	std::vector<Eigen::Vector3d>ground_points;
	//#pragma omp parallel for
	for (int l = 0; l < ctrl_layers; l++) {
		double z0 = min_height + l * (max_height - min_height) / (ctrl_layers - 1);
		for (const auto& image_point : extent_image_points) {
			int i = static_cast<int>(image_point.x());
			int j = static_cast<int>(image_point.y());

			Eigen::Vector3d ground_point;
			rotation_hanler.calculate_XYZ(z0, ex_element.gps[i], ground_point, i, j);
			//#pragma omp critical
			ground_points.push_back(ground_point);
		}
	}


	std::vector<Eigen::Vector3d>ground_points_blh;
	std::ofstream gcp_file("../inter_result/ground_points.txt");
	rotation_hanler.convert_wgs84_to_geodetic(ground_points, ground_points_blh);
	for (const auto& ground_point : ground_points_blh) {
		double lat = ground_point.x(), lon = ground_point.y(), alt = ground_point.z();
		gcp_file << lat << "\t" << lon << "\t" << alt << "\n";
	}
	gcp_file.close();

	ClassRPC rpc;
	std::vector<Eigen::Vector2d> image_barycentric;
	std::vector<Eigen::Vector3d> ground_barycentric;

	rpc.barycentric(image_points, ground_points_blh, image_barycentric, ground_barycentric);
	rpc.calculate_RPC(image_barycentric, ground_barycentric);
	rpc.save_RPCs("../inter_result/result.txt");
	return 0;
}