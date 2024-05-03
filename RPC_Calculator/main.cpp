#include <iostream>
#include "exterior_elements.h"
#include "rotation.h"
#include "class_RPC.h"
#include "precise_check.h"

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

	//std::ofstream b2j_file("../inter_result/b2j.txt");
	//for (const auto& b2j : rotation_hanler.R_b2j) {
	//	b2j_file << b2j << "\n";
	//}
	//b2j_file.close();
	//std::ofstream j2w_file("../inter_result/j2w.txt");
	//for (const auto& j2w : rotation_hanler.R_j2w) {
	//	j2w_file << j2w << "\n";
	//}
	//j2w_file.close();
	//std::ofstream ux_file("../inter_result/ux.txt");
	//for (const auto& ux_ : rotation_hanler.ux) {
	//	ux_file << ux_ << "\n";
	//}
	//ux_file.close();

	// 像方划分格网
	std::vector<Eigen::Vector2d> image_points;
	for (int j = 0; j <= ctrl_grid_n; ++j) {
		for (int i = 0; i <= ctrl_grid_m; ++i) {
			image_points.push_back(Eigen::Vector2d((img_row - 1) / ctrl_grid_n * j, (img_col - 1) / ctrl_grid_m * i));
		}
	}
	// 6层虚拟格网复制6份
	std::vector<Eigen::Vector2d> extent_image_points;
	for (int i = 0; i < ctrl_layers; i++) {
		extent_image_points.insert(extent_image_points.end(), image_points.begin(), image_points.end());
	}

	//std::ofstream img_file("../inter_result/img_points.txt");
	//for (const auto& image_point : extent_image_points) {
	//	img_file << image_point.x() << " " << image_point.y() << "\n";
	//}
	//img_file.close();
	
	// 物方格网点
	std::vector<Eigen::Vector3d>ground_points;
	//#pragma omp parallel for
	for (int l = 0; l < ctrl_layers; l++) {
		double z0 = min_height + l * (max_height - min_height) / (ctrl_layers - 1);
		for (const auto& image_point : image_points) {
			int i = static_cast<int>(image_point.x());
			int j = static_cast<int>(image_point.y());

			Eigen::Vector3d ground_point;
			rotation_hanler.calculate_XYZ(z0, ex_element.gps[i], ground_point, i, j);
			//#pragma omp critical
			ground_points.push_back(ground_point);
		}
	}
	//std::ofstream gcp_file("../inter_result/ground_points.txt");
	//for (const auto& ground_point : ground_points) {
	//	double X = ground_point.x(), Y = ground_point.y(), Z = ground_point.z();
	//	gcp_file << std::fixed << X << "\t" << Y << "\t" << Z << "\n";
	//}
	//gcp_file.close();

	std::vector<Eigen::Vector3d>ground_points_blh;
	//std::ofstream gcpblh_file("../inter_result/ground_points_BLH.txt");
	rotation_hanler.convert_wgs84_to_geodetic(ground_points, ground_points_blh);
	for (const auto& ground_point : ground_points_blh) {
		double lat = ground_point.x(), lon = ground_point.y(), alt = ground_point.z();
		//gcpblh_file << std::fixed << lat << "\t" << lon << "\t" << alt << "\n";
	}
	//gcpblh_file.close();

	ClassRPC rpc;
	std::vector<Eigen::Vector2d> image_norm;
	std::vector<Eigen::Vector3d> ground_norm;

	rpc.calculate_norm_param(extent_image_points, ground_points_blh);
	rpc.normalize(extent_image_points, ground_points_blh, image_norm, ground_norm);

	//std::ofstream image_bary_file("../inter_result/image_barycentric.txt");
	//for (const auto& image_bary : image_norm) {
	//	double X = image_bary.x(), Y = image_bary.y();
	//	image_bary_file << std::fixed << X << "\t" << Y  << "\n";
	//}
	//image_bary_file.close();

	//std::ofstream ground_bary_file("../inter_result/ground_barycentric.txt");
	//for (const auto& ground_bary : ground_norm) {
	//	double X = ground_bary.x(), Y = ground_bary.y(), Z = ground_bary.z();
	//	ground_bary_file << std::fixed << X << "\t" << Y << "\t" << Z << "\n";
	//}
	//ground_bary_file.close();
	rpc.calculate_RPCs(image_norm, ground_norm);
	rpc.save_RPCs("../inter_result/result.txt");

	std::string path_range = "../data/range.txt";
	const char* path_dem = "../data/n35_e114_1arc_v3.tif";
	// 获取检查点
	std::vector<Eigen::Vector3d> check_points;
	PreciseCheck::get_check_grid(path_range, path_dem, check_grid_m, check_grid_n, check_points);
	// 转为wgs84坐标
	std::vector<Eigen::Vector3d>check_Ptswgs;
	rotation_hanler.convert_geodetic_to_wgs84(check_points, check_Ptswgs);
	// 利用严格模型计算检查点
	std::vector<Eigen::Vector2d> check_image;
	std::vector<Eigen::Vector3d> check_ground;
	rpc.check(check_Ptswgs, ex_element.gps, rotation_hanler.R_j2w, rotation_hanler.R_b2j, rotation_hanler.ux, check_image, check_ground);
	// 转为大地坐标BLH
	std::vector<Eigen::Vector3d>check_groundBLH;
	rotation_hanler.convert_wgs84_to_geodetic(check_ground, check_groundBLH);
	// 标准化
	std::vector<Eigen::Vector2d> check_image_norm;
	std::vector<Eigen::Vector3d> check_ground_norm;
	rpc.normalize(check_image, check_groundBLH, check_image_norm, check_ground_norm);
	// 利用RPC模型计算像点坐标
	std::vector<Eigen::Vector2d> check_image_predNorm;
	rpc.calculate_imgPt_from_RPCs(check_image_predNorm, check_ground_norm);

	double error_row = 0., error_col = 0.;
	for (int i = 0; i < check_image_norm.size(); ++i) {
		error_row += std::abs(check_image_predNorm[i][0] - check_image_norm[i][0]);
		error_col += std::abs(check_image_predNorm[i][1] - check_image_norm[i][1]);
	}
	error_row /= check_image_norm.size();
	error_col /= check_image_norm.size();
	std::cout << "MAE in row:" << error_row << std::endl;
	std::cout << "MAE in col:" << error_col << std::endl;

	return 0;
}