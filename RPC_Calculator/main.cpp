#include <iostream>
#include "exterior_elements.h"
#include "rotation.h"
#include "class_RPC.h"
#include "precise_check.h"
#include <iomanip>

int main() {
	// 参数设定
	int check_grid_m = 20;
	int check_grid_n = 20;
	int ctrl_grid_m = 10;
	int ctrl_grid_n = 10;
	int ctrl_layers = 6;
	int img_col = 8192;
	int img_row = 5378;
	double max_height = 947;
	double min_height = 41;

	std::string path_range = "../inter_result/range.txt";
	std::string path_RPC = "../results/result.txt";
	std::string path_utcTime = "../inter_result/utc_imgTime.txt";
	const char* path_dem = "../data/n35_e114_1arc_v3.tif";
	std::string path_checkResult = "../results/check.txt";

	// 外方位元素
	std::string att_path = "../data/DX_ZY3_NAD_att.txt";
	std::string gps_path = "../data/DX_ZY3_NAD_gps.txt";
	std::string imageTime_path = "../data/DX_ZY3_NAD_imagingTime.txt";
	ExteriorElements ex_element(att_path, gps_path, imageTime_path);

	// 计算utc时间，用于J2000到WGS84坐标转换矩阵计算
	std::ofstream utc_file(path_utcTime);
	for (const auto&utc_time: ex_element.transformed_time) {
		utc_file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << std::get<0>(utc_time) << " " << std::get<1>(utc_time) << " " << std::get<2>(utc_time) << " "
			<< std::get<3>(utc_time) << " " << std::get<4>(utc_time) << " " << std::get<5>(utc_time) << std::endl;
	}
	utc_file.close();
	
	// 旋转矩阵
	std::string direct_path = "../data/NAD.cbr";
	std::string Ru_path = "../data/NAD.txt";
	std::string Rj2w_path = "../data/J2000_2_wgs84.txt";
	Rotation rotation_hanler(Ru_path, direct_path, Rj2w_path, ex_element.att);

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
	// WGS84转大地坐标
	std::vector<Eigen::Vector3d>ground_points_blh;
	rotation_hanler.convert_wgs84_to_geodetic(ground_points, ground_points_blh);

	double max_x = 0., max_y = 0.;
	double min_x = ground_points_blh[0].x(), min_y = ground_points_blh[0].y();
	for (const auto& point : ground_points_blh) {
		max_x = std::max(max_x, point.x());
		max_y = std::max(max_y, point.y());
		min_x = std::min(min_x, point.x());
		min_y = std::min(min_y, point.y());
	}
	// 保存控制点覆盖范围，用于检查格网点生成
	std::ofstream gcp_range(path_range);
	gcp_range << max_x << " " << min_x << " " << max_y << " " << min_y;
	gcp_range.close();

	ClassRPC rpc;
	std::vector<Eigen::Vector2d> image_norm;
	std::vector<Eigen::Vector3d> ground_norm;

	rpc.calculate_norm_param(extent_image_points, ground_points_blh);
	rpc.normalize(extent_image_points, ground_points_blh, image_norm, ground_norm);

	rpc.calculate_RPCs(image_norm, ground_norm);
	rpc.save_RPCs(path_RPC);

	// 获取检查格网点
	std::vector<Eigen::Vector3d> check_gridPts;
	PreciseCheck::get_check_grid(path_range, path_dem, check_grid_m, check_grid_n, check_gridPts);

	// 转为wgs84坐标
	std::vector<Eigen::Vector3d>check_gridPtsWGS84;
	rotation_hanler.convert_geodetic_to_wgs84(check_gridPts, check_gridPtsWGS84);

	// 利用严格模型计算有效检查点（像方与物方）
	std::vector<Eigen::Vector2d> check_imagePts;
	std::vector<Eigen::Vector3d> check_groundPts;
	rpc.generate_checkPts(check_gridPtsWGS84, ex_element.gps, rotation_hanler.R_j2w, rotation_hanler.R_b2j, rotation_hanler.ux, check_imagePts, check_groundPts);
	// 转为大地坐标BLH
	std::vector<Eigen::Vector3d>check_groundBLH;
	rotation_hanler.convert_wgs84_to_geodetic(check_groundPts, check_groundBLH);

	// 标准化
	std::vector<Eigen::Vector2d> check_image_norm;
	std::vector<Eigen::Vector3d> check_ground_norm;
	rpc.normalize(check_imagePts, check_groundBLH, check_image_norm, check_ground_norm);
	// 利用RPC模型计算像点坐标
	std::vector<Eigen::Vector2d> check_image_predNorm;
	rpc.calculate_imgPt_from_RPCs(check_image_predNorm, check_ground_norm);
	// 去归一化
	std::vector<Eigen::Vector2d> image_pred_deNorm;
	rpc.de_normalize(check_image_predNorm, image_pred_deNorm);
	// 计算平均绝对误差（MAE）
	double error_row = 0., error_col = 0.;
	for (int i = 0; i < check_image_norm.size(); ++i) {
		error_row += std::abs(image_pred_deNorm[i][0] - check_imagePts[i][0]);
		error_col += std::abs(image_pred_deNorm[i][1] - check_imagePts[i][1]);
	}
	error_row /= image_pred_deNorm.size();
	error_col /= image_pred_deNorm.size();
	std::ofstream file_check(path_checkResult);
	file_check << "MAE in row:" << error_row << std::endl;
	file_check << "MAE in col:" << error_col << std::endl;

	return 0;
}