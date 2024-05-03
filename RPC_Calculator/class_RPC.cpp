#include "class_RPC.h"

void ClassRPC::calculate_norm_param(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points)
{
	double max_x = 0., max_y = 0., max_X = 0., max_Y = 0., max_Z = 0.;
	double min_x = image_points[0].x(), min_y = image_points[0].y(), min_X = ground_points[0].x(), min_Y = ground_points[0].y(), min_Z = ground_points[0].z();
	for (int i = 0; i < image_points.size(); i++) {
		image_centroid += image_points[i];
		ground_centroid += ground_points[i];

		max_x = std::max(max_x, image_points[i].x());
		max_y = std::max(max_y, image_points[i].y());
		max_X = std::max(max_X, ground_points[i].x());
		max_Y = std::max(max_Y, ground_points[i].y());
		max_Z = std::max(max_Z, ground_points[i].z());

		min_x = std::min(min_x, image_points[i].x());
		min_y = std::min(min_y, image_points[i].y());
		min_X = std::min(min_X, ground_points[i].x());
		min_Y = std::min(min_Y, ground_points[i].y());
		min_Z = std::min(min_Z, ground_points[i].z());
	}
	// 标准化平移参数
	image_centroid /= image_points.size();
	ground_centroid /= ground_points.size();
	// 标准化比例参数
	image_scale.x() = std::max(std::abs(max_x - image_centroid.x()), std::abs(min_x - image_centroid.x()));
	image_scale.y() = std::max(std::abs(max_y - image_centroid.y()), std::abs(min_y - image_centroid.y()));

	ground_scale.x() = std::max(std::abs(max_X - ground_centroid.x()), std::abs(min_X - ground_centroid.x()));
	ground_scale.y() = std::max(std::abs(max_Y - ground_centroid.y()), std::abs(min_Y - ground_centroid.y()));
	ground_scale.z() = std::max(std::abs(max_Z - ground_centroid.z()), std::abs(min_Z - ground_centroid.z()));
}

void ClassRPC::normalize(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points, std::vector<Eigen::Vector2d>& image_norm, std::vector<Eigen::Vector3d>& ground_norm)
{
	for (int i = 0; i < image_points.size(); i++) {
		Eigen::Vector2d xn;
		Eigen::Vector3d Pn;
		xn = image_points[i] - image_centroid;
		Pn = ground_points[i] - ground_centroid;
		image_norm.push_back(Eigen::Vector2d(xn.x() / image_scale.x(), xn.y() / image_scale.y()));
		ground_norm.push_back(Eigen::Vector3d(Pn.x() / ground_scale.x(), Pn.y() / ground_scale.y(), Pn.z() / ground_scale.z()));
	}
}

void ClassRPC::de_normalize(const std::vector<Eigen::Vector2d>& image_norm, const std::vector<Eigen::Vector3d>& ground_norm, std::vector<Eigen::Vector2d>& image_points, std::vector<Eigen::Vector3d>& ground_points)
{
	for (int i = 0; i < image_norm.size(); i++) {
		image_points.push_back(image_centroid + Eigen::Vector2d(image_norm[i].x() * image_scale.x(), image_norm[i].y() * image_scale.y()));
		ground_points.push_back(ground_centroid + Eigen::Vector3d(ground_norm[i].x() * ground_scale.x(), ground_norm[i].y() * ground_scale.y(), ground_norm[i].z() * ground_scale.z()));
	}
}

void ClassRPC::calculate_RPCs(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points)
{
	int num = image_points.size();
	Eigen::VectorXd R(num), C(num);
	Eigen::MatrixXd M(num, 39), N(num, 39);
	Eigen::MatrixXd W_r = Eigen::MatrixXd::Identity(num, num), W_c = Eigen::MatrixXd::Identity(num, num);
	for (int i = 0; i < ground_points.size(); i++) {
		R(i) = image_points[i].x();
		C(i) = image_points[i].y();
		//G(i) = image_points[i].x();
		//G(i + num) = image_points[i].y();
		double X = ground_points[i].x();
		double Y = ground_points[i].y();
		double Z = ground_points[i].z();
		Eigen::VectorXd vec1(20), vec2;
		vec1 << 1, Y, X, Z, Y* X, Y* Z, X* Z, Y* Y, X* X, Z* Z, X* Y* Z, Y* Y* Y, Y* X* X, Y* Z* Z, Y* Y* X, X* X* X, X* Z* Z, Y* Y* Z, X* X* Z, Z* Z* Z;
		//vec << 1, Z, Y, X, Z* Y, Z* X, Y* X, Z* Z, Y* Y, X* X, Z* Y* X, Z* Z* Y, Z* Z* X, Y* Y* Z, Y* Y* X, Z* X* X, Y* X* X, Z* Z* Z, Y* Y* Y, X* X* X;
		vec2 = vec1.tail(19);

		double B = vec1.dot(b);
		M.row(i) << vec1.transpose(), -image_points[i].x() * vec2.transpose();

		double D = vec1.dot(d);
		N.row(i) << vec1.transpose(), -image_points[i].y() * vec2.transpose();
	}
	// 行列分别解算
	Eigen::MatrixXd A_r = M.transpose() * W_r * M;
	Eigen::MatrixXd L_r = M.transpose() * W_r * R;
	Eigen::VectorXd J = A_r.inverse() * L_r;

	Eigen::MatrixXd A_c = N.transpose() * W_c * N;
	Eigen::MatrixXd L_c = N.transpose() * W_c * C;
	Eigen::VectorXd K = A_c.inverse() * L_c;
	// 求初值
	a = J.segment(0, 20);
	b.segment(1, 19) = J.segment(20, 19);
	c = K.segment(0, 20);
	d.segment(1, 19) = K.segment(20, 19);

	for (int iter = 0; iter < 20; iter++) {
		for (int i = 0; i < ground_points.size(); i++) {
			R(i) = image_points[i].x();
			C(i) = image_points[i].y();

			double X = ground_points[i].x();
			double Y = ground_points[i].y();
			double Z = ground_points[i].z();
			Eigen::VectorXd vec1(20), vec2;
			vec1 << 1, Y, X, Z, Y* X, Y* Z, X* Z, Y* Y, X* X, Z* Z, X* Y* Z, Y* Y* Y, Y* X* X, Y* Z* Z, Y* Y* X, X* X* X, X* Z* Z, Y* Y* Z, X* X* Z, Z* Z* Z;
			//vec << 1, Z, Y, X, Z* Y, Z* X, Y* X, Z* Z, Y* Y, X* X, Z* Y* X, Z* Z* Y, Z* Z* X, Y* Y* Z, Y* Y* X, Z* X* X, Y* X* X, Z* Z* Z, Y* Y* Y, X* X* X;
			vec2 = vec1.tail(19);

			double B = vec1.dot(b);
			M.row(i) << vec1.transpose(), -image_points[i].x() * vec2.transpose();
			W_r(i, i) = 1 / B;

			double D = vec1.dot(d);
			N.row(i) << vec1.transpose(), -image_points[i].y() * vec2.transpose();
			W_c(i, i) = 1 / D;
		}
		// 行列分别解算
		A_r = M.transpose() * W_r * M;
		L_r = M.transpose() * W_r * R;
		J = A_r.inverse() * L_r;

		A_c = N.transpose() * W_c * N;
		L_c = N.transpose() * W_c * C;
		K = A_c.inverse() * L_c;

		Eigen::VectorXd V_r = W_r * M * J - W_r * R;
		Eigen::VectorXd V_c = W_c * N * K - W_c * C;

		a = J.segment(0, 20);
		b.segment(1, 19) = J.segment(20, 19);
		c = K.segment(0, 20);
		d.segment(1, 19) = K.segment(20, 19);

		if (std::max(std::abs(V_r.maxCoeff()) * image_scale.x(), std::abs(V_c.maxCoeff()) * image_scale.y()) < 1) {
			std::cout << "iteration: " << iter << std::endl;
			break;
		}
	}
}

void ClassRPC::save_RPCs(const std::string& filePath)
{
	std::ofstream file(filePath);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << filePath << std::endl;
	}
	std::vector<std::string> listOff = { "LINE_OFF: ", "SAMP_OFF: ", "LAT_OFF: ", "LONG_OFF: ", "HEIGHT_OFF: " };
	std::vector<std::string>listScale = { "LINE_SCALE: ", "SAMP_SCALE: ", "LAT_SCALE: ", "LONG_SCALE: ", "HEIGHT_SCALE: " };
	std::vector<std::string> list1 = { " pixels", " pixels", " degrees", " degrees", " meters" };
	for (int i = 0; i < 2; ++i) {
		file << listOff[i] << image_centroid[i] << list1[i] << "\n";
	}
	for (int i = 0; i < 3; i++) {
		file << listOff[i + 2] << ground_centroid[i] << list1[i + 2] << "\n";
	}
	for (int i = 0; i < 2; ++i) {
		file << listScale[i] << image_scale[i] << list1[i] << "\n";
	}
	for (int i = 0; i < 3; i++) {
		file << listScale[i + 2] << ground_scale[i] << list1[i + 2] << "\n";
	}
	std::vector<std::string> listCoeff = { "LINE_NUM_COEFF_", "LINE_DEN_COEFF_", "SAMP_NUM_COEFF_", "SAMP_DEN_COEFF_" };
	for (int i = 0; i < 20; ++i) {
		file << listCoeff[0] << i + 1 << ":    " << a[i] << "\n";
	}
	for (int i = 0; i < 20; ++i) {
		file << listCoeff[1] << i + 1 << ":    " << b[i] << "\n";
	}
	for (int i = 0; i < 20; ++i) {
		file << listCoeff[2] << i + 1 << ":    " << c[i] << "\n";
	}
	for (int i = 0; i < 20; ++i) {
		file << listCoeff[3] << i + 1 << ":    " << d[i] << "\n";
	}
	file.close();
}

void ClassRPC::calculate_imgPt_from_RPCs(std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& ground_points)
{
	for (int i = 0; i < ground_points.size(); i++) {
		double X = ground_points[i].x();
		double Y = ground_points[i].y();
		double Z = ground_points[i].z();
		Eigen::VectorXd vec(20), vec2;
		vec << 1, Y, X, Z, Y* X, Y* Z, X* Z, Y* Y, X* X, Z* Z, X* Y* Z, Y* Y* Y, Y* X* X, Y* Z* Z, Y* Y* X, X* X* X, X* Z* Z, Y* Y* Z, X* X* Z, Z* Z* Z;
		//vec << 1, Z, Y, X, Z* Y, Z* X, Y* X, Z* Z, Y* Y, X* X, Z* Y* X, Z* Z* Y, Z* Z* X, Y* Y* Z, Y* Y* X, Z* X* X, Y* X* X, Z* Z* Z, Y* Y* Y, X* X* X;
		double row = vec.dot(a) / vec.dot(b);
		double col = vec.dot(c) / vec.dot(d);
		image_points.push_back(Eigen::Vector2d(row, col));
	}
}

void ClassRPC::calculate_imagePt(const Eigen::Matrix3d& R_j2w, const Eigen::Matrix3d& R_b2j, const Eigen::Vector3d& XYZ, const Eigen::Vector3d& gps, Eigen::Vector2d& image_point)
{
	Eigen::Vector3d XYZ_hat = (R_j2w * R_b2j).inverse() * (XYZ - gps);
	image_point << -XYZ_hat[0] / XYZ_hat[2], -XYZ_hat[1] / XYZ_hat[2];
}

void ClassRPC::check(const std::vector<Eigen::Vector3d>& check_points, const std::vector<Eigen::Vector3d>& gps, const std::vector<Eigen::Matrix3d>& R_j2w, const std::vector<Eigen::Matrix3d>& R_b2j, const std::vector<Eigen::Vector3d>& ux, std::vector<Eigen::Vector2d>& image_points, std::vector<Eigen::Vector3d>& ground_points)
{
	std::vector<int> check_x;
	std::vector<int> check_y;
	std::vector<int> invalid;
	int line_num = gps.size();
	for (int i = 0; i < check_points.size(); ++i) {
		Eigen::Vector3d checkPt = check_points[i];
		int Ls = 0;
		int Le = line_num - 1;
		bool iteration = true;
		while (iteration)
		{
			int L = (Ls + Le) / 2;
			Eigen::Vector2d xs, x, xe;
			calculate_imagePt(R_j2w[Ls], R_b2j[Ls], checkPt, gps[Ls], xs);
			calculate_imagePt(R_j2w[L], R_b2j[L], checkPt, gps[L], x);
			calculate_imagePt(R_j2w[Le], R_b2j[Le], checkPt, gps[Le], xe);
			if (xs[0] < 0 && x[0] < 0 && xe[0] < 0) {
				iteration = false;
				invalid.push_back(i);
			}
			else if (xs[0] > 0 && x[0] > 0 && xe[0] > 0) {
				iteration = false;
				invalid.push_back(i);
			}
			else {
				if (xs[0] * x[0] <= 0) {
					Le = L;
				}
				else if (x[0] * xe[0] <= 0) {
					Ls = L;
				}
				else {
					Ls = (Ls + L) / 2;
					Le = (Le + L) / 2;
				}
				if (std::abs(Le - Ls) <= 1) {
					iteration = false;
					check_x.push_back(L);
					check_y.push_back(std::min_element(ux.begin(), ux.end(),
						[&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
							return std::abs(x[1] - a[1]) < std::abs(x[1] - b[1]);
						}) - ux.begin());
				}
			}
		}
	}

	for (int i = 0; i < check_x.size(); ++i) {
		image_points.push_back(Eigen::Vector2d(check_x[i], check_y[i]));
	}
	for (int i = 0; i < check_points.size(); ++i) {
		if (std::find(invalid.begin(), invalid.end(), i) == invalid.end()) {
			ground_points.push_back(check_points[i]);
		}
	}
}


