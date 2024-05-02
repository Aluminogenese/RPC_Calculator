#include "class_RPC.h"

void ClassRPC::barycentric(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points, std::vector<Eigen::Vector2d>& image_barycentric, std::vector<Eigen::Vector3d>& control_barycentric)
{
	double max_x = 0., max_y = 0., max_X = 0., max_Y = 0., max_Z = 0.;
	double min_x = image_points[0].x(), min_y = image_points[0].y(), min_X = control_points[0].x(), min_Y = control_points[0].y(), min_Z = control_points[0].z();
	for (int i = 0; i < image_points.size(); i++) {
		image_centroid += image_points[i];
		control_centroid += control_points[i];

		max_x = std::max(max_x, image_points[i].x());
		max_y = std::max(max_y, image_points[i].y());
		max_X = std::max(max_X, control_points[i].x());
		max_Y = std::max(max_Y, control_points[i].y());
		max_Z = std::max(max_Z, control_points[i].z());

		min_x = std::min(min_x, image_points[i].x());
		min_y = std::min(min_y, image_points[i].y());
		min_X = std::min(min_X, control_points[i].x());
		min_Y = std::min(min_Y, control_points[i].y());
		min_Z = std::min(min_Z, control_points[i].z());
	}
	// 标准化平移参数
	image_centroid /= image_points.size();
	control_centroid /= control_points.size();
	// 标准化比例参数
	image_scale.x() = std::max(std::abs(max_x - image_centroid.x()), std::abs(min_x - image_centroid.x()));
	image_scale.y() = std::max(std::abs(max_y - image_centroid.y()), std::abs(min_y - image_centroid.y()));

	control_scale.x() = std::max(std::abs(max_X - control_centroid.x()), std::abs(min_X - control_centroid.x()));
	control_scale.y() = std::max(std::abs(max_Y - control_centroid.y()), std::abs(min_Y - control_centroid.y()));
	control_scale.z() = std::max(std::abs(max_Z - control_centroid.z()), std::abs(min_Z - control_centroid.z()));

	for (int i = 0; i < image_points.size(); i++) {
		Eigen::Vector2d xn;
		Eigen::Vector3d Pn;
		xn = image_points[i] - image_centroid;
		Pn = control_points[i] - control_centroid;
		image_barycentric.push_back(Eigen::Vector2d(xn.x() / image_scale.x(), xn.y() / image_scale.y()));
		control_barycentric.push_back(Eigen::Vector3d(Pn.x() / control_scale.x(), Pn.y() / control_scale.y(), Pn.z() / control_scale.z()));
	}
}

void ClassRPC::calculate_RPC(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& control_points)
{
	for (int iter = 0; iter < 3; iter++) {
		int num = image_points.size();
		Eigen::VectorXd R(num), C(num);
		//Eigen::VectorXd G(num * 2);
		Eigen::MatrixXd M(num, 39), N(num, 39);
		Eigen::MatrixXd W_r = Eigen::MatrixXd::Zero(num, num), W_c = Eigen::MatrixXd::Zero(num, num);
		for (int i = 0; i < control_points.size(); i++) {
			R(i) = image_points[i].x();
			C(i) = image_points[i].y();
			//G(i) = image_points[i].x();
			//G(i + num) = image_points[i].y();
			double X = control_points[i].x();
			double Y = control_points[i].y();
			double Z = control_points[i].z();
			Eigen::VectorXd vec1(20), vec2;
			//vec1 << 1, Y, X, Z, Y* X, Y* Z, X* Z, Y* Y, X* X, Z* Z, X* Y* Z, Y* Y* Y, Y* X* X, Y* Z* Z, Y* Y* X, X* X* X, X* Z* Z, Y* Y* Z, X* X* Z, Z* Z* Z;
			vec1 << 1, Z, Y, X, Z* Y, Z* X, Y* X, Z* Z, Y* Y, X* X, Z* Y* X, Z* Z* Y, Z* Z* X, Y* Y* Z, Y* Y* X, Z* X* X, Y* X* X, Z* Z* Z, Y* Y* Y, X* X* X;
			vec2 = vec1.tail(19);

			double B = vec1.dot(b);
			M.row(i) << vec1.transpose(), -image_points[i].x() * vec2.transpose();
			W_r(i, i) = 1 / B;

			double D = vec1.dot(d);
			N.row(i) << vec1.transpose(), -image_points[i].y() * vec2.transpose();
			W_c(i, i) = 1 / D;
		}
		// 行列分别解算
		Eigen::MatrixXd A_r = M.transpose() * (W_r * W_r) * M;
		Eigen::MatrixXd L_r = M.transpose() * (W_r * W_r) * R;
		Eigen::VectorXd J = A_r.inverse() * L_r;

		Eigen::MatrixXd A_c = N.transpose() * (W_c * W_c) * N;
		Eigen::MatrixXd L_c = N.transpose() * (W_c * W_c) * C;
		Eigen::VectorXd K = A_c.inverse() * L_c;

		Eigen::VectorXd V_r = W_r * M * J - W_r * R;
		Eigen::VectorXd V_c = W_c * N * K - W_c * C;
		std::cout << std::fixed <<"J:\n"<< J.transpose() << std::endl << "K:\n" << K.transpose() << std::endl;
		std::cout << std::fixed << "V_r:\n" << V_r.transpose() << std::endl << "V_c:\n" << V_c.transpose() << std::endl;
		a = J.segment(0, 20);
		b.segment(1, 19) = J.segment(20, 19);
		c = K.segment(0, 20);
		d.segment(1, 19) = K.segment(20, 19);
		//// V=WTX-WG
		//Eigen::MatrixXd T(num * 2, 39 * 2);
		//T.block(0, 0, num, 39) = M;
		//T.block(num, 39, num, 39) = N;
		//
		//Eigen::MatrixXd W(2 * num, 2 * num);
		//W.block(0, 0, num, num) = W_r;
		//W.block(num, num, num, num) = W_c;
		//// ldlt
		////Eigen::VectorXd X = (T.transpose() * W * W * T).ldlt().solve(T.transpose() * W * W * G);
		//// QR
		////Eigen::VectorXd X = (A).householderQr().solve(Y);
		//// 法化
		////(WT)^T*(WT)
		//Eigen::MatrixXd A = T.transpose() * W * W * T;
		////(WT)^T*G
		//Eigen::MatrixXd L = T.transpose() * W * W * G;
		//
		//Eigen::VectorXd X = A.inverse() * L;
		//Eigen::VectorXd V = W * T * X - W * G;
		//// update
		//a = X.segment(0, 20);
		//b.segment(1, 19) = X.segment(20, 19);
		//c = X.segment(39, 20);
		//d.segment(1, 19) = X.segment(59, 19);

		if (std::max(std::abs(V_r.maxCoeff()),std::abs(V_c.maxCoeff())) < 0.0015) {
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
		file << listOff[i + 2] << control_centroid[i] << list1[i + 2] << "\n";
	}
	for (int i = 0; i < 2; ++i) {
		file << listScale[i] << image_scale[i] << list1[i] << "\n";
	}
	for (int i = 0; i < 3; i++) {
		file << listScale[i + 2] << control_scale[i] << list1[i + 2] << "\n";
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


