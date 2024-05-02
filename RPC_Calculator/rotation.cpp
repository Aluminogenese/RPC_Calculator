#include "rotation.h"

Rotation::Rotation(const std::string& RuPath, const std::string& directPath, const std::string& rotPath, std::vector<Eigen::Vector4d>& att)
{
    std::vector<std::tuple<int, double, double>> directData;
    // 导入指向角文件
    load_direct_data(directPath, directData);
    // 相机坐标系到本体坐标系
    calculate_Rc2b(RuPath);
    // 本体坐标系到J2000坐标系
    calculate_Rb2j(att);
    // J2000到WGS84
    calculate_Rj2w(rotPath);
    // 视向量
    get_ux(directData);

}

void Rotation::load_direct_data(const std::string& filePath, std::vector<std::tuple<int, double, double>>& directData)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
    }
    std::string line;
    std::getline(file, line);// 读取首行
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int pixelID;
        double phi_x, phi_y;
        if (!(iss >> pixelID >> phi_x >> phi_y)) {
            std::cerr << "Error parsing line:" << line << std::endl;
            continue;
        }
        directData.push_back(std::make_tuple(pixelID, phi_x, phi_y));
    }
    file.close();
}

void Rotation::calculate_Rc2b(const std::string& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }
    std::string line;
    double startTime, pitch, Vpitch, roll, Vroll, yaw, Vyaw;
    while (std::getline(file, line)) {
        if (line.find("starttime") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> startTime;
        }
        else if (line.find("pitch") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> pitch;
        }
        else if (line.find("Vpitch") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> Vpitch;
        }
        else if (line.find("roll") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> roll;
        }
        else if (line.find("Vroll") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> Vroll;
        }
        else if (line.find("yaw") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> yaw;
        }
        else if (line.find("Vyaw") != std::string::npos) {
            std::istringstream iss(line.substr(line.find("=") + 1));
            iss >> Vyaw;
        }
    }
    //Eigen::Matrix3d R_phi, R_omega, R_kappa;
    //R_phi << cos(roll), 0, sin(roll),
    //    0, 1, 0,
    //    -sin(roll), 0, cos(roll);
    //R_omega << 1, 0, 0,
    //    0, cos(pitch), -sin(pitch),
    //    0, sin(pitch), 0, cos(pitch);
    //R_kappa << cos(yaw), -sin(yaw), 0,
    //    sin(yaw), cos(yaw), 0,
    //    0, 0, 1;
    
    // 由欧拉角构造旋转矩阵
    R_c2b = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void Rotation::calculate_Rb2j(const std::vector<Eigen::Vector4d>& att)
{
    for (const auto& att_item : att) {
        // 由四元数构造旋转矩阵
        Eigen::Quaterniond q(att_item);
        Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
        R_b2j.push_back(rotation_matrix);
    }
}

void Rotation::calculate_Rj2w(const std::string& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }
    std::string line;
    double a1, a2, a3, b1, b2, b3, c1, c2, c3;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        if (!(iss >> a1 >> a2 >> a3 >> b1 >> b2 >> b3 >> c1 >> c2 >> c3)) {
            std::cerr << "Error parsing line:" << line << std::endl;
            continue;
        }
        Eigen::Matrix3d tmp_matrix;
        tmp_matrix << a1, a2, a3, b1, b2, b3, c1, c2, c3;
        R_j2w.push_back(tmp_matrix);
    }
    file.close();
}

void Rotation::get_ux(const std::vector<std::tuple<int, double, double>>& directData)
{
    for (const auto& direct_data : directData) {
        ux.push_back(Eigen::Vector3d(tan(std::get<2>(direct_data)), tan(std::get<1>(direct_data)), -1));
    }
}

bool Rotation::solve_quadratic(const double& a, const double& b, const double& c, double& x)
{
    double discriminant = b * b - 4 * a * c;
    if (discriminant > 0) {
        double x1 = (-b + sqrt(discriminant)) / (2 * a);
        double x2 = (-b - sqrt(discriminant)) / (2 * a);
        x = std::max(x1, x2);
        return true;
    }
    else if (discriminant == 0) {
        x = -b / (2 * a);
        return true;
    }
    else {
        return false;
    }
}

void Rotation::calculate_XYZ(const double& h, const Eigen::Vector3d& gps_XYZ, Eigen::Vector3d& XYZ, const int& i, const int& j)
{
    const double A = 6378137 + h;
    const double B = 6356752.3142518 + h;
    Eigen::Vector3d miu_3 = R_j2w[i] * R_b2j[i] * R_c2b * ux[j];
    double a = (miu_3[0] * miu_3[0] + miu_3[1] * miu_3[1]) / (A * A) + miu_3[2] * miu_3[2] / (B * B);
    double b = 2 * ((miu_3[0] * gps_XYZ[0] + miu_3[1] * gps_XYZ[1]) / (A * A) + miu_3[2] * gps_XYZ[2] / (B * B));
    double c = (gps_XYZ[0] * gps_XYZ[0] + gps_XYZ[1] * gps_XYZ[1]) / (A * A) + gps_XYZ[2] * gps_XYZ[2] / (B * B) - 1;
    double x;
    if (solve_quadratic(a, b, c, x)) {
        XYZ = gps_XYZ + x * miu_3;
    }
    else {
        XYZ = gps_XYZ;
    }
}

void Rotation::convert_wgs84_to_geodetic(const std::vector<Eigen::Vector3d>& XYZ, std::vector<Eigen::Vector3d>& BLH)
{
    double epsilon = 1.0E-15;
    double a = 6378137.0; // 长半轴（赤道半径）
    double f_inverse = 298.257223563; // 扁率倒数
    double b = a - a / f_inverse; // 短半轴（极半径）
    double e = sqrt(a * a - b * b) / a;

    for (const auto& xyz : XYZ) {
        double x = xyz.x();
        double y = xyz.y();
        double z = xyz.z();

        double current_B = 0.;
        double N = 0.;
        double calculate_B = atan2(z, sqrt(x * x + y * y));
        int count = 0;
        while (abs(current_B - calculate_B) * 180 / M_PI > epsilon && count < 25) {
            current_B = calculate_B;
            N = a / sqrt(1 - e * e * sin(current_B) * sin(current_B));
            calculate_B = atan2(z + N * e * e * sin(current_B), sqrt(x * x + y * y));
            count++;
        }
        double B = current_B * 180 / M_PI;
        double L = atan2(y, x) * 180 / M_PI;
        double H = z / sin(current_B) - N * (1 - e * e);
        BLH.push_back(Eigen::Vector3d(B, L, H));
    }
}

void Rotation::convert_geodetic_to_wgs84(const std::vector<Eigen::Vector3d>& BLH, std::vector<Eigen::Vector3d>& XYZ)
{
    double a = 6378137.0; // 长半轴（赤道半径）
    const double f_inverse = 298.257223563;	//扁率倒数
    const double b = a - a / f_inverse; // 短半轴（极半径）

    double e = sqrt(a * a - b * b) / a;
    for (const auto& blh : BLH) {
        double latitude = blh.x();
        double longitude = blh.y();
        double altitude = blh.z();
        double N = a / sqrt(1 - e * e * pow(sin(latitude), 2));
        Eigen::Vector3d xyz;
        xyz.x() = (N + altitude) * cos(latitude) * cos(longitude);
        xyz.y() = (N + altitude) * cos(latitude) * sin(longitude);
        xyz.z() = (N * (1 - e * e) + altitude) * sin(latitude);
        XYZ.push_back(xyz);
    }
}

