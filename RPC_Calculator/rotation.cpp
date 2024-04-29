#include "rotation.h"

Rotation::Rotation(const std::string& Rupath, const std::string directPath, const std::string& rotPath)
{
}

void Rotation::load_direct_data(const std::string& filePath, std::vector<std::tuple<int, double, double>>& directData)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
    }
    std::string line;
    std::getline(file, line);// ¶ÁÈ¡Ê×ÐÐ
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

void Rotation::calculate_Rc2b(const std::string& filePath, Eigen::Matrix3d& R_c2b)
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
    R_c2b = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void Rotation::calculate_Rb2j(const std::vector<std::tuple<double, double, double, double>>& att, std::vector<Eigen::Matrix3d>& R_b2j)
{
    for (const auto& att_item : att) {
        Eigen::Quaterniond q(std::get<3>(att_item), std::get<0>(att_item), std::get<1>(att_item), std::get<2>(att_item));
        R_b2j.push_back(Eigen::Matrix3d(q.normalized().toRotationMatrix()));
    }
}

void Rotation::get_ux(const std::vector<std::tuple<int, double, double>>& directData, std::vector<Eigen::Vector3d>& ux)
{
    for (const auto& direct_data : directData) {
        ux.push_back(Eigen::Vector3d(tan(std::get<2>(direct_data)), tan(std::get<2>(direct_data)), -1));
    }
}
