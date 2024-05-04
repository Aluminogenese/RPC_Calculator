#include "exterior_elements.h"
#include <iomanip>
ExteriorElements::ExteriorElements(const std::string& attDataPath, const std::string& gpsDataPath, const std::string& imgTimePath) {
    // 存放读入的时间标签文件 RelLine Time deltaTime
    std::vector<std::tuple<int, double, double>> image_time;
    // 存放读入的gps数据文件 timeCode dateTime PX PY PZ VX VY VZ
    std::vector<std::tuple<double, std::string, double, double, double, double, double, double>> gps_data;
    // 存放读入的姿态数据文件 timeCode dateTime roll pitch yaw roll_velocity pitch_velocity yaw_velocity q1 q2 q3 q4 
    std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>> att_data;

    load_imageTime(imgTimePath, image_time);
    load_gpsData(gpsDataPath, gps_data);
    load_attData(attDataPath, att_data);
    transform_time(image_time);

    if (image_time.empty() || gps_data.empty() || att_data.empty()) {
        std::cout << "Error reading external orientation data." << std::endl;
    }
    else {
        att_interpolate(image_time, att_data);
        gps_interpolate(image_time, gps_data);
    }
}

void ExteriorElements::load_imageTime(const std::string& filePath, std::vector<std::tuple<int, double, double>>& imageTime) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
    }
    std::string line;
    std::getline(file, line);// 读取首行
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int RelLine;
        double Time, deltaTime;
        if (!(iss >> RelLine >> Time >> deltaTime)) {
            std::cerr << "Error parsing line:" << line << std::endl;
            continue;
        }
        imageTime.push_back(std::make_tuple(RelLine, Time, deltaTime));
    }
    file.close();
}

void ExteriorElements::load_gpsData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("gpsData_") != std::string::npos) {
            double timeCode, PX, PY, PZ, VX, VY, VZ;
            std::string dateTime;

            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                if (line.find("timeCode") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> timeCode;
                }
                else if (line.find("dateTime") != std::string::npos) {
                    dateTime = line.substr(line.find("=") + 2, line.size() - 3 - line.find("="));
                }
                else if (line.find("PX") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> PX;
                }
                else if (line.find("PY") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> PY;
                }
                else if (line.find("PZ") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> PZ;
                }
                else if (line.find("VX") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> VX;
                }
                else if (line.find("VY") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> VY;
                }
                else if (line.find("VZ") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> VZ;
                }
            }

            gpsData.push_back(std::make_tuple(timeCode, dateTime, PX, PY, PZ, VX, VY, VZ));
        }
    }
    file.close();
}

void ExteriorElements::load_attData(const std::string& filePath, std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("attData_") != std::string::npos) {
            double timeCode, eulor1, eulor2, eulor3, roll_velocity, pitch_velocity, yaw_velocity, q1, q2, q3, q4;
            std::string dateTime;

            while (std::getline(file, line) && line.find("}") == std::string::npos) {
                if (line.find("timeCode") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> timeCode;
                }
                else if (line.find("dateTime") != std::string::npos) {
                    dateTime = line.substr(line.find("=") + 2, line.size() - 3 - line.find("="));
                }
                else if (line.find("eulor1") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> eulor1;
                }
                else if (line.find("eulor2") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> eulor2;
                }
                else if (line.find("eulor3") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> eulor3;
                }
                else if (line.find("roll_velocity") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> roll_velocity;
                }
                else if (line.find("roll_velocity") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> pitch_velocity;
                }
                else if (line.find("roll_velocity") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> yaw_velocity;
                }
                else if (line.find("q1") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> q1;
                }
                else if (line.find("q2") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> q2;
                }
                else if (line.find("q3") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> q3;
                }
                else if (line.find("q4") != std::string::npos) {
                    std::istringstream iss(line.substr(line.find("=") + 1));
                    iss >> q4;
                }
            }
            attData.push_back(std::make_tuple(timeCode, dateTime, eulor1, eulor2, eulor3, roll_velocity, pitch_velocity, yaw_velocity, q1, q2, q3, q4));
        }
    }
    file.close();
}

void ExteriorElements::transform_time(const std::vector<std::tuple<int, double, double>>& imageTime) {
    double timeCode0= 131862356.2500000000;
    int year = 2013;
    int month = 3;
    double base_day = 7 + 4.0 / 24 + 25 / (24.0 * 60) + 56.25 / (24.0 * 60 * 60);

    for (int i = 0; i < imageTime.size(); ++i) {
        double image_time = base_day + (std::get<1>(imageTime[i]) - timeCode0) / (24.0 * 60 * 60);
        int day = static_cast<int>(image_time);

        double hoursDecimal = (image_time - day) * 24;
        int hours = static_cast<int>(hoursDecimal);

        double minutesDecimal = (hoursDecimal - hours) * 60;
        int minutes = static_cast<int>(minutesDecimal);

        double seconds = (minutesDecimal - minutes) * 60;

        transformed_time.push_back(std::make_tuple(year, month, day,hours,minutes,seconds));
    }
}

void ExteriorElements::att_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData) {
    double deltaTime = std::get<0>(attData[1]) - std::get<0>(attData[0]);
    double t_attStart = std::get<0>(attData[0]);
    for (const auto& image_time : imageTime) {
        double t = std::get<1>(image_time);
        int index = std::floor((t - t_attStart) / deltaTime);
        double t0 = std::get<0>(attData[index]);
        double t1 = std::get<0>(attData[index + 1]);
        Eigen::Vector4d q0(std::get<8>(attData[index]), std::get<9>(attData[index]), std::get<10>(attData[index]), std::get<11>(attData[index]));
        Eigen::Vector4d q1(std::get<8>(attData[index + 1]), std::get<9>(attData[index + 1]), std::get<10>(attData[index + 1]), std::get<11>(attData[index + 1]));
        //Eigen::Vector4d qt;

        Eigen::Quaterniond Q0(q0);
        Eigen::Quaterniond Q1(q1);
        Eigen::Quaterniond Qt = Q0.slerp((t - t0) / (t1 - t0), Q1);
        att.push_back(Qt.coeffs());
        //double tq = q0.dot(q1);
        ////if (std::abs(1 - std::abs(tq)) < 0.001) {
        //if (std::abs(tq) == 1) {
        //    qt = q0 * (t1 - t) / (t1 - t0) + q1 * (t - t0) / (t1 - t0);
        //}
        //else {
        //    if (tq < 0) {
        //        q0 = -q0;
        //    }
        //    double theta = acos(tq);
        //    double eta0 = sin(theta * (t1 - t) / (t1 - t0)) / sin(theta);
        //    double eta1 = sin(theta * (t - t0) / (t1 - t0)) / sin(theta);
        //    qt = eta0 * q0 + eta1 * q1;
        //}
        //att.push_back(qt);
    }
}

void ExteriorElements::gps_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData) {
    double deltaTime = std::get<0>(gpsData[1]) - std::get<0>(gpsData[0]);
    double t_gpsStart = std::get<0>(gpsData[0]);
    for (const auto& image_time : imageTime) {
        double t = std::get<1>(image_time);
        int index = std::floor((t - t_gpsStart) / deltaTime);
        Eigen::Vector3d Pt = Eigen::Vector3d::Zero();
        for (int i = 1; i < 9; i++) {
            Eigen::Vector3d Pi(std::get<2>(gpsData[index + i - 4]), std::get<3>(gpsData[index + i - 4]), std::get<4>(gpsData[index + i - 4]));
            double ti = std::get<0>(gpsData[index + i - 4]);
            for (int j = 1; j < 9; j++) {
                if (j != i) {
                    Pi *= (t - std::get<0>(gpsData[index + j - 4])) / (ti - std::get<0>(gpsData[index + j - 4]));
                }
            }
            Pt += Pi;
        }
        gps.push_back(Pt);
    }
}
