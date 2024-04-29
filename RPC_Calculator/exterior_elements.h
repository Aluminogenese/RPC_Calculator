#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <eigen3/Eigen/Dense>

class ExteriorElements
{
public:
    ExteriorElements(const std::string& attDataPath, const std::string& gpsDataPath, const std::string& imgTimePath);

    void load_imageTime(const std::string& fileName, std::vector<std::tuple<int, double, double>>& imageTime);

    void load_gpsData(const std::string& fileName, std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData);

    void load_attData(const std::string& fileName, std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData);

    void transform_time(const std::vector<std::tuple<int, double, double>>& imageTime, std::vector<double>& transformedTime);

    void att_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>>& attData, std::vector<std::tuple<double, double, double, double>>& att);

    void gps_interpolate(const std::vector<std::tuple<int, double, double>>& imageTime, const std::vector<std::tuple<double, std::string, double, double, double, double, double, double>>& gpsData, std::vector<std::tuple<double, double, double, double, double, double>>& gps);

    std::vector<std::tuple<int, double, double>> image_time;
    std::vector<std::tuple<double, std::string, double, double, double, double, double, double>> gps_data;
    std::vector<std::tuple<double, std::string, double, double, double, double, double, double, double, double, double, double>> att_data;

    std::vector<double> transformed_time;
    std::vector<std::tuple<double,double,double,double>> att;
    std::vector<std::tuple<double, double, double, double, double, double>> gps;
};

