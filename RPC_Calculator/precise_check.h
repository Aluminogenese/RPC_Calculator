#pragma once
#include <iostream>
//#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_conv.h"
class PreciseCheck
{
public:
    static void transform(const double transformer[6], const Eigen::Vector2i& pixel_coor, Eigen::Vector2d& geo_coor);
    static void reverse_transform(const double transformer[6], const Eigen::Vector2d& geo_coor, Eigen::Vector2d& pixel_coor);
    static void get_check_grid(const std::string& path_range, const char* path_dem, const int& grid_m, const int& grid_n, std::vector<Eigen::Vector3d>& ground_poinds);
};