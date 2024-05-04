#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_conv.h"
class PreciseCheck
{
public:
	/**
     * @brief 像素坐标转地理坐标
     *
     * @param transformer 转换矩阵
     * @param pixel_coor 像素坐标
     * @param geo_coor 转换得到的地理坐标
     */
    static void pixelToGeo(const double* transformer, const Eigen::Vector2i& pixel_coor, Eigen::Vector2d& geo_coor);

    /**
     * @brief 地理坐标转像素坐标
     *
     * @param transformer 转换矩阵
     * @param geo_coor 地理坐标
     * @param pixel_coor 转换得到的像素坐标
     */
    static void geoToPixel(double* transformer, const Eigen::Vector2d& geo_coor, Eigen::Vector2d& pixel_coor);

    /**
     * @brief 获取检查格网点
     *
     * @param path_range 格网范围文件路径
     * @param path_dem DEM文件路径
     * @param m 格网宽
     * @param n 格网高
     * @param ground_poinds 得到的物方格网点
     */
    static void get_check_grid(const std::string& path_range, const char* path_dem, const int& grid_m, const int& grid_n, std::vector<Eigen::Vector3d>& ground_poinds);
};