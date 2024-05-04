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
     * @brief ��������ת��������
     *
     * @param transformer ת������
     * @param pixel_coor ��������
     * @param geo_coor ת���õ��ĵ�������
     */
    static void pixelToGeo(const double* transformer, const Eigen::Vector2i& pixel_coor, Eigen::Vector2d& geo_coor);

    /**
     * @brief ��������ת��������
     *
     * @param transformer ת������
     * @param geo_coor ��������
     * @param pixel_coor ת���õ�����������
     */
    static void geoToPixel(double* transformer, const Eigen::Vector2d& geo_coor, Eigen::Vector2d& pixel_coor);

    /**
     * @brief ��ȡ��������
     *
     * @param path_range ������Χ�ļ�·��
     * @param path_dem DEM�ļ�·��
     * @param m ������
     * @param n ������
     * @param ground_poinds �õ����﷽������
     */
    static void get_check_grid(const std::string& path_range, const char* path_dem, const int& grid_m, const int& grid_n, std::vector<Eigen::Vector3d>& ground_poinds);
};