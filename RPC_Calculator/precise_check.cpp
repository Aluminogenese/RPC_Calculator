#include "precise_check.h"

void PreciseCheck::transform(const double transformer[6], const Eigen::Vector2i& pixel_coor, Eigen::Vector2d& geo_coor)
{
	geo_coor(0) = transformer[0] + pixel_coor(0) * transformer[1] + pixel_coor(1) * transformer[2];
	geo_coor(1) = transformer[3] + pixel_coor(0) * transformer[4] + pixel_coor(1) * transformer[5];
}

void PreciseCheck::reverse_transform(const double transformer[6], const Eigen::Vector2d& geo_coor, Eigen::Vector2d& pixel_coor)
{
    double det = transformer[1] * transformer[5] - transformer[2] * transformer[4];
    if (det == 0) {
        std::cerr << "Matrix determinant is zero." << std::endl;
        pixel_coor << 0, 0;
    }
    double inv_det = 1.0 / det;
    double w = (geo_coor.x() - transformer[0]) * inv_det * transformer[5] - (geo_coor.y() - transformer[3]) * inv_det * transformer[2];
    double h = (geo_coor.y() - transformer[3]) * inv_det * transformer[1] - (geo_coor.x() - transformer[0]) * inv_det * transformer[4];
    pixel_coor << w, h;
}

void PreciseCheck::get_check_grid(const std::string& path_range, const char* path_dem, const int& grid_m, const int& grid_n, std::vector<Eigen::Vector3d>& ground_poinds)
{
    GDALAllRegister();
    GDALDataset* f = (GDALDataset*)GDALOpen(path_dem, GA_ReadOnly);
    if (f == NULL) {
        std::cerr << "Failed to open dataset." << std::endl;
        exit(1);
    }

    int im_width = f->GetRasterXSize();
    int im_height = f->GetRasterYSize();
    GDALRasterBand* band = f->GetRasterBand(1);
    double* im_data = new double[im_width * im_height];
    band->RasterIO(GF_Read, 0, 0, im_width, im_height, im_data, im_width, im_height, GDT_Float64, 0, 0);

    double transformer[6];
    f->GetGeoTransform(transformer);

    double y_max, y_min, x_max, x_min;
    std::ifstream file(path_range);
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> y_max >> y_min >> x_max >> x_min;
    file.close();

    y_max -= 0.05;
    y_min += 0.05;
    x_max -= 0.05;
    x_min += 0.05;
    Eigen::Vector2d coverage, step;
    reverse_transform(transformer, Eigen::Vector2d(x_max, y_min), coverage);
    reverse_transform(transformer, Eigen::Vector2d(x_min, y_max), step);

    double w_max = coverage.x();
    double h_max = coverage.y();
    double w_min = step.x();
    double h_min = step.y();

    // Equally divide grid points
    double dx = w_min;
    double dy = h_min;
    double dw = w_max - dx;
    double dh = h_max - dy;

    for (int j = 0; j <= grid_n; ++j) {
        for (int i = 0; i <= grid_m; ++i) {
            int pt_x = static_cast<int>((j * dh) / grid_n + dy);
            int pt_y = static_cast<int>((i * dw) / grid_m + dx);
            Eigen::Vector2d geo_coor;
            transform(transformer, Eigen::Vector2i(pt_x, pt_y), geo_coor);
            double hh = im_data[pt_x * im_width + pt_y];
            ground_poinds.push_back(Eigen::Vector3d(geo_coor.y(), geo_coor.x(), hh));
        }
    }

    delete[] im_data;
    GDALClose(f);
}
