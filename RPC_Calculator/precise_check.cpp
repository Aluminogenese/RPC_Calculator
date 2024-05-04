#include "precise_check.h"

void PreciseCheck::pixelToGeo(const double* transformer, const Eigen::Vector2i& pixel_coor, Eigen::Vector2d& geo_coor)
{
	geo_coor(0) = transformer[0] + pixel_coor(0) * transformer[1] + pixel_coor(1) * transformer[2];
	geo_coor(1) = transformer[3] + pixel_coor(0) * transformer[4] + pixel_coor(1) * transformer[5];
}

void PreciseCheck::geoToPixel(double* transformer, const Eigen::Vector2d& geo_coor, Eigen::Vector2d& pixel_coor)
{
    pixel_coor.x() = (geo_coor.x() - transformer[0]) / transformer[1];
    pixel_coor.y() = (geo_coor.y() - transformer[3]) / transformer[5];

    //double invTransformer[6];
    //GDALInvGeoTransform(transformer, invTransformer);
    //double w = invTransformer[0] + invTransformer[1] * geo_coor.x() + invTransformer[2] * geo_coor.y();
    //double h = invTransformer[3] + invTransformer[4] * geo_coor.x() + invTransformer[5] * geo_coor.y();
    //pixel_coor << w, h;
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

    double transformer[6], invtransformer[6];
    f->GetGeoTransform(transformer);
    GDALInvGeoTransform(transformer, invtransformer);
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
    Eigen::Vector2d right_corner, left_corner;
    geoToPixel(transformer, Eigen::Vector2d(x_max, y_min), right_corner);
    geoToPixel(transformer, Eigen::Vector2d(x_min, y_max), left_corner);

    double w_max = right_corner.x();
    double h_max = right_corner.y();
    double w_min = left_corner.x();
    double h_min = left_corner.y();

    double dw = w_max - w_min;
    double dh = h_max - h_min;

    for (int j = 0; j <= grid_n; ++j) {
        for (int i = 0; i <= grid_m; ++i) {
            int pt_x = static_cast<int>((i * dh) / grid_n + w_min);
            int pt_y = static_cast<int>((j * dw) / grid_m + h_min);
            Eigen::Vector2d geo_coor;
            pixelToGeo(transformer, Eigen::Vector2i(pt_x, pt_y), geo_coor);//pt_x:col,pt_y:row
            double hh = im_data[pt_x * im_width + pt_y];
            ground_poinds.push_back(Eigen::Vector3d(geo_coor.y(), geo_coor.x(), hh));
        }
    }

    delete[] im_data;
    GDALClose(f);
}
