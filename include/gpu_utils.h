#ifndef GPU_UTILS_H
#define GPU_UTILS_H

#include <stdio.h>
#include <math.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_device_runtime_api.h>
#include <device_launch_parameters.h>       //Header for threadInd.x

#define CLOUD_WIDTH             ((int)640)
#define CLOUD_HEIGHT            ((int)480)
#define ANGLE_INCREMENT_DEG     ((float)0.25)
#define ANGLE_INCREMENT_RAD     ((float) ANGLE_INCREMENT_DEG * M_PI / 180)
#define HORIZONTAL_VIEW_DEG     ((float) 120)
#define HORIZONTAL_VIEW_RAD     ((float) HORIZONTAL_VIEW_DEG * M_PI / 180)
#define LASER_SCANNER_POINTS    ((float) HORIZONTAL_VIEW_DEG / ANGLE_INCREMENT_DEG + 1)


class GPUUtils
{
public:

    GPUUtils(bool sensor_1_, bool sensor_2_);
    ~GPUUtils();

    void computeLaserScan(const unsigned char *input_cloud, float *transformation,
                          float tolerance, float robot_height, float* laser_scan);

    void computeLaserScan(const unsigned char *input_cloud_1, float *transformation_1,
                          const unsigned char *input_cloud_2, float *transformation_2,
                          float tolerance, float robot_height, float* laser_scan);

    void extractCloudFloor(const unsigned char *input_cloud, float *transformation, float floor_tolerance,
                           unsigned char **output_cloud, int &cloud_floor_points);

    void extractCloudObstacle(const unsigned char *input_cloud, float *transformation, float min_height, float max_height,
                              unsigned char **output_cloud, int &cloud_obstacle_points);

private:
    bool sensor_1, sensor_2;
    float *d_input_cloud_1;
    float *d_input_cloud_2;
    float *d_transformation_1;
    float *d_transformation_2;
    float *d_laser_scan;

    dim3 dim_block;
    dim3 dim_grid;
};

#endif //GPU_UTILS_H
