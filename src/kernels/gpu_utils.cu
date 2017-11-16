#include <gpu_utils.h>

// =============================================================================
// =============================== MAIN UTILS ==================================
// =============================================================================

static __device__ float atomicMin(float* address, float val)
{
    int* address_as_i = (int*) address;
    int old = *address_as_i, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_i, assumed,
            __float_as_int(fminf(val, __int_as_float(assumed))));
    } while (assumed != old);
    return __int_as_float(old);
}

__global__ void gpuComputeLaserScan(float *d_input_cloud, float *d_transform,
                                    float tolerance, float robot_height,
                                    float *laser_scan)
{
    //Calculate indicies
    int index_x = blockIdx.x*blockDim.x + threadIdx.x;
    int index_y = blockIdx.y*blockDim.y + threadIdx.y;
    int index = index_y*blockDim.x*gridDim.x+ index_x;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Shared memory for performance reasons
    __shared__ float sh_TransfMatrix[16];

    // Each thread from specific warp writes to shared matrix
    if(threadIdx.y == 0 && threadIdx.x<16)
        sh_TransfMatrix[threadIdx.x] = d_transform[threadIdx.x];
    __syncthreads();

    // Check if number if not a nan
    if (d_input_cloud[index*4]==d_input_cloud[index*4])
    {

        //rotation P = R x P;
        x = sh_TransfMatrix[0]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[1]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[2]*d_input_cloud[index*4 + 2];

        y = sh_TransfMatrix[4]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[5]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[6]*d_input_cloud[index*4 + 2];

        z = sh_TransfMatrix[8]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[9]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[10]*d_input_cloud[index*4 + 2];

        //translation P = P + t;
        x += sh_TransfMatrix[3];
        y += sh_TransfMatrix[7];
        z += sh_TransfMatrix[11];
    }

    // Check distance
//    if (z<-tolerance || (z>tolerance && z<robot_height))
    if (z>tolerance && z<robot_height)
    {
        int laser_scan_index = (int)((HORIZONTAL_VIEW_RAD/2 + atan2(y, x))/ANGLE_INCREMENT_RAD + 0.5*ANGLE_INCREMENT_RAD);
        atomicMin(&laser_scan[laser_scan_index], sqrt(x*x + y*y));
    }
}

// =============================================================================
// =============================== HELP UTILS ==================================
// =============================================================================

__global__ void gpuExtractCloudFloor(float *d_input_cloud, float *d_transform, float tolerance,
                                     float *d_output_cloud, int *d_floor_points_number)
{
    //Calculate indicies
    int index_x = blockIdx.x*blockDim.x + threadIdx.x;
    int index_y = blockIdx.y*blockDim.y + threadIdx.y;
    int index = index_y*blockDim.x*gridDim.x+ index_x;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Shared memory for performance reasons
    __shared__ float sh_TransfMatrix[16];

    // Each thread from specific warp writes to shared matrix
    if(threadIdx.y == 0 && threadIdx.x<16)
        sh_TransfMatrix[threadIdx.x] = d_transform[threadIdx.x];
    __syncthreads();

    // Check if number if not a nan
    if (d_input_cloud[index*4]==d_input_cloud[index*4])
    {

        //rotation P = R x P;
        x = sh_TransfMatrix[0]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[1]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[2]*d_input_cloud[index*4 + 2];

        y = sh_TransfMatrix[4]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[5]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[6]*d_input_cloud[index*4 + 2];

        z = sh_TransfMatrix[8]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[9]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[10]*d_input_cloud[index*4 + 2];

        //translation P = P + t;
        x += sh_TransfMatrix[3];
        y += sh_TransfMatrix[7];
        z += sh_TransfMatrix[11];
    }

    // Check distance
    if (abs(z)<tolerance)
    {
        int where_to_add = atomicAdd(d_floor_points_number,1);
        d_output_cloud[where_to_add*4 + 0] = x;
        d_output_cloud[where_to_add*4 + 1] = y;
        d_output_cloud[where_to_add*4 + 2] = z;
        d_output_cloud[where_to_add*4 + 3] = 0;
    }
}

__global__ void gpuExtractCloudObstacle(float *d_input_cloud, float *d_transform,
                                        float min_height, float max_height,
                                        float *d_output_cloud, int *d_obstacle_points_number)
{
    //Calculate indicies
    int index_x = blockIdx.x*blockDim.x + threadIdx.x;
    int index_y = blockIdx.y*blockDim.y + threadIdx.y;
    int index = index_y*blockDim.x*gridDim.x+ index_x;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Shared memory for performance reasons
    __shared__ float sh_TransfMatrix[16];

    // Each thread from specific warp writes to shared matrix
    if(threadIdx.y == 0 && threadIdx.x<16)
        sh_TransfMatrix[threadIdx.x] = d_transform[threadIdx.x];
    __syncthreads();

    // Check if number if not a nan
    if (d_input_cloud[index*4]==d_input_cloud[index*4])
    {

        //rotation P = R x P;
        x = sh_TransfMatrix[0]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[1]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[2]*d_input_cloud[index*4 + 2];

        y = sh_TransfMatrix[4]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[5]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[6]*d_input_cloud[index*4 + 2];

        z = sh_TransfMatrix[8]*d_input_cloud[index*4 + 0] +
            sh_TransfMatrix[9]*d_input_cloud[index*4 + 1] +
            sh_TransfMatrix[10]*d_input_cloud[index*4 + 2];

        //translation P = P + t;
        x += sh_TransfMatrix[3];
        y += sh_TransfMatrix[7];
        z += sh_TransfMatrix[11];
    }

    // Check distance
    if (z>min_height && z<max_height)
    {
        int where_to_add = atomicAdd(d_obstacle_points_number,1);
        d_output_cloud[where_to_add*4 + 0] = x;
        d_output_cloud[where_to_add*4 + 1] = y;
        d_output_cloud[where_to_add*4 + 2] = z;
        d_output_cloud[where_to_add*4 + 3] = 0;
    }
}

// =============================================================================
// =============================== CON & DES ===================================
// =============================================================================

GPUUtils::GPUUtils(bool sensor_1_, bool sensor_2_)
{
    // Alocate gpu memory
    sensor_1 = sensor_1_;
    sensor_2 = sensor_2_;
    if (sensor_1)
    {
        cudaMalloc((void**)&d_input_cloud_1, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float));
        cudaMalloc((void**)&d_transformation_1, 16*sizeof(float));
    }
    if (sensor_2)
    {
        cudaMalloc((void**)&d_input_cloud_2, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float));
        cudaMalloc((void**)&d_transformation_2, 16*sizeof(float));
    }
    cudaMalloc((void**)&d_laser_scan, LASER_SCANNER_POINTS*sizeof(float));

    // Kernel invocation params
    dim_block = dim3(32,32,1);                             // 32x32 = 1024
    dim_grid  = dim3(CLOUD_WIDTH/32,CLOUD_HEIGHT/32,1);    // 620/32 = 20, 480/32=15
}
GPUUtils::~GPUUtils()
{
    // Dealocate gpu memory
    if (sensor_1)
    {
        cudaFree((void*)d_input_cloud_1);
        cudaFree((void*)d_transformation_1);
    }
    if (sensor_2)
    {
        cudaFree((void*)d_input_cloud_2);
        cudaFree((void*)d_transformation_2);
    }
    cudaFree((void*)d_laser_scan);
}

// =============================================================================
// ============================== LASER SCAN ===================================
// =============================================================================

void GPUUtils::computeLaserScan(const unsigned char *input_cloud, float *transformation,
                                float tolerance, float robot_height, float* laser_scan)
{
    // Copy memory to GPU
    cudaMemcpy((void*)d_input_cloud_1, (void*)input_cloud, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_transformation_1, transformation, 16*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_laser_scan, (void*)laser_scan, LASER_SCANNER_POINTS*sizeof(float), cudaMemcpyHostToDevice);

    // Kernel
    gpuComputeLaserScan<<<dim_grid,dim_block>>>(d_input_cloud_1, d_transformation_1,
                                                tolerance, robot_height,
                                                d_laser_scan);

    // Copy memory back to CPU
    cudaMemcpy((void*)laser_scan, (void*)d_laser_scan, LASER_SCANNER_POINTS*sizeof(float), cudaMemcpyDeviceToHost);
}

void GPUUtils::computeLaserScan(const unsigned char *input_cloud_1, float *transformation_1,
                                const unsigned char *input_cloud_2, float *transformation_2,
                                float tolerance, float robot_height, float* laser_scan)
{
    // Time measurement
//    float a,b,c;
//    cudaEvent_t time_start, time_memcpyH2D, time_kernel, time_memcpyD2H;
//    cudaEventCreate(&time_start);
//    cudaEventCreate(&time_memcpyH2D);
//    cudaEventCreate(&time_kernel);
//    cudaEventCreate(&time_memcpyD2H);
//    cudaEventRecord(time_start,0);

    // Copu memory to GPU
    cudaMemcpy((void*)d_input_cloud_1, (void*)input_cloud_1, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_input_cloud_2, (void*)input_cloud_2, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_transformation_1, transformation_1, 16*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_transformation_2, transformation_2, 16*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_laser_scan, (void*)laser_scan, LASER_SCANNER_POINTS*sizeof(float), cudaMemcpyHostToDevice);
//    cudaEventRecord(time_memcpyH2D,0);

    // Kernel
    gpuComputeLaserScan<<<dim_grid,dim_block>>>(d_input_cloud_1, d_transformation_1,
                                                tolerance, robot_height,
                                                d_laser_scan);
    gpuComputeLaserScan<<<dim_grid,dim_block>>>(d_input_cloud_2, d_transformation_2,
                                                tolerance, robot_height,
                                                d_laser_scan);
//    cudaEventRecord(time_kernel,0);

    // Copy back and free
    cudaMemcpy((void*)laser_scan, (void*)d_laser_scan, LASER_SCANNER_POINTS*sizeof(float), cudaMemcpyDeviceToHost);

    //Time
//    cudaEventRecord(time_memcpyD2H,0);
//    cudaEventSynchronize(time_memcpyD2H);
//    cudaEventElapsedTime(&a, time_start,time_memcpyH2D);
//    cudaEventElapsedTime(&b, time_memcpyH2D,time_kernel);
//    cudaEventElapsedTime(&c, time_kernel,time_memcpyD2H);

//    printf("MemoryH2D                 =%f\n", a);
//    printf("Kernel                    =%f\n", b);
//    printf("MemoryD2H                 =%f\n", c);
}

// =============================================================================
// ================================= DEBUG =====================================
// =============================================================================

void GPUUtils::extractCloudFloor(const unsigned char *input_cloud, float *transformation, float floor_tolerance,
                                 unsigned char **output_cloud, int &cloud_floor_points)
{
    // Alocate gpu memory and copy
    float *d_output_cloud;
    int *d_floor_points_number;
    cudaMalloc((void**)&d_output_cloud, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float));
    cudaMalloc((void**)&d_floor_points_number, sizeof(int));
    cudaMemcpy((void*)d_input_cloud_1, (void*)input_cloud, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_transformation_1, transformation, 16*sizeof(float), cudaMemcpyHostToDevice);

    // Pre-initialize memory for grid cells
    cudaMemset((void*)d_floor_points_number, 0, sizeof(int));
    cudaMemset((void*)d_output_cloud, 0, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(int));

    // GPU kernel
    gpuExtractCloudFloor<<<dim_grid,dim_block>>>(d_input_cloud_1, d_transformation_1,
                                                 floor_tolerance, d_output_cloud,
                                                 d_floor_points_number);

    // Copy interesing data
    cudaMemcpy((void*)&cloud_floor_points,(void*)d_floor_points_number,sizeof(int),cudaMemcpyDeviceToHost);
    *output_cloud = new unsigned char[16*cloud_floor_points];
    cudaMemcpy((void*)*output_cloud, (void*)d_output_cloud, 4*cloud_floor_points*sizeof(float), cudaMemcpyDeviceToHost);

    // Free the gpu resources
    cudaFree((void*)d_output_cloud);
    cudaFree((void*)d_floor_points_number);
}

void GPUUtils::extractCloudObstacle(const unsigned char *input_cloud, float *transformation, float min_height, float max_height,
                                   unsigned char **output_cloud, int &cloud_obstacle_points)
{
    // Alocate gpu memory and copy
    float *d_output_cloud;
    int *d_obstacle_points_number;
    cudaMalloc((void**)&d_output_cloud, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float));
    cudaMalloc((void**)&d_obstacle_points_number, sizeof(int));
    cudaMemcpy((void*)d_input_cloud_1, (void*)input_cloud, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)d_transformation_1, transformation, 16*sizeof(float), cudaMemcpyHostToDevice);

    // Pre-initialize memory for grid cells
    cudaMemset((void*)d_obstacle_points_number, 0, sizeof(int));
    cudaMemset((void*)d_output_cloud, 0, CLOUD_WIDTH*CLOUD_HEIGHT*4*sizeof(int));

    // GPU kernel
    gpuExtractCloudObstacle<<<dim_grid,dim_block>>>(d_input_cloud_1, d_transformation_1,
                                                    min_height, max_height, d_output_cloud,
                                                    d_obstacle_points_number);

    // Copy interesing data
    cudaMemcpy((void*)&cloud_obstacle_points, (void*)d_obstacle_points_number, sizeof(int),cudaMemcpyDeviceToHost);
    *output_cloud = new unsigned char[16*cloud_obstacle_points];
    cudaMemcpy((void*)*output_cloud, (void*)d_output_cloud, 4*cloud_obstacle_points*sizeof(float), cudaMemcpyDeviceToHost);

    // Free the gpu resources
    cudaFree((void*)d_output_cloud);
    cudaFree((void*)d_obstacle_points_number);
}

