#include <cuda_runtime.h>
#include <cstdio>
    
#define POINT_RADIUS 3

namespace miivii_image_projection_based_fusion
{
__device__ unsigned int mapIntensityToColor(float intensity, float minIntensity = 0.0f, float maxIntensity = 255.0f) {
    // 归一化强度值到[0, 1]区间
    float normalizedIntensity = fmaxf(0.0f, fminf(1.0f, (intensity - minIntensity) / (maxIntensity - minIntensity)));

    // 将归一化强度值映射到HSV颜色空间
    float hue = normalizedIntensity * 180.0f; // 色调从0（红色）到180（绿色)

    // 饱和度和亮度固定为1（最饱和和最亮）
    float saturation = 1.0f;
    float value = 1.0f;

    // 将HSV颜色转换为RGB颜色
    float r, g, b;
    int i = (int)(hue / 60.0f);
    float f = hue / 60.0f - i;
    float p = value * (1.0f - saturation);
    float q = value * (1.0f - f * saturation);
    float t = value * (1.0f - (1.0f - f) * saturation);

    switch (i % 6) {
        case 0: r = value, g = t, b = p; break;
        case 1: r = q, g = value, b = p; break;
        case 2: r = p, g = value, b = t; break;
        case 3: r = p, g = q, b = value; break;
        case 4: r = t, g = p, b = value; break;
        case 5: r = value, g = p, b = q; break;
    }

    // 将RGB颜色转换为BGR颜色
    unsigned int bgr = (static_cast<unsigned char>(b * 255) << 16) |
                       (static_cast<unsigned char>(g * 255) << 8)  |
                       static_cast<unsigned char>(r * 255);

    return bgr;
}


__global__ void projectPointsKernel(
    float *pointcloud_data,         // 展开的点云数据 (x, y, z, intensity)
    int pointcloud_size,
    float *camera_projection_matrix, // 相机投影矩阵作为平面数组
    float *lidar2cam_affine,         // LiDAR到相机的仿射变换矩阵
    unsigned char *d_image,
    int width, int height          // 输出图像的尺寸
) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pointcloud_size) return;  // 如果索引超出了点云的大小，退出

    // 获取点云数据：每个点由 (x, y, z, intensity) 组成
    float x = pointcloud_data[idx * 4];
    float y = pointcloud_data[idx * 4 + 1];
    float z = pointcloud_data[idx * 4 + 2];
    // float intensity = pointcloud_data[idx * 4 + 3];

    // 将点从LiDAR坐标系转换到相机坐标系
    float lidar_point[4] = {x, y, z, 1.0f};  // 4x1向量 (x, y, z, 1)
    float cam_point[4] = {0.0f};  // 初始化相机坐标系中的点

    // 使用仿射变换矩阵进行坐标转换
    for (int i = 0; i < 4; i++) {
        cam_point[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            cam_point[i] += lidar2cam_affine[i * 4 + j] * lidar_point[j];
        }
    }

    //打印lidar_point
    // printf("lidar_point: [%.2f, %.2f, %.2f, %.2f]\n", lidar_point[0], lidar_point[1], lidar_point[2], lidar_point[3]);

    //打印cam_point
    // printf("cam_point: [%.2f, %.2f, %.2f, %.2f]\n", cam_point[0], cam_point[1], cam_point[2], cam_point[3]);

    // 获取转换后的相机坐标
    float cam_x = cam_point[0];
    float cam_y = cam_point[1];
    float cam_z = cam_point[2];

    // 如果转换后的z值小于零，表示该点位于相机前面，投影无意义，跳过
    if (cam_z <= 0) return;

    // 利用相机投影矩阵将3D点投影到2D图像平面
    float proj_x = camera_projection_matrix[0] * cam_x + camera_projection_matrix[1] * cam_y + camera_projection_matrix[2] * cam_z;
    float proj_y = camera_projection_matrix[3] * cam_x + camera_projection_matrix[4] * cam_y + camera_projection_matrix[5] * cam_z;
    float proj_w = camera_projection_matrix[6] * cam_x + camera_projection_matrix[7] * cam_y + camera_projection_matrix[8] * cam_z;

    // 归一化处理，得到投影后的图像坐标 (u, v)
    if (proj_w != 0) {
        proj_x /= proj_w;
        proj_y /= proj_w;
    }
    
    int u_center = roundf(proj_x);
    int v_center = roundf(proj_y);
    if (u_center < 0 || u_center >= width || v_center < 0 || v_center >= height) return;

    // 计算颜色
    // unsigned int color = mapIntensityToColor(intensity);
    // unsigned char b = (color >> 16) & 0xFF;
    // unsigned char g = (color >> 8) & 0xFF;
    // unsigned char r = color & 0xFF;

    // 绘制大一点的点：在中心点周围绘制一个小圆盘
    for (int dy = -POINT_RADIUS; dy <= POINT_RADIUS; ++dy) {
        for (int dx = -POINT_RADIUS; dx <= POINT_RADIUS; ++dx) {
            int u = u_center + dx;
            int v = v_center + dy;

            // 检查是否超出图像边界
            if (u >= 0 && u < width && v >= 0 && v < height &&
                // 并且确保当前点位于以(u_center, v_center)为中心、POINT_RADIUS为半径的圆内
                dx * dx + dy * dy <= POINT_RADIUS * POINT_RADIUS)
            {
                // 设置像素颜色
                d_image[(v * width + u) * 3]     = 0; // B通道
                d_image[(v * width + u) * 3 + 1] = 0; // G通道
                d_image[(v * width + u) * 3 + 2] = 255; // R通道

            }
        }
    }

}


extern "C" void projectPointsOnGPU(
    float *pointcloud_data_host,          // Flattened point cloud data (x, y, z, intensity)
    int pointcloud_size,                  // Size of point cloud
    float *camera_projection_matrix_host, // Camera projection matrix (3x3)
    float *lidar2cam_affine_host,         // LiDAR to camera affine matrix (4x4)
    unsigned char *host_image,              // Image
    int width, int height               // Image width and height
    )
{
    float *d_pointcloud_data, *d_camera_projection_matrix, *d_lidar2cam_affine;
    unsigned char *d_image;

    //打印lidar2cam_affine_host和camera_projection_matrix_host
    // printf("lidar2cam_affine_host:\n");
    // for (int i = 0; i < 16; i++) {
    //     printf("%f ", lidar2cam_affine_host[i]);
    //     if ((i + 1) % 4 == 0) printf("\n");
    // }
    // printf("\n");

    // printf("camera_projection_matrix_host:\n");
    // for (int i = 0; i < 9; i++) {
    //     printf("%f ", camera_projection_matrix_host[i]);
    //     if ((i + 1) % 3 == 0) printf("\n");
    // }

    cudaMalloc((void **)&d_pointcloud_data,  pointcloud_size * 4 * sizeof(float));
    cudaMalloc((void **)&d_camera_projection_matrix,  9 * sizeof(float));
    cudaMalloc((void **)&d_lidar2cam_affine,  16 * sizeof(float));
    cudaMalloc(&d_image, width * height * 3 * sizeof(unsigned char));

    cudaMemcpy(d_pointcloud_data, pointcloud_data_host,  pointcloud_size * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_camera_projection_matrix, camera_projection_matrix_host, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_lidar2cam_affine, lidar2cam_affine_host, 16 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_image, host_image, width * height * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);

    int blockSize = 256;  // 每个线程块的大小
    int numBlocks = (pointcloud_size + blockSize - 1) / blockSize; // 计算块的数量

    projectPointsKernel<<<numBlocks, blockSize>>>(
        d_pointcloud_data, 
        pointcloud_size,
        d_camera_projection_matrix, 
        d_lidar2cam_affine,
        d_image,  
        width, 
        height
    );

    // 获取结果
    cudaMemcpy(host_image, d_image, width * height * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();

    cudaFree(d_pointcloud_data);
    cudaFree(d_camera_projection_matrix);
    cudaFree(d_lidar2cam_affine);
}

}