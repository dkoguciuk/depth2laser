// std
#include <string>
#include <numeric>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/**
 * @brief PointCloudXYZ     Pointcloud typedef.
 */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

/**
 * @brief processCloud      Find height of a sensor above the floor.
 * @param cloud             Cloud from the sensor.
 */
void processCloud(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    // Loops
    static int loop_ctr = 0;
    std::vector<float> Z;

    // Vars
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr viewer_inliers = pcl::PointIndicesPtr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr viewer_coeff = pcl::ModelCoefficientsPtr(new pcl::ModelCoefficients);

    // Convert cloud to PCL
    PointCloudXYZ::Ptr cloud_pcl(new PointCloudXYZ);
    pcl::PCLPointCloud2 cloud_ros;
    pcl_conversions::toPCL(*cloud, cloud_ros);
    pcl::fromPCLPointCloud2(cloud_ros, *cloud_pcl);

    // Segment biggest
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud_pcl);
    seg.segment(*viewer_inliers, *viewer_coeff);

    // Find z
    Z.push_back(viewer_coeff->values[3]);
    cout << "z       = " << Z.back() << std::endl;
    if (++loop_ctr >= 100)
    {
        cout << "z_MEAN  = " << std::accumulate(Z.begin(), Z.end(), 0.0f) / Z.size() << std::endl;
        std::exit(-1);
    }
}

// Main
int main(int argc, char** argv)
{
    //Init ros node ======================================================
    ros::init(argc, argv, "z_calib");
    ros::NodeHandle n("~");

    //==================================================================
    //============================== ROS PARAMS ============================
    //==================================================================

    std::string camera_topic;
    n.param<std::string>("camera_topic", camera_topic, "/camera_left/depth/points");

    //==================================================================
    //========================= Sub & Publishers =======================
    //==================================================================

    cout << "Subscribing to " << camera_topic << "..." << std::endl;
    ros::Subscriber sub_cloud = n.subscribe<sensor_msgs::PointCloud2>(camera_topic, 1, processCloud);
    ros::spin();

    return 0;
}
