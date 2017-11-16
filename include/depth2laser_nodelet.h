#ifndef DEPTH2LASER_NODELET_H
#define DEPTH2LASER_NODELET_H

#include <math.h>                           //PI definition
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <gpu_utils.h>
#include <transform_matrix.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/chrono.hpp>

namespace depth2laser
{
    class Depth2LaserNodelet : public nodelet::Nodelet
    {
    public:

        //====================================================
        //===================== TYPEDEFS =====================
        //====================================================

        /** \brief Message filters with exact time policy. */
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                                sensor_msgs::PointCloud2> MySyncPolicy;

        // ================================
        // ======== PUBLIC METHODS ========
        // ================================

        Depth2LaserNodelet();
        ~Depth2LaserNodelet();

    protected:

        virtual void onInit();
        void publish();

        //===================================
        //========== ROS VARIABLES ==========
        //===================================

        /** \brief Camera1 depth image subscriber. */
        message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_sensor_depth_1;

        /** \brief Camera2 depth image subscriber. */
        message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_sensor_depth_2;

        /** \brief Cameras messages synchronization. */
        message_filters::Synchronizer<MySyncPolicy> *sync;

        // ==================================
        // ======== ROS PUBLISHERS ==========
        // ==================================

        ros::Publisher pub_laser_scan;

        // ==================================
        // ========= ROS SUBSCRIBERS ========
        // ==================================

        ros::Subscriber sub_cloud_1;
        ros::Subscriber sub_cloud_2;

        /** \brief Sensor cloud received callback */
        void sensorCb( const sensor_msgs::PointCloud2ConstPtr &cloud_1,
                       const sensor_msgs::PointCloud2ConstPtr &cloud_2);

        // ==================================
        // ========== ROS MESSAGES ==========
        // ==================================

        sensor_msgs::LaserScan laser_scan;

        // ==================================
        // ======== ROS CALLBACKS ===========
        // ==================================

        void processCloud1(const sensor_msgs::PointCloud2ConstPtr &cloud);
        void processCloud2(const sensor_msgs::PointCloud2ConstPtr &cloud);

        sensor_msgs::PointCloud2ConstPtr cloud_1;
        sensor_msgs::PointCloud2ConstPtr cloud_2;

        // ==================================
        // ===== TRANSFORMATION MATRIX ======
        // ==================================

        TransformMatrix transformation_matrix1;
        TransformMatrix transformation_matrix2;

        // ==================================
        // ============== TEMP ==============
        // ==================================


        bool sensor1, sensor2, debug, sensors_sync, verbose;

        float cell_size;
        float sensor_radius;
        float tolerance;

        GPUUtils *gpu;
    };
}

#endif //DEPTH2LASER_NODELET_H
