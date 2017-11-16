#include <depth2laser_nodelet.h>
#include <pluginlib/class_list_macros.h>

#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */

namespace depth2laser
{
    Depth2LaserNodelet::Depth2LaserNodelet() : gpu{NULL} {}
    Depth2LaserNodelet::~Depth2LaserNodelet()
    {
        if (gpu) delete gpu;
    }

    void Depth2LaserNodelet::onInit()
    {
        NODELET_DEBUG("Initializing depth2laser nodelet...");

        //==================================================================
        //============================== ROS PARAMS ============================
        //==================================================================

        std::string base_link_frame;
        std::string camera_left_link_frame, camera_right_link_frame;
        std::string camera_left_topic, camera_right_topic;
        std::string output_laser_scan_topic;

        getPrivateNodeHandle().param<bool>("verbose", verbose, false);
        getPrivateNodeHandle().param<bool>("sensor1", sensor1, true);
        getPrivateNodeHandle().param<bool>("sensor2", sensor2, true);
        getPrivateNodeHandle().param<bool>("sensors_sync", sensors_sync, true);

        getPrivateNodeHandle().param<std::string>("base_link_frame", base_link_frame, "/batman/base_link");
        getPrivateNodeHandle().param<std::string>("camera_left_link_frame", camera_left_link_frame, "camera_left_link");
        getPrivateNodeHandle().param<std::string>("camera_right_link_frame", camera_right_link_frame, "camera_right_link");

        getPrivateNodeHandle().param<std::string>("output_laser_scan_topic", output_laser_scan_topic, "scan");
        getPrivateNodeHandle().param<std::string>("camera_left_topic", camera_left_topic, "/camera_left/depth/points");
        getPrivateNodeHandle().param<std::string>("camera_right_topic", camera_right_topic, "/camera_right/depth/points");

        //==================================================================
        //========================= Sub & Publishers =======================
        //==================================================================

        // GPU
        gpu = new GPUUtils(sensor1, sensor2);

        if (sensors_sync)
        {

            if (!sensor1 || !sensor2)
            {
                ROS_ERROR("In sync mode both sensors should be enabled.");
                std::exit(-1);
            }

            sub_sensor_depth_1 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), camera_left_topic, 2);
            sub_sensor_depth_2 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), camera_right_topic, 2);

            sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(2), *sub_sensor_depth_1, *sub_sensor_depth_2);
            sync->registerCallback(boost::bind(&Depth2LaserNodelet::sensorCb, this, _1, _2));

        } else
        {
            if (sensor1)
            {
                sub_cloud_1 = getPrivateNodeHandle().subscribe<sensor_msgs::PointCloud2>(camera_left_topic,1, &Depth2LaserNodelet::processCloud1, this, ros::TransportHints().tcpNoDelay());
                ROS_INFO("Starting depth2laser_driver for first sensor.");
            }
            if (sensor2)
            {
                sub_cloud_2 = getPrivateNodeHandle().subscribe<sensor_msgs::PointCloud2>(camera_right_topic,1, &Depth2LaserNodelet::processCloud2, this, ros::TransportHints().tcpNoDelay());
                ROS_INFO("Starting depth2laser_driver for second sensor.");
            }
            if (!sensor1 && !sensor2)
            {
                ROS_ERROR("No sensors specified - you should enable at least one of sensor1 or sensor2.");
                std::exit(-1);
            }

            ROS_INFO("Waiting for sensors to connect...");
            ros::Rate(0.2).sleep();
            if (!sub_cloud_1.getNumPublishers() && !sub_cloud_2.getNumPublishers())
            {
                ROS_ERROR("There is no primesense sensor connected! Please connect at least one sensor and try again...");
                std::exit(-1);
            }
        }
        ROS_INFO("Sensors connected successfully");

        pub_laser_scan = getPrivateNodeHandle().advertise<sensor_msgs::LaserScan>(output_laser_scan_topic, 1);
        laser_scan.header.frame_id = base_link_frame;
        laser_scan.angle_increment = ANGLE_INCREMENT_RAD;
        laser_scan.range_min = 0.5;
        laser_scan.range_max = 3.0;
        laser_scan.angle_min = -HORIZONTAL_VIEW_RAD/2;
        laser_scan.angle_max =  HORIZONTAL_VIEW_RAD/2;
        for (int i=0; i<LASER_SCANNER_POINTS; i++)
            laser_scan.ranges.push_back(laser_scan.range_max);

        //==================================================================
        //=========================== TRANSFORM ============================
        //==================================================================

        tf::TransformListener listener[2];
        tf::StampedTransform transform[2];

        ROS_INFO("Waiting for transforms...");
        if (sensor1)
        {
            try
            {
                listener[0].waitForTransform(base_link_frame, camera_left_link_frame,
                                             ros::Time(0), ros::Duration(1.0));
                listener[0].lookupTransform(base_link_frame, camera_left_link_frame,
                                            ros::Time(0), transform[0]);
            } catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                std::exit(-1);
            }

            tf::Vector3 vec = transform[0].getOrigin();
            tf::Matrix3x3 rot = transform[0].getBasis();
            tfScalar rot_z, rot_y, rot_x;
            rot.getEulerYPR(rot_z, rot_y, rot_x);
            transformation_matrix1.setRotationYPR(vec[0], vec[1], vec[2],
                                                  rot_z, rot_y, rot_x);
        }

        if (sensor2)
        {
            try
            {
                listener[1].waitForTransform(base_link_frame, camera_right_link_frame,
                                             ros::Time(0), ros::Duration(1.0));
                listener[1].lookupTransform(base_link_frame, camera_right_link_frame,
                                            ros::Time(0), transform[1]);
            } catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                std::exit(-1);
            }

            tf::Vector3 vec = transform[1].getOrigin();
            tf::Matrix3x3 rot = transform[1].getBasis();
            tfScalar rot_z, rot_y, rot_x;
            rot.getEulerYPR(rot_z, rot_y, rot_x);
            transformation_matrix2.setRotationYPR(vec[0], vec[1], vec[2],
                                                  rot_z, rot_y, rot_x);
        }
        ROS_INFO("Transforms obtained successfully");
    }

    void Depth2LaserNodelet::sensorCb(const sensor_msgs::PointCloud2ConstPtr &cloud_1,
                              const sensor_msgs::PointCloud2ConstPtr &cloud_2)
    {
        if (verbose)
        {
            std::cout << "========== SYNC===========" << std::endl;
            std::cout << "CLOUD1= " << cloud_1->header.stamp.sec << std::endl;
            std::cout << "CLOUD1= " << cloud_1->header.stamp.nsec/1000000 << std::endl;
            std::cout << "CLOUD2= " << cloud_2->header.stamp.sec << std::endl;
            std::cout << "CLOUD2= " << cloud_2->header.stamp.nsec/1000000 << std::endl;
            std::cout << "START = " << ros::Time::now().sec << std::endl;
            std::cout << "START = " << ros::Time::now().nsec/1000000 << std::endl;
        }

        boost::chrono::high_resolution_clock::time_point start = boost::chrono::high_resolution_clock::now();

        for (int i=0; i<LASER_SCANNER_POINTS; i++)
            laser_scan.ranges[i] = laser_scan.range_max;
        gpu->computeLaserScan(&cloud_1->data[0], transformation_matrix1.getTransformationMatrix(),
                              &cloud_2->data[0], transformation_matrix2.getTransformationMatrix(),
                              0.075, 0.3, laser_scan.ranges.data());

        boost::chrono::milliseconds ms = boost::chrono::duration_cast<boost::chrono::milliseconds> (boost::chrono::high_resolution_clock::now() - start);
        std::cout << "Compute laser scan took: " << ms.count() << "ms " << "\n";

        if (verbose)
        {
            std::cout << "STOP  = " << ros::Time::now().sec << std::endl;
            std::cout << "STOP  = " << ros::Time::now().nsec/1000000 << std::endl;
            std::cout << "============" << std::endl;
        }

        laser_scan.header.stamp = cloud_1->header.stamp + (cloud_2->header.stamp - cloud_1->header.stamp) * 0.5;
        pub_laser_scan.publish(laser_scan);
    }

    void Depth2LaserNodelet::processCloud1(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        cloud_1 = cloud;
    }
    void Depth2LaserNodelet::processCloud2(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        cloud_2 = cloud;
        publish();
    }
    void Depth2LaserNodelet::publish()
    {
        if (verbose)
        {
            std::cout << "========== SYNC===========" << std::endl;
            std::cout << "CLOUD1= " << cloud_1->header.stamp.sec << std::endl;
            std::cout << "CLOUD1= " << cloud_1->header.stamp.nsec/1000000 << std::endl;
            std::cout << "CLOUD2= " << cloud_2->header.stamp.sec << std::endl;
            std::cout << "CLOUD2= " << cloud_2->header.stamp.nsec/1000000 << std::endl;
            std::cout << "START = " << ros::Time::now().sec << std::endl;
            std::cout << "START = " << ros::Time::now().nsec/1000000 << std::endl;
        }

        for (int i=0; i<LASER_SCANNER_POINTS; i++)
            laser_scan.ranges[i] = laser_scan.range_max;
        gpu->computeLaserScan(&cloud_1->data[0], transformation_matrix1.getTransformationMatrix(),
                              &cloud_2->data[0], transformation_matrix2.getTransformationMatrix(),
                              0.075, 0.3, laser_scan.ranges.data());

        if (verbose)
        {
            std::cout << "STOP  = " << ros::Time::now().sec << std::endl;
            std::cout << "STOP  = " << ros::Time::now().nsec/1000000 << std::endl;
            std::cout << "============" << std::endl;
        }

        laser_scan.header.stamp = cloud_1->header.stamp + (cloud_2->header.stamp - cloud_1->header.stamp) * 0.5;
        pub_laser_scan.publish(laser_scan);
    }
}

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(depth2laser, Depth2LaserNodelet, depth2laser::Depth2LaserNodelet, nodelet::Nodelet)
