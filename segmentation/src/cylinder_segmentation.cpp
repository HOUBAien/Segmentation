// Dynamic, Always subscribing, updating
// ===============================================================
#include <ros/ros.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <segmentation/cylinder_data.h>
#include <Eigen/Dense>

class cylinderSegmentation
{
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub_;
    ros::Publisher cylinder_pub_;
    bool viewDisplay;
    bool cmdOutput;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:

    cylinderSegmentation()
    {
        cloud_sub_ = nh.subscribe("/camera/depth_registered/points", 1, &cylinderSegmentation::cloud_cb, this);
        cylinder_pub_ = nh.advertise<segmentation::cylinder_data>("cylinder_data", 10);

        //cloud_sub_ = nh.subscribe("cloudInput", 1, &cylinderSegmentation::cloud_cb, this);
        if (!nh.getParam("viewDisplay", viewDisplay))
        {
            viewDisplay = false;
        }
        if (!nh.getParam("cmdOutput", cmdOutput))
        {
            cmdOutput = false;
        }
        if (viewDisplay) viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));

        if (cmdOutput) ROS_INFO("cylinder Segmentation Started");
        if (viewDisplay)
        {
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addCoordinateSystem(0.5);
            viewer->initCameraParameters();
        }

    }

    ~cylinderSegmentation()
    {
        if (viewDisplay) viewer->close();
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
    // void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
    {
        // ROS_INFO("Enter Cloud CB");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::fromROSMsg(*input, *cloud);
        // pcl::fromPCLPointCloud2 (*input, *cloud);


        pcl::PassThrough<pcl::PointXYZRGB> pass;
        // Build a passthrough filter to remove spurious NaNs
        if (cmdOutput) std::cerr << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.0);
        pass.filter(*cloud_filtered);
        if (cmdOutput) std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;


        // Estimate point normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        // Create the segmentation object for the planar model and set all the parameters
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_plane, *coefficients_plane);
        if (cmdOutput) std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        extract.filter (*cloud_plane);
        if (cmdOutput) std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        extract.setNegative (true);
        extract.filter (*cloud_filtered2);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);

        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02);
        seg.setRadiusLimits(0, 0.1);
        seg.setInputCloud(cloud_filtered2);
        seg.setInputNormals(cloud_normals2);

        // Obtain the cylinder inliers and coefficients
        // http://docs.pointclouds.org/1.7.2/a01146.html
        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        if (cmdOutput) std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        extract.setInputCloud(cloud_filtered2);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);

        extract.setNegative(true);
        extract.filter(*cloud_filtered);


        if (viewDisplay)
        {
            viewer->removePointCloud("sample cloud");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

            viewer->removePointCloud("plane cloud");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plane, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "plane cloud");


            //ROS_INFO("Cloud Viewer Entered");
            if (cloud_cylinder->points.empty ())
                std::cerr << "Can't find the cylindrical component." << std::endl;
            else
            {
                if (cmdOutput) std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
                viewer->removePointCloud("cylinder cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cylinder, "cylinder cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cylinder cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "cylinder cloud");
            }
            if (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }

        }

        if (!cloud_cylinder->points.empty ())
        {
            Eigen::Vector3d pseudo_center(0, 0, 0);

            int num_points_in_cylinder = cloud_cylinder->points.size();
            for (int i = 0; i < num_points_in_cylinder; i++)
            {
                pseudo_center(0) += cloud_cylinder->points[i].x;
                pseudo_center(1) += cloud_cylinder->points[i].y;
                pseudo_center(2) += cloud_cylinder->points[i].z;
            }
            pseudo_center /= num_points_in_cylinder;

            Eigen::Vector3d central_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);

            // Project the pseduo_center onto the central_axis
            // https://en.wikipedia.org/wiki/Vector_projection
            Eigen::Vector3d point_on_central_axis(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]);
            Eigen::Vector3d pseudo_center_vector = pseudo_center - point_on_central_axis;
            Eigen::Vector3d pseudo_center_vector_projection = pseudo_center_vector.dot(central_axis) / central_axis.dot(central_axis) * central_axis;
            Eigen::Vector3d center = pseudo_center_vector_projection + point_on_central_axis;

            std::vector<double> data(10, 0);
            for (int i = 0; i < 7; i++)
            {
                data[i] = coefficients_cylinder->values[i];
            }
            // data[7] = pseudo_center(0);
            // data[8] = pseudo_center(1);
            // data[9] = pseudo_center(2);
            data[7] = center(0);
            data[8] = center(1);
            data[9] = center(2);


            segmentation::cylinder_data msg;
            msg.cylinder_data = data;
            cylinder_pub_.publish(msg);
        }


    }
};


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cylinder_seg");
    cylinderSegmentation cs;
    ros::spin();

    // while (!cs.viewer->wasStopped() && ros::ok())
    // {
    //     cs.viewer->spinOnce();
    //     ros::spinOnce();
    // }
}



// // Static, read pcd from existing file
// // ===============================================================

// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/conversions.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <boost/thread/thread.hpp>

// #include <pcl/common/common_headers.h>
// #include <pcl/console/parse.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>

// typedef pcl::PointXYZRGB PointT;

// int
// main (int argc, char** argv)
// {
//     // All the objects needed
//     pcl::PCDReader reader;
//     pcl::PassThrough<PointT> pass;
//     pcl::NormalEstimation<PointT, pcl::Normal> ne;
//     pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//     pcl::PCDWriter writer;
//     pcl::ExtractIndices<PointT> extract;
//     pcl::ExtractIndices<pcl::Normal> extract_normals;
//     pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

//     // Datasets
//     pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//     pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

//     // Read in the cloud data
//     reader.read ("/home/chentao/Documents/test_binary.pcd", *cloud);
//     std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

//     // Build a passthrough filter to remove spurious NaNs
//     pass.setInputCloud (cloud);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (0, 1.0);
//     pass.filter (*cloud_filtered);
//     std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

//     // Estimate point normals
//     ne.setSearchMethod (tree);
//     ne.setInputCloud (cloud_filtered);
//     ne.setKSearch (50);
//     ne.compute (*cloud_normals);

//     // Create the segmentation object for the planar model and set all the parameters
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//     seg.setNormalDistanceWeight (0.1);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setMaxIterations (100);
//     seg.setDistanceThreshold (0.03);
//     seg.setInputCloud (cloud_filtered);
//     seg.setInputNormals (cloud_normals);
//     // Obtain the plane inliers and coefficients
//     seg.segment (*inliers_plane, *coefficients_plane);
//     std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

//     // Extract the planar inliers from the input cloud
//     extract.setInputCloud (cloud_filtered);
//     extract.setIndices (inliers_plane);
//     extract.setNegative (false);

//     // Write the planar inliers to disk
//     pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//     extract.filter (*cloud_plane);
//     std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//     writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

//     // Remove the planar inliers, extract the rest
//     extract.setNegative (true);
//     extract.filter (*cloud_filtered2);
//     extract_normals.setNegative (true);
//     extract_normals.setInputCloud (cloud_normals);
//     extract_normals.setIndices (inliers_plane);
//     extract_normals.filter (*cloud_normals2);

//     // Create the segmentation object for cylinder segmentation and set all the parameters
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_CYLINDER);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setNormalDistanceWeight (0.1);
//     seg.setMaxIterations (10000);
//     seg.setDistanceThreshold (0.05);
//     seg.setRadiusLimits (0, 0.1);
//     seg.setInputCloud (cloud_filtered2);
//     seg.setInputNormals (cloud_normals2);

//     // Obtain the cylinder inliers and coefficients
//     seg.segment (*inliers_cylinder, *coefficients_cylinder);
//     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//     // Write the cylinder inliers to disk
//     extract.setInputCloud (cloud_filtered2);
//     extract.setIndices (inliers_cylinder);
//     extract.setNegative (false);
//     pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
//     extract.filter (*cloud_cylinder);
//     if (cloud_cylinder->points.empty ())
//         std::cerr << "Can't find the cylindrical component." << std::endl;
//     else
//     {
//         std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
//         writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
//     }


//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//     viewer->setBackgroundColor(0, 0, 0);
//     viewer->addCoordinateSystem(0.5);
//     viewer->initCameraParameters();
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
//     viewer->removePointCloud("sample cloud");
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered,rgb, "sample cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

//     viewer->removePointCloud("plane cloud");
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plane, "plane cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "plane cloud");

//     viewer->removePointCloud("cylinder cloud");
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cylinder, "cylinder cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cylinder cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "cylinder cloud");
//     while (!viewer->wasStopped())
//     {
//         viewer->spinOnce();
//     }
//     return (0);
// }
