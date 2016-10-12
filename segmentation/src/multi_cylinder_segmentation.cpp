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

#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#if !defined(__SUNPRO_CC) || (__SUNPRO_CC > 0x530)
#include <boost/generator_iterator.hpp>
#endif

const int colors_size = 10;
const double colors[colors_size][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
    {0.3, 0.5, 0.3},
    {0.8, 0.2, 0.5},
    {0.8, 0.3, 0.2},
    {0.8, 0.8, 0.2},
    {0.5, 0.9, 0.2}
};

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
        cloud_sub_ = nh.subscribe("/camera/depth_registered/points", 1, &cylinderSegmentation::cloud_cb, this);   //x, y, z in rgb_optical_frame
        cylinder_pub_ = nh.advertise<segmentation::cylinder_data>("cylinder_data", 10);

        //cloud_sub_ = nh.subscribe("cloudInput", 1, &cylinderSegmentation::cloud_cb, this);
        if (!nh.getParam("viewDisplay", viewDisplay))
        {
            viewDisplay = true;
        }
        if (!nh.getParam("cmdOutput", cmdOutput))
        {
            cmdOutput = true;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cylinders;
        std::vector<pcl::ModelCoefficients::Ptr> cylinder_coefficients;

        pcl::fromROSMsg(*input, *cloud);
        // pcl::fromPCLPointCloud2 (*input, *cloud);


        pcl::PassThrough<pcl::PointXYZ> pass;
        // Build a passthrough filter to remove spurious NaNs
        if (cmdOutput) std::cerr << "PointCloud before passthrough filtering has: " << cloud->points.size () << " data points." << std::endl;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.0);
        pass.filter(*cloud_filtered);
        if (cmdOutput) std::cerr << "PointCloud after passthrough filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        // ============================================================================================================
        // Downsampling a PointCloud using a VoxelGrid filter
        // pcl::VoxelGrid<pcl::PointXYZ> vsor;
        // vsor.setInputCloud (cloud_filtered);
        // vsor.setLeafSize (0.0025f, 0.0025f, 0.0025f);
        // vsor.filter (*cloud_filtered);
        // if (cmdOutput) std::cerr << "PointCloud after VoxelGrid filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        // viewer->removeAllPointClouds();
        // viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "voxel");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "voxel");
        // viewer->spinOnce();
        // ============================================================================================================

        // ============================================================================================================
        // Removing outliers using a StatisticalOutlierRemoval filter
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud (cloud_filtered);
        // sor.setMeanK (50);
        // sor.setStddevMulThresh (1.0);
        // sor.filter (*cloud_filtered);
        // if (cmdOutput) std::cerr << "PointCloud after StatisticalOutlierRemoval filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        // sor.setNegative (true);
        // sor.filter (*cloud_filtered2);


        // viewer->removeAllPointClouds();

        // viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "inliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "inliers");

        // viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered2, "outliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "outliers");
        // viewer->spinOnce();
        // // ============================================================================================================

        // Estimate point normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        // Create the segmentation object for the planar model and set all the parameters
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_plane, *coefficients_plane);
        if (cmdOutput) std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        pcl::ModelCoefficients::Ptr coefficients_plane_loose (new pcl::ModelCoefficients);
        seg.segment (*inliers_plane, *coefficients_plane_loose);
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        extract.filter (*cloud_plane);
        if (cmdOutput) std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals);

        // Create the segmentation objects for cylinder segmentation and set all the parameters

        bool mayHaveCylinder = true;
        do
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ> ());
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight(0.1);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.05);
            seg.setRadiusLimits(0, 0.1);
            seg.setInputCloud(cloud_filtered);
            seg.setInputNormals(cloud_normals);

            // Obtain the cylinder inliers and coefficients
            // http://docs.pointclouds.org/1.7.2/a01146.html
            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
            seg.segment(*inliers_cylinder, *coefficients_cylinder);



            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers_cylinder);
            extract.setNegative(false);
            extract.filter(*cloud_cylinder);

            if (cloud_cylinder->points.size() > 2000)
            {
                // if (cmdOutput) std::cerr << "Cylinder [" << cloud_cylinders.size() + 1 <<"] coefficients: " << *coefficients_cylinder << std::endl;
                cylinder_coefficients.push_back(coefficients_cylinder);
                cloud_cylinders.push_back(cloud_cylinder);

                extract.setNegative(true);
                extract.filter(*cloud_filtered);
                std::cerr << "cloud filtered size:" << cloud_filtered->points.size() << std::endl;
                extract_normals.setNegative (true);
                extract_normals.setInputCloud (cloud_normals);
                extract_normals.setIndices (inliers_cylinder);
                extract_normals.filter (*cloud_normals);
            }
            else
            {
                mayHaveCylinder = false;
            }

        } while (mayHaveCylinder);



        if (viewDisplay)
        {
            viewer->removeAllPointClouds();

            viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "all cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all cloud");


            viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "plane cloud");


            //ROS_INFO("Cloud Viewer Entered");
            if (cloud_cylinders.empty ())
                std::cerr << "Can't find any cylindrical component." << std::endl;
            else
            {
                typedef boost::minstd_rand base_generator_type;
                base_generator_type generator(6u);
                for (int i = 0; i < cloud_cylinders.size(); i++)
                {
                    std::string str = "cylinder cloud ";
                    str.append(boost::lexical_cast<std::string>(i));
                    std::cerr << "cylinder name:" << str << std::endl;
                    std::cerr << "points size:" << cloud_cylinders[i]->points.size() << std::endl;
                    // std::cerr<<cloud_cylinders[i]<<std::endl;
                    viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinders[i], str);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);



                    double r;
                    double g;
                    double b;
                    if (i < colors_size)
                    {
                        r = colors[i][0];
                        g = colors[i][1];
                        b = colors[i][2];
                    }
                    else
                    {
                        boost::uniform_real<> uni_dist(0, 1);
                        boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);
                        r = uni();
                        g = uni();
                        b = uni();
                    }

                    std::cerr << "r: " << r << "    g: " << g << "    b: " << b << std::endl;

                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r , g, b, str);
                }
            }
            if (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }

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

// typedef pcl::PointXYZ PointT;

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
//     reader.read ("/home/chentao/Documents/table_cup_pcd.pcd", *cloud);
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
//     viewer->removePointCloud("sample cloud");
//     viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

//     viewer->removePointCloud("plane cloud");
//     viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "plane cloud");

//     viewer->removePointCloud("cylinder cloud");
//     viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "cylinder cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cylinder cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "cylinder cloud");
//     while (!viewer->wasStopped())
//     {
//         viewer->spinOnce();
//     }
//     return (0);
// }
