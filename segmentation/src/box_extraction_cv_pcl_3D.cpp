
#include <ros/ros.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <segmentation/box_3D_data.h>
#include <algorithm>



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


cv::Rect rectangle_;
bool drawBox = false;
bool rectDone = false;

void mouseHandle(int event, int x, int y, int flags, void* param)
{
    cv::Mat& image = *(cv::Mat*)param;
    if (rectDone)
        return;
    switch (event)
    {
    case cv::EVENT_MOUSEMOVE:
    {
        rectangle_.width = x - rectangle_.x;
        rectangle_.height = y - rectangle_.y;
    }
    break;
    case cv::EVENT_LBUTTONDOWN:
    {
        rectangle_ = cv::Rect(x, y, 0, 0);
        drawBox = true;
    }
    break;
    case cv::EVENT_LBUTTONUP:
    {
        drawBox = false;
        if (rectangle_.width < 0)
        {
            rectangle_.x += rectangle_.width;
            rectangle_.width *= -1;
        }
        if (rectangle_.height < 0)
        {
            rectangle_.y += rectangle_.height;
            rectangle_.height *= -1;
        }
        rectDone = true;
    }
    }

}

struct IdxCompare
{   //descending order
    const std::vector<double>& target;

    IdxCompare(const std::vector<double>& target): target(target) {}

    bool operator()(int a, int b) const { return target[a] > target[b]; }
};


class boxSegmentation
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_rgb_;
    ros::Subscriber cloud_sub_;
    ros::Publisher box_pub_;
    bool viewDisplay;
    bool cmdOutput;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
    std::string cvwindowName;
    int imageWidth;
    int imageHeight;

public:
    cv::Mat rgbImage_;
    boxSegmentation(ros::NodeHandle nh_): nh(nh_), it_(nh_)
    {
        cloud_sub_ = nh.subscribe("/camera/depth_registered/points", 1, &boxSegmentation::cloud_cb, this);
        box_pub_ = nh.advertise<segmentation::box_3D_data>("box_3D_data", 10);

        //cloud_sub_ = nh.subscribe("cloudInput", 1, &boxSegmentation::cloud_cb, this);
        if (!nh.getParam("viewDisplay", viewDisplay))
        {
            viewDisplay = true;
        }
        if (!nh.getParam("cmdOutput", cmdOutput))
        {
            cmdOutput = true;
        }
        if (viewDisplay)
        {
            viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer2.reset(new pcl::visualization::PCLVisualizer("3D Viewer Whole"));
        }

        if (cmdOutput) ROS_INFO("Box Segmentation Started");
        if (viewDisplay)
        {
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addCoordinateSystem(0.5);
            viewer->initCameraParameters();
        }
        imageWidth = 640;
        imageHeight = 480;
        cvwindowName = "RGB Image window";
        cv::namedWindow(cvwindowName);
        cv::setMouseCallback(cvwindowName, mouseHandle, (void*)&rgbImage_);
        image_sub_rgb_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &boxSegmentation::rgbCb, this);

    }

    ~boxSegmentation()
    {
        if (viewDisplay) viewer->close();
    }

    void rgbCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // ROS_INFO("Image Received");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(rgbImage_);
        if (rectDone || drawBox)
        {
            cv::rectangle(rgbImage_, rectangle_.tl(), rectangle_.br(), cv::Scalar(255, 0, 0), 3);
        }

        cv::imshow(cvwindowName, rgbImage_);
        cv::waitKey(5);

    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
    // void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
    {
        // ROS_INFO("Point Cloud Received");
        if (!rectDone)
        {
            return;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_copy(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_original_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_copy(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZRGB> ());

        pcl::fromROSMsg(*input, *cloud_original);
        pcl::copyPointCloud(*cloud_original, *cloud);
        // pcl::fromPCLPointCloud2 (*input, *cloud);
        cv::Point topLeft = rectangle_.tl();
        // cv::Point topLeft(10,10);
        int rect_width = rectangle_.width;
        int rect_height = rectangle_.height;
        // int width = 600;
        // int height = 400;
        pcl::PointIndices::Ptr roiIndices(new pcl::PointIndices);
        generateIndices(*roiIndices, topLeft, rect_width, rect_height);

        if (viewDisplay)
        {
            viewer2->removePointCloud("All Points");
            viewer2->addPointCloud<pcl::PointXYZRGB>(cloud, "All Points");
            viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "All Points");
            viewer2->spinOnce();
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(roiIndices);
        extract.setNegative(false);
        extract.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        // Build a passthrough filter to remove spurious NaNs
        if (cmdOutput) std::cerr << "PointCloud (Whole) before filtering has: " << cloud_original->points.size () << " data points." << std::endl;
        pass.setInputCloud (cloud_original);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.0);
        pass.filter(*cloud_original_filtered);
        if (cmdOutput) std::cerr << "PointCloud (Whole) after filtering has: " << cloud_original_filtered->points.size () << " data points." << std::endl;

        if (cmdOutput) std::cerr << "PointCloud in the selected region the before filtering has: " << cloud->points.size () << " data points." << std::endl;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.0);
        pass.filter(*cloud_filtered);
        if (cmdOutput) std::cerr << "PointCloud in the selected region after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;



        // Estimate point normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        ne.setInputCloud(cloud_original_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_original_normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        // Create the segmentation object for the planar model and set all the parameters
        pcl::ModelCoefficients::Ptr coefficients_table (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_table (new pcl::PointIndices);

        // To get the precise normal vector of the plane, but this method will leave out a lot of points that should be included in the table plane,
        // so I use pcl::SACMODEL_NORMAL_PLANE first to get the relatively accurate normal vector, and then use pcl::SACMODEL_PLANE as a slack condition to get the plane
        // seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud_original_filtered);
        seg.setInputNormals (cloud_original_normals);
        // Obtain the plane inliers and coefficients
        seg.segment (*inliers_table, *coefficients_table);
        if (cmdOutput) std::cerr << "Table coefficients: " << *coefficients_table << std::endl;

        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud_original_filtered);
        seg.setInputNormals (cloud_original_normals);
        // Obtain the plane inliers and coefficients
        pcl::ModelCoefficients::Ptr coefficients_table_wide (new pcl::ModelCoefficients);
        seg.segment (*inliers_table, *coefficients_table_wide);
        // Extract the planar inliers from the input cloud


        extract.setInputCloud (cloud_original_filtered);
        extract.setIndices (inliers_table);
        extract.setNegative (false);

        extract.filter (*cloud_table);
        if (cmdOutput) std::cerr << "PointCloud representing the table planar component: " << cloud_table->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        // extract.setNegative (true);
        // extract.filter (*cloud_filtered);
        // extract_normals.setNegative (true);
        // extract_normals.setInputCloud (cloud_normals);
        // extract_normals.setIndices (inliers_table);
        // extract_normals.filter (*cloud_normals);

        pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_copy);
        pcl::copyPointCloud(*cloud_normals, *cloud_normals_copy);

        Eigen::Vector3f table_normal(coefficients_table->values[0], coefficients_table->values[1], coefficients_table->values[2]);

        std::vector<pcl::ModelCoefficients::Ptr> coefficients_planes;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_planes;
        bool mayHavePlane = true;
        bool boxParallelToTable = false;


        double height = 0;
        do
        {
            pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB> ());

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
            seg.setNormalDistanceWeight (0.1);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);
            seg.setInputCloud (cloud_filtered_copy);
            seg.setInputNormals (cloud_normals_copy);
            seg.segment (*inliers_plane, *coefficients_plane);

            extract.setInputCloud(cloud_filtered_copy);
            extract.setIndices(inliers_plane);
            extract.setNegative(false);
            extract.filter(*cloud_plane);

            int planePointSizeThresh = rect_width * rect_height * 0.05;
            int parallelPlanePointSizeThresh = rect_width * rect_height * 0.2;
            // std::cerr<<"planePointSizeThresh:"<<planePointSizeThresh<<std::endl;
            // std::cerr<<"parallelPlanePointSizeThresh:"<<parallelPlanePointSizeThresh<<std::endl;

            if (cloud_plane->points.size() > planePointSizeThresh)
            {
                // cloud_plane->points.size() > parallelPlanePointSizeThresh is used to filter invalid horizontal plane

                if (isParallelToTable(coefficients_table, coefficients_plane, height) && cloud_plane->points.size() > parallelPlanePointSizeThresh)
                {
                    mayHavePlane = false;
                    boxParallelToTable = true;

                    coefficients_planes.push_back(coefficients_plane);
                    cloud_planes.push_back(cloud_plane);
                }
                else
                {
                    coefficients_planes.push_back(coefficients_plane);
                    cloud_planes.push_back(cloud_plane);

                    extract.setNegative(true);
                    extract.filter(*cloud_filtered_copy);
                    extract_normals.setNegative (true);
                    extract_normals.setInputCloud (cloud_normals_copy);
                    extract_normals.setIndices (inliers_plane);
                    extract_normals.filter (*cloud_normals_copy);
                }
            }
            else
            {
                if (cmdOutput) std::cerr << "No more planes..." << std::endl;
                mayHavePlane = false;
            }

        } while (mayHavePlane);

        if (boxParallelToTable)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB> ());
            pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
            cloud_plane = cloud_planes.back();
            coefficients_plane = coefficients_planes.back();
            std::cerr << "Find a plane that is parallel to the table, using 2D box pose estimation algorithm..." << std::endl;
            if (cmdOutput) std::cerr << "Point size on the box top plane: " << cloud_plane->points.size() << std::endl;

            std::vector<double> msg_center(3, 0);

            double msg_longestEdgeLen;
            double msg_medianEdgeLen;
            double msg_shortestEdgeLen;

            std::vector<double> msg_longestEdgeDir(3, 0);
            std::vector<double> msg_medianEdgeDir(3, 0);
            std::vector<double> msg_shortestEdgeDir(3, 0);



            // if (cmdOutput) std::cerr << "Box Top Face Plane coefficients: " << *coefficients_plane << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_flat(new pcl::PointCloud<pcl::PointXYZRGB>);
            Eigen::Matrix3f inv_plane_rotation;
            projectToPlaneNormalToZAxis(cloud_plane, coefficients_table, cloud_projected, cloud_projected_flat, inv_plane_rotation);

            // Find the convex hull
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
            convex_hull.setInputCloud(cloud_projected_flat);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*cloud_hull);


            double min_area = 10000000;

            // if (cmdOutput) std::cerr << "Cloud Hull Size: " << cloud_hull->size() << std::endl;
            for (int i = 0; i < cloud_hull->size(); i++)
            {

                Eigen::Matrix3f inv_rotation;
                double x_min = 1000.0;
                double x_max = -1000.0;
                double y_min = 1000.0;
                double y_max = -1000.0;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                rotateEdgeToBeXAxis(i, cloud_hull, x_min, x_max, y_min, y_max, inv_rotation, projected_cloud);

                double area = (x_max - x_min) * (y_max - y_min);

                if (area  < min_area)
                {
                    Eigen::Matrix3f transformation = inv_plane_rotation * inv_rotation;

                    Eigen::Vector3f edgeOneDir(1, 0, 0);
                    Eigen::Vector3f edgeTwoDir(0, 1, 0);
                    Eigen::Vector3f edgeThreeDir(0, 0, 1);

                    std::vector<Eigen::Vector3f> edgesDir;


                    edgesDir.push_back(edgeOneDir);
                    edgesDir.push_back(edgeTwoDir);
                    edgesDir.push_back(edgeThreeDir);
                    std::vector<Eigen::Vector3f> sortedEdgesDir(edgesDir.size());

                    std::vector<double> edgesLength(3, 0);
                    std::vector<double> sortedEdgesLength(3);  //descending order

                    edgesLength[0] = (x_max - x_min);
                    edgesLength[1] = (y_max - y_min);
                    edgesLength[2] = height;

                    sortEdges(edgesLength, sortedEdgesLength, edgesDir, sortedEdgesDir);


                    msg_longestEdgeLen = sortedEdgesLength[0];
                    msg_medianEdgeLen = sortedEdgesLength[1];
                    msg_shortestEdgeLen = sortedEdgesLength[2];



                    Eigen::Vector3f center((x_max + x_min) / 2.0, (y_max + y_min) / 2.0, projected_cloud->points[0].z + height / 2.0);
                    center = transformation * center;
                    for (int idx_t = 0; idx_t < sortedEdgesDir.size(); idx_t++)
                    {
                        sortedEdgesDir[idx_t] = transformation * sortedEdgesDir[idx_t];
                    }
                    storeEdgesDir(sortedEdgesDir, msg_longestEdgeDir, msg_medianEdgeDir, msg_shortestEdgeDir);
                    msg_center[0] = center(0);
                    msg_center[1] = center(1);
                    msg_center[2] = center(2);

                    min_area = area;
                }
            }

            segmentation::box_3D_data msg;
            msg.center = msg_center;
            msg.longestEdgeLen = msg_longestEdgeLen;
            msg.medianEdgeLen = msg_medianEdgeLen;
            msg.shortestEdgeLen = msg_shortestEdgeLen;
            msg.longestEdgeDir = msg_longestEdgeDir;
            msg.medianEdgeDir = msg_medianEdgeDir;
            msg.shortestEdgeDir = msg_shortestEdgeDir;

            box_pub_.publish(msg);

            if (viewDisplay)
            {
                for (int i = 0; i < cloud_hull->size(); i++)
                {
                    cloud_hull->points[i].getVector3fMap() = inv_plane_rotation * cloud_hull->points[i].getVector3fMap();
                }
            }


            if (viewDisplay)
            {
                viewer->removePointCloud("cloud_filtered");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "cloud_filtered");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");

                viewer->removePointCloud("table cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_table, "table cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "table cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "table cloud");

                viewer->removePointCloud("box top cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plane, "box top cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "box top cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "box top cloud");

                viewer->removePointCloud("projected cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_projected, "projected cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "projected cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 0.0, 1.0, "projected cloud");


                viewer->removePointCloud("convex cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_hull, "convex cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "convex cloud");


                if (!viewer->wasStopped())
                {
                    viewer->spinOnce();
                }

            }
        }
        else
        {
            // if the box is in the 3D space and not parallel to the table, then use the following pose estimation algorithm
            std::cerr << "Using 3D box pose estimation algorithm..." << std::endl;

            std::vector<double> msg_center(3, 0);

            double msg_longestEdgeLen;
            double msg_medianEdgeLen;
            double msg_shortestEdgeLen;

            std::vector<double> msg_longestEdgeDir(3, 0);
            std::vector<double> msg_medianEdgeDir(3, 0);
            std::vector<double> msg_shortestEdgeDir(3, 0);

            // Find three planes that are perpendicular to each other
            int numOfPlanes = cloud_planes.size();
            bool findThreePerpendPlanes = false;

            std::vector<int> planeIndices(3, 0);

            if (numOfPlanes >= 3)
            {
                for (int i = 0; (i < numOfPlanes - 2) && !findThreePerpendPlanes; i++)
                {
                    for (int j = i + 1; (j < numOfPlanes - 1) && !findThreePerpendPlanes; j++)
                    {
                        for (int k = j + 1; (k < numOfPlanes) && !findThreePerpendPlanes; k++)
                        {
                            Eigen::Vector3f planeOneNormal(coefficients_planes[i]->values[0], coefficients_planes[i]->values[1], coefficients_planes[i]->values[2]);
                            Eigen::Vector3f planeTwoNormal(coefficients_planes[j]->values[0], coefficients_planes[j]->values[1], coefficients_planes[j]->values[2]);
                            Eigen::Vector3f planeThreeNormal(coefficients_planes[k]->values[0], coefficients_planes[k]->values[1], coefficients_planes[k]->values[2]);
                            double normalProductThresh = 0.2;
                            if (fabs(planeOneNormal.dot(planeTwoNormal)) < normalProductThresh &&
                                    fabs(planeTwoNormal.dot(planeThreeNormal)) < normalProductThresh &&
                                    fabs(planeThreeNormal.dot(planeOneNormal)) < normalProductThresh)
                            {
                                findThreePerpendPlanes = true;
                                planeIndices[0] = i;
                                planeIndices[1] = j;
                                planeIndices[2] = k;
                            }
                        }
                    }
                }
            }

            if (findThreePerpendPlanes)
            {
                std::cerr << "Find 3 perpendicular planes..." << std::endl;
                // find the intersecting point among the three planes
                // a1x + b1y + c1z + d1 = 0
                // a2x + b2y + c2z + d2 = 0
                // a3x + b3y + c3z + d3 = 0
                Eigen::Matrix3f A_planes;
                Eigen::Vector3f crossingPoint;
                Eigen::Vector3f center;
                A_planes(0, 0) = coefficients_planes[planeIndices[0]]->values[0];
                A_planes(0, 1) = coefficients_planes[planeIndices[0]]->values[1];  // -theta, not theta
                A_planes(0, 2) = coefficients_planes[planeIndices[0]]->values[2];
                A_planes(1, 0) = coefficients_planes[planeIndices[1]]->values[0];
                A_planes(1, 1) = coefficients_planes[planeIndices[1]]->values[1];
                A_planes(1, 2) = coefficients_planes[planeIndices[1]]->values[2];
                A_planes(2, 0) = coefficients_planes[planeIndices[2]]->values[0];
                A_planes(2, 1) = coefficients_planes[planeIndices[2]]->values[1];
                A_planes(2, 2) = coefficients_planes[planeIndices[2]]->values[2];

                Eigen::Vector3f d_column(coefficients_planes[planeIndices[0]]->values[3], coefficients_planes[planeIndices[1]]->values[3], coefficients_planes[planeIndices[2]]->values[3]);
                Eigen::Matrix3f A_planes_inv;
                bool A_planes_invertible;
                A_planes.computeInverseWithCheck(A_planes_inv, A_planes_invertible);
                if (A_planes_invertible)
                {
                    crossingPoint = -A_planes_inv * d_column;
                    std::vector<Eigen::Vector3f> faceCenters;
                    getPointsOnFace(cloud_planes, coefficients_planes, planeIndices, faceCenters);



                    // edgesLength contains the length, width, height
                    // edgesLength[0] is the edge length between the first plane and the second plane
                    // edgesLength[1] is the edge length between the second plane and the third plane
                    // edgesLength[2] is the edge length between the third plane and the first plane
                    // edgesDirection[0] is the unit vector representing the edge between the first plane and the second plane
                    // edgesDirection[1] is the unit vector representing the edge between the second plane and the third plane
                    // edgesDirection[2] is the unit vector representing the edge between the third plane and the first plane
                    std::vector<double> edgesLength(planeIndices.size(), 0);
                    std::vector<Eigen::Vector3f> edgesDirection;


                    center = crossingPoint;
                    for (int w = 0; w < planeIndices.size(); w++)
                    {
                        int planeIndex = (w + 2) % planeIndices.size();
                        Eigen::Vector3f normal(coefficients_planes[planeIndices[planeIndex]]->values[0],
                                               coefficients_planes[planeIndices[planeIndex]]->values[1],
                                               coefficients_planes[planeIndices[planeIndex]]->values[2]);

                        Eigen::Vector3f faceDiagonal;
                        faceDiagonal = faceCenters[w] - crossingPoint;
                        double projection = faceDiagonal.dot(normal);
                        edgesLength[w] = fabs(projection) * 2.0;
                        int dir = projection >= 0 ? 1 : -1;
                        edgesDirection.push_back(normal * dir);

                        center += edgesDirection.back() * fabs(projection);
                    }



                    for (int idx_t = 0; idx_t < msg_center.size(); idx_t++)
                    {
                        msg_center[idx_t] = center[idx_t];
                    }

                    std::vector<double> sortedEdgesLength(edgesLength.size(), 0);
                    std::vector<Eigen::Vector3f> sortedEdgesDirection(edgesDirection.size());
                    sortEdges(edgesLength, sortedEdgesLength, edgesDirection, sortedEdgesDirection);


                    msg_longestEdgeLen = sortedEdgesLength[0];
                    msg_medianEdgeLen = sortedEdgesLength[1];
                    msg_shortestEdgeLen = sortedEdgesLength[2];



                    storeEdgesDir(sortedEdgesDirection, msg_longestEdgeDir, msg_medianEdgeDir, msg_shortestEdgeDir);

                    segmentation::box_3D_data msg;
                    msg.center = msg_center;
                    msg.longestEdgeLen = msg_longestEdgeLen;
                    msg.medianEdgeLen = msg_medianEdgeLen;
                    msg.shortestEdgeLen = msg_shortestEdgeLen;
                    msg.longestEdgeDir = msg_longestEdgeDir;
                    msg.medianEdgeDir = msg_medianEdgeDir;
                    msg.shortestEdgeDir = msg_shortestEdgeDir;
                    box_pub_.publish(msg);

                    viewPerpenPlanes(cloud_planes, planeIndices);

                    viewer->removePointCloud("table cloud");
                    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_table, "table cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "table cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "table cloud");


                }


            }
            else
            {
                std::cerr << "Find 2 perpendicular planes..." << std::endl;
                bool findTwoPerpenPlanes = false;
                std::vector<int> twoPlaneIndices(2, 0);

                if (numOfPlanes >= 2)
                {
                    for (int i = 0; (i < numOfPlanes - 1) && !findTwoPerpenPlanes; i++)
                    {
                        for (int j = i + 1; (j < numOfPlanes ) && !findTwoPerpenPlanes; j++)
                        {

                            Eigen::Vector3f plane1Normal(coefficients_planes[i]->values[0], coefficients_planes[i]->values[1], coefficients_planes[i]->values[2]);
                            Eigen::Vector3f plane2Normal(coefficients_planes[j]->values[0], coefficients_planes[j]->values[1], coefficients_planes[j]->values[2]);
                            double normalProductThresh = 0.2;
                            if (fabs(plane1Normal.dot(plane2Normal)) < normalProductThresh)
                            {
                                findTwoPerpenPlanes = true;
                                twoPlaneIndices[0] = i;
                                twoPlaneIndices[1] = j;
                            }

                        }
                    }
                }


                std::vector<Eigen::Vector3f> twofaceCenters;
                std::vector<Eigen::Vector3f> corners;
                getPointsOnFace(cloud_planes, coefficients_planes, twoPlaneIndices, twofaceCenters, corners);

                double minDistance = 1000000;

                // the common corner point (either one of two) of these two faces
                // commonPointIdx1 is the index of this point on the first plane
                // commonPointIdx2 is the index of this point on the second plane
                int commonPointIdx1 = 0;
                int commonPointIdx2 = 0;
                for (int idx_t = 0; idx_t < 4; idx_t++)
                {
                    Eigen::Vector3f pointOne;
                    pointOne = corners[idx_t];
                    for (int idx_w = 4; idx_w < 8; idx_w++)
                    {
                        Eigen::Vector3f pointTwo;
                        pointTwo = corners[idx_w];
                        Eigen::Vector3f diff = pointTwo - pointOne;
                        double distance = diff.norm();
                        if (distance < minDistance)
                        {
                            commonPointIdx1 = idx_t;
                            commonPointIdx2 = idx_w;
                            minDistance = distance;
                        }
                    }
                }
                Eigen::Vector3f commonPoint = (corners[commonPointIdx1] + corners[commonPointIdx2]) / 2.0;
                Eigen::Vector3f plane1Normal(coefficients_planes[twoPlaneIndices[0]]->values[0], coefficients_planes[twoPlaneIndices[0]]->values[1], coefficients_planes[twoPlaneIndices[0]]->values[2]);
                Eigen::Vector3f plane2Normal(coefficients_planes[twoPlaneIndices[1]]->values[0], coefficients_planes[twoPlaneIndices[1]]->values[1], coefficients_planes[twoPlaneIndices[1]]->values[2]);
                Eigen::Vector3f lineOfIntersection = plane1Normal.cross(plane2Normal);


                std::vector<double> edgesLength;
                std::vector<Eigen::Vector3f> edgesDirection;

                Eigen::Vector3f center;
                center = commonPoint;
                for (int w = 0; w < twoPlaneIndices.size(); w++)
                {
                    int planeIndex = 1 - w;
                    Eigen::Vector3f normal_temp(coefficients_planes[twoPlaneIndices[planeIndex]]->values[0],
                                                coefficients_planes[twoPlaneIndices[planeIndex]]->values[1],
                                                coefficients_planes[twoPlaneIndices[planeIndex]]->values[2]);

                    Eigen::Vector3f faceDiagonal;
                    faceDiagonal = twofaceCenters[w] - commonPoint;
                    double projection = faceDiagonal.dot(normal_temp);
                    edgesLength.push_back(fabs(projection) * 2.0);
                    int dir = projection >= 0 ? 1 : -1;
                    edgesDirection.push_back(normal_temp * dir);

                    center += edgesDirection.back() * fabs(projection);
                }

                Eigen::Vector3f faceDiagonal;
                faceDiagonal = twofaceCenters[0] - commonPoint;
                double projection = faceDiagonal.dot(lineOfIntersection);
                edgesLength.push_back(fabs(projection) * 2.0);
                int dir = projection >= 0 ? 1 : -1;
                edgesDirection.push_back(lineOfIntersection * dir);
                center += edgesDirection.back() * fabs(projection);

                for (int idx_t = 0; idx_t < msg_center.size(); idx_t++)
                {
                    msg_center[idx_t] = center[idx_t];
                }

                std::vector<double> sortedEdgesLength(edgesLength.size(), 0);
                std::vector<Eigen::Vector3f> sortedEdgesDirection(edgesDirection.size());

                sortEdges(edgesLength, sortedEdgesLength, edgesDirection, sortedEdgesDirection);

                msg_longestEdgeLen = sortedEdgesLength[0];
                msg_medianEdgeLen = sortedEdgesLength[1];
                msg_shortestEdgeLen = sortedEdgesLength[2];



                storeEdgesDir(sortedEdgesDirection, msg_longestEdgeDir, msg_medianEdgeDir, msg_shortestEdgeDir);

                segmentation::box_3D_data msg;
                msg.center = msg_center;
                msg.longestEdgeLen = msg_longestEdgeLen;
                msg.medianEdgeLen = msg_medianEdgeLen;
                msg.shortestEdgeLen = msg_shortestEdgeLen;
                msg.longestEdgeDir = msg_longestEdgeDir;
                msg.medianEdgeDir = msg_medianEdgeDir;
                msg.shortestEdgeDir = msg_shortestEdgeDir;
                box_pub_.publish(msg);

                viewPerpenPlanes(cloud_planes, twoPlaneIndices);
                viewer->removePointCloud("table cloud");
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_table, "table cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "table cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "table cloud");


            }
        }


    }

    void sortEdges(std::vector<double> &edgesLength, std::vector<double> &sortedEdgesLength, std::vector<Eigen::Vector3f> &edgesDirection, std::vector<Eigen::Vector3f> &sortedEdgesDirection)
    {
        std::vector<int> indices(edgesLength.size(), 0);
        for (int idx_t = 0; idx_t < indices.size(); idx_t++)
        {
            indices[idx_t] = idx_t;
        }

        std::sort(indices.begin(), indices.end(), IdxCompare(edgesLength));

        for (int idx_t = 0; idx_t < indices.size(); idx_t++)
        {
            sortedEdgesLength[idx_t] = edgesLength[indices[idx_t]];
            sortedEdgesDirection[idx_t] = edgesDirection[indices[idx_t]];
        }
    }

    void viewPerpenPlanes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_planes, std::vector<int> &planeIndices)
    {
        for (int idx_t = 0; idx_t < planeIndices.size(); idx_t++)
        {
            std::string str = "plane cloud ";
            str.append(boost::lexical_cast<std::string>(idx_t));
            if (cmdOutput) std::cerr << "plane name:" << str << std::endl;
            if (cmdOutput) std::cerr << "   points size:" << cloud_planes[planeIndices[idx_t]]->points.size() << std::endl;
            viewer->removePointCloud(str);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_planes[planeIndices[idx_t]], str);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);

            double r = colors[idx_t][0];
            double g = colors[idx_t][1];
            double b = colors[idx_t][2];

            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r , g, b, str);
        }
    }

    void getPointsOnFace(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_planes, std::vector<pcl::ModelCoefficients::Ptr> &coefficients_planes,
                         std::vector<int> planeIndices, std::vector<Eigen::Vector3f> &faceCenters, std::vector<Eigen::Vector3f> &corners)
    {
        for (int planeIdx = 0; planeIdx < planeIndices.size(); planeIdx++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_projected_flat(new pcl::PointCloud<pcl::PointXYZRGB>);
            Eigen::Matrix3f inv_plane_rotation;
            projectToPlaneNormalToZAxis(cloud_planes[planeIndices[planeIdx]], coefficients_planes[planeIndices[planeIdx]], plane_projected, plane_projected_flat, inv_plane_rotation);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
            convex_hull.setInputCloud(plane_projected_flat);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*cloud_hull);

            double min_area = 10000000;
            Eigen::Vector3f faceCenter(0, 0, 0);
            Eigen::Vector3f corner1(0, 0, 0);
            Eigen::Vector3f corner2(0, 0, 0);
            Eigen::Vector3f corner3(0, 0, 0);
            Eigen::Vector3f corner4(0, 0, 0);


            // if (cmdOutput) std::cerr << "Cloud Hull Size: " << cloud_hull->size() << std::endl;
            for (int hullIdx = 0; hullIdx < cloud_hull->size(); hullIdx++)
            {

                Eigen::Matrix3f inv_rotation;
                double x_min = 1000.0;
                double x_max = -1000.0;
                double y_min = 1000.0;
                double y_max = -1000.0;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                rotateEdgeToBeXAxis(hullIdx, cloud_hull, x_min, x_max, y_min, y_max, inv_rotation, projected_cloud);

                double area = (x_max - x_min) * (y_max - y_min);

                if (area  < min_area)
                {
                    Eigen::Matrix3f transformation = inv_plane_rotation * inv_rotation;

                    faceCenter(0) = (x_max + x_min) / 2.0;
                    faceCenter(1) = (y_max + y_min) / 2.0;
                    faceCenter(2) = projected_cloud->points[0].z;

                    corner1(0) = x_min;
                    corner1(1) = y_min;
                    corner1(2) = projected_cloud->points[0].z;

                    corner2(0) = x_min;
                    corner2(1) = y_max;
                    corner2(2) = projected_cloud->points[0].z;

                    corner3(0) = x_max;
                    corner3(1) = y_min;
                    corner3(2) = projected_cloud->points[0].z;

                    corner4(0) = x_max;
                    corner4(1) = y_max;
                    corner4(2) = projected_cloud->points[0].z;




                    faceCenter = transformation * faceCenter;
                    corner1 = transformation * corner1;
                    corner2 = transformation * corner2;
                    corner3 = transformation * corner3;
                    corner4 = transformation * corner4;

                    min_area = area;
                }
            }
            faceCenters.push_back(faceCenter);
            corners.push_back(corner1);
            corners.push_back(corner2);
            corners.push_back(corner3);
            corners.push_back(corner4);
        }
    }

    void storeEdgesDir(std::vector<Eigen::Vector3f> &sortedEdgesDir, std::vector<double> &msg_longestEdgeDir, std::vector<double> &msg_medianEdgeDir, std::vector<double> &msg_shortestEdgeDir)
    {

        for (int idx_t = 0; idx_t < sortedEdgesDir.size(); idx_t++)
        {
            Eigen::Vector3f edge;
            edge = sortedEdgesDir[0];
            msg_longestEdgeDir[idx_t] = edge(idx_t);
        }

        for (int idx_t = 0; idx_t < sortedEdgesDir.size(); idx_t++)
        {
            Eigen::Vector3f edge;
            edge = sortedEdgesDir[1];
            msg_medianEdgeDir[idx_t] = edge(idx_t);
        }

        for (int idx_t = 0; idx_t < sortedEdgesDir.size(); idx_t++)
        {
            Eigen::Vector3f edge;
            edge = sortedEdgesDir[2];
            msg_shortestEdgeDir[idx_t] = edge(idx_t);
        }
    }

    void getPointsOnFace(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_planes, std::vector<pcl::ModelCoefficients::Ptr> &coefficients_planes,
                         std::vector<int> planeIndices, std::vector<Eigen::Vector3f> &faceCenters)
    {
        for (int planeIdx = 0; planeIdx < planeIndices.size(); planeIdx++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_projected_flat(new pcl::PointCloud<pcl::PointXYZRGB>);
            Eigen::Matrix3f inv_plane_rotation;
            projectToPlaneNormalToZAxis(cloud_planes[planeIndices[planeIdx]], coefficients_planes[planeIndices[planeIdx]], plane_projected, plane_projected_flat, inv_plane_rotation);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
            convex_hull.setInputCloud(plane_projected_flat);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*cloud_hull);

            double min_area = 10000000;
            Eigen::Vector3f faceCenter(0, 0, 0);


            // if (cmdOutput) std::cerr << "Cloud Hull Size: " << cloud_hull->size() << std::endl;
            for (int hullIdx = 0; hullIdx < cloud_hull->size(); hullIdx++)
            {

                Eigen::Matrix3f inv_rotation;
                double x_min = 1000.0;
                double x_max = -1000.0;
                double y_min = 1000.0;
                double y_max = -1000.0;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                rotateEdgeToBeXAxis(hullIdx, cloud_hull, x_min, x_max, y_min, y_max, inv_rotation, projected_cloud);

                double area = (x_max - x_min) * (y_max - y_min);

                if (area  < min_area)
                {
                    Eigen::Matrix3f transformation = inv_plane_rotation * inv_rotation;

                    faceCenter(0) = (x_max + x_min) / 2.0;
                    faceCenter(1) = (y_max + y_min) / 2.0;
                    faceCenter(2) = projected_cloud->points[0].z;


                    faceCenter = transformation * faceCenter;
                    min_area = area;
                }
            }
            faceCenters.push_back(faceCenter);
        }
    }




    void projectToPlaneNormalToZAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_plane, pcl::ModelCoefficients::Ptr &target_plane_coeffi,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_projected,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_projected_flat, Eigen::Matrix3f &inv_plane_rotation)
    {

        // Create the filtering object, Project object into 2d, using plane model coefficients
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud_plane);
        proj.setModelCoefficients (target_plane_coeffi);
        proj.filter (*cloud_projected);

        Eigen::Vector3f plane_normal(target_plane_coeffi->values[0], target_plane_coeffi->values[1], target_plane_coeffi->values[2]);

        // Rotate plane so that Z=0
        Eigen::Quaternionf qz;
        qz.setFromTwoVectors(plane_normal, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f plane_rotation = qz.toRotationMatrix();
        inv_plane_rotation = plane_rotation.inverse();

        for (int i = 0; i < cloud_projected->size(); i++)
        {
            pcl::PointXYZRGB p;
            p.getVector3fMap() = plane_rotation * cloud_projected->points[i].getVector3fMap();
            cloud_projected_flat->push_back(p);
        }

    }

    void rotateEdgeToBeXAxis(int idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hull, double &x_min, double &x_max, double &y_min, double &y_max,
                             Eigen::Matrix3f &inv_rotation, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &projected_cloud)
    {
        // For each pair of hull points, determine the angle
        double delta_y = cloud_hull->points[(idx + 1) % cloud_hull->size()].y - cloud_hull->points[idx].y;
        double delta_x = cloud_hull->points[(idx + 1) % cloud_hull->size()].x - cloud_hull->points[idx].x;

        double delta_l = sqrt((delta_y  * delta_y ) + (delta_x * delta_x));
        double sin_theta = delta_y / delta_l;
        double cos_theta = delta_x / delta_l;


        // Build rotation matrix from change of basis
        Eigen::Matrix3f rotation;
        rotation(0, 0) = cos_theta;
        rotation(0, 1) = sin_theta;  // -theta, not theta
        rotation(0, 2) = 0.0;
        rotation(1, 0) = -sin_theta;
        rotation(1, 1) = cos_theta;
        rotation(1, 2) = 0.0;
        rotation(2, 0) = 0.0;
        rotation(2, 1) = 0.0;
        rotation(2, 2) = 1.0;

        inv_rotation = rotation.inverse();

        // Project hull to new coordinate system
        // And compute min/max

        for (int j = 0; j < cloud_hull->size(); j++)
        {
            pcl::PointXYZRGB p;
            p.getVector3fMap() = rotation * cloud_hull->points[j].getVector3fMap();
            projected_cloud->push_back(p);
            if (p.x < x_min) x_min = p.x;
            if (p.x > x_max) x_max = p.x;

            if (p.y < y_min) y_min = p.y;
            if (p.y > y_max) y_max = p.y;
        }
    }

    void generateIndices(pcl::PointIndices &pointIndices, cv::Point &topleft, int width, int height)
    {
        pointIndices.indices.clear();
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                int column = topleft.x + i;
                int row = topleft.y + j;
                int index = row * imageWidth + column;
                pointIndices.indices.push_back(index);
            }
        }
    }

    bool isParallelToTable(pcl::ModelCoefficients::Ptr table, pcl::ModelCoefficients::Ptr plane, double &height)
    {
        // check if the plane is parallel to the table; if it is, check again whether the plane is just the table plane
        double distanceToTable = 0;
        Eigen::Vector3f table_normal(table->values[0], table->values[1], table->values[2]);
        Eigen::Vector3f plane_normal(plane->values[0], plane->values[1], plane->values[2]);
        float angle = acos(table_normal.dot(plane_normal)) / M_PI * 180;
        bool flag = false;
        float tolerance = 10;
        if (fabs(angle) < tolerance || fabs(angle - 180) < tolerance)
        {
            flag = true;
            double d1 = table->values[3];
            double d2 = plane->values[3];
            int sign_count = table_normal[0] * plane_normal[0] > 0;
            sign_count += table_normal[1] * plane_normal[1] > 0 ;
            sign_count += table_normal[2] * plane_normal[2] > 0;
            if (sign_count > 0)
            {
                distanceToTable = fabs(d1 - d2) / table_normal.norm();
            }
            else
            {
                distanceToTable = fabs(d1 + d2) / table_normal.norm();
            }
            height = distanceToTable;

            if (distanceToTable < 0.02)
            {
                flag = false;
            }
        }
        return flag;

    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_seg");
    ros::NodeHandle nh;
    boxSegmentation cs(nh);
    ros::spin();
}

