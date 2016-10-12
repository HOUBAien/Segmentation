
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

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <segmentation/box_data.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

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
        box_pub_ = nh.advertise<segmentation::box_data>("box_data", 10);

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
        // ROS_INFO("Enter Cloud CB");
        if (!rectDone)
        {
            return;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_original_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_copy(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_box_top (new pcl::PointCloud<pcl::PointXYZRGB> ());
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

        pcl::ModelCoefficients::Ptr coefficients_box_top (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_box_top (new pcl::PointIndices);
        bool mayHavePlane = true;
        bool havePlaneParallelToTable = false;
        double height = 0;
        do
        {
            // seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
            seg.setNormalDistanceWeight (0.1);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);
            seg.setInputCloud (cloud_filtered_copy);
            seg.setInputNormals (cloud_normals_copy);


            seg.segment (*inliers_box_top, *coefficients_box_top);


            extract.setInputCloud(cloud_filtered_copy);
            extract.setIndices(inliers_box_top);
            extract.setNegative(false);
            extract.filter(*cloud_box_top);

            if (cloud_box_top->points.size() > 500)
            {


                if (isParallelToTable(coefficients_table, coefficients_box_top, height))
                {
                    if(cmdOutput) std::cerr << "PointCloud representing the box top planar component:" << cloud_box_top->points.size() << " data points." << std::endl;
                    mayHavePlane = false;
                    havePlaneParallelToTable = true;
                }
                else
                {
                    extract.setNegative(true);
                    extract.filter(*cloud_filtered_copy);
                    extract_normals.setNegative (true);
                    extract_normals.setInputCloud (cloud_normals_copy);
                    extract_normals.setIndices (inliers_box_top);
                    extract_normals.filter (*cloud_normals_copy);
                }
            }
            else
            {
                std::cerr << "Cannot find plane which is parallel to the table plane" << std::endl;
                mayHavePlane = false;
            }

        } while (mayHavePlane);
        // // http://docs.pointclouds.org/trunk/group__sample__consensus.html
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);

        // seg.setAxis(table_normal);
        // seg.setEpsAngle(M_PI * 30.0 / 180.0);
        // seg.setNormalDistanceWeight(0.1);
        // seg.setMaxIterations(1000);
        // seg.setDistanceThreshold(0.03);
        // seg.setInputCloud(cloud_filtered);
        // seg.setInputNormals(cloud_normals);

        // // http://docs.pointclouds.org/1.7.2/a01146.html
        // pcl::ModelCoefficients::Ptr coefficients_box_top (new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers_box_top (new pcl::PointIndices);
        // seg.segment(*inliers_box_top, *coefficients_box_top);
        std::vector<double> rect_data(9, 0);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (havePlaneParallelToTable)
        {
            if (cmdOutput) std::cerr << "Box Top Face Plane coefficients: " << *coefficients_box_top << std::endl;
            // if (cmdOutput) std::cerr << "Height: " << height<<std::endl;
            // if (cmdOutput) std::cerr << "PointCloud representing the box top planar component: " << cloud_box_top->points.size () << " data points." << std::endl;

            // Create the filtering object, Project object into 2d, using plane model coefficients
            pcl::ProjectInliers<pcl::PointXYZRGB> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud_box_top);
            proj.setModelCoefficients (coefficients_table);
            proj.filter (*cloud_projected);

            // Rotate plane so that Z=0
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_flat(new pcl::PointCloud<pcl::PointXYZRGB>);

            Eigen::Quaternionf qz;
            qz.setFromTwoVectors(table_normal, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f plane_rotation = qz.toRotationMatrix();
            Eigen::Matrix3f inv_plane_rotation = plane_rotation.inverse();

            for (int i = 0; i < cloud_projected->size(); i++)
            {
                pcl::PointXYZRGB p;
                p.getVector3fMap() = plane_rotation * cloud_projected->points[i].getVector3fMap();
                cloud_projected_flat->push_back(p);
            }

            // Find the convex hull

            pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
            convex_hull.setInputCloud(cloud_projected_flat);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*cloud_hull);


            double min_area = 10000000;

            if (cmdOutput) std::cerr << "Cloud Hull Size: " << cloud_hull->size() << std::endl;
            for (int i = 0; i < cloud_hull->size(); i++)
            {
                // For each pair of hull points, determine the angle
                double delta_y = cloud_hull->points[(i + 1) % cloud_hull->size()].y - cloud_hull->points[i].y;
                double delta_x = cloud_hull->points[(i + 1) % cloud_hull->size()].x - cloud_hull->points[i].x;

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

                Eigen::Matrix3f inv_rotation = rotation.inverse();

                // Project hull to new coordinate system
                // And compute min/max
                double x_min = 1000.0;
                double x_max = -1000.0;
                double y_min = 1000.0;
                double y_max = -1000.0;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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

                double area = (x_max - x_min) * (y_max - y_min);

                if (area  < min_area)
                {
                    Eigen::Matrix3f transformation = inv_plane_rotation * inv_rotation;
                    Eigen::Vector3f long_edge(1, 0, 0);

                    rect_data[0] = (x_max - x_min);
                    rect_data[1] = (y_max - y_min);
                    rect_data[2] = height;     // height

                    // rect_data[0] represents the length (the longer edge of the bottom face)
                    // rect_data[1] represents the width
                    // rect_data[2] represents the height
                    if (rect_data[0] < rect_data[1])
                    {
                        double temp = rect_data[0];
                        rect_data[0] = rect_data[1];
                        rect_data[1] = temp;
                        if (fabs(rect_data[0] - rect_data[1]) > 0.02)
                        {
                            long_edge(0) = 0;
                            long_edge(1) = 1;
                            long_edge(2) = 0;
                        }
                    }
                    // std::cout<<"======================================================================"<<std::endl;
                    // std::cout<<"Long edge: "<<long_edge<<std::endl;
                    // std::cout<<"rect_data: "<<std::endl;
                    // std::cout<<rect_data[0]<<std::endl;
                    // std::cout<<rect_data[1]<<std::endl;
                    // std::cout<<rect_data[2]<<std::endl;
                    // std::cout<<"==========================================="<<std::endl;
                    Eigen::Vector3f center((x_max + x_min) / 2.0, (y_max + y_min) / 2.0, projected_cloud->points[0].z + height / 2.0);
                    center = transformation * center;
                    long_edge = transformation * long_edge;
                    rect_data[3] = center(0);
                    rect_data[4] = center(1);
                    rect_data[5] = center(2);
                    rect_data[6] = long_edge(0);
                    rect_data[7] = long_edge(1);
                    rect_data[8] = long_edge(2);

                    min_area = area;
                }
            }
            segmentation::box_data msg;
            msg.box_data = rect_data;
            box_pub_.publish(msg);
            if (viewDisplay)
            {
                for (int i = 0; i < cloud_hull->size(); i++)
                {
                    cloud_hull->points[i].getVector3fMap() = inv_plane_rotation * cloud_hull->points[i].getVector3fMap();
                }
            }
        }

        if (viewDisplay)
        {
            viewer->removePointCloud("cloud_filtered");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "cloud_filtered");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");

            viewer->removePointCloud("plane cloud");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_table, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0 , 1.0, 0.0, "plane cloud");
            // if (havePlaneParallelToTable)
            // {
            viewer->removePointCloud("box top cloud");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_box_top, "box top cloud");
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
            // }
            // else
            // {
            //     std::cerr << "Can't find the box component." << std::endl;
            // }

            if (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }

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
    bool isParallelToTable(pcl::ModelCoefficients::Ptr table, pcl::ModelCoefficients::Ptr plane, double& height)
    {
        Eigen::Vector3f table_normal(table->values[0], table->values[1], table->values[2]);
        Eigen::Vector3f plane_normal(plane->values[0], plane->values[1], plane->values[2]);
        float angle = acos(table_normal.dot(plane_normal)) / M_PI * 180;
        bool flag = false;
        float tolerance = 20;
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
                height = fabs(d1 - d2) / table_normal.norm();
            }
            else
            {
                height = fabs(d1 + d2) / table_normal.norm();
            }

            if (height < 0.02)
            {
                flag = false;
            }
        }
        return flag;

    }

};


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "box_seg");
    ros::NodeHandle nh;
    boxSegmentation cs(nh);
    ros::spin();
}

