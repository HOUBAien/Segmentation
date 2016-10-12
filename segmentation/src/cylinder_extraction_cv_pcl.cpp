#include <ros/ros.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <segmentation/cylinder_data.h>
#include <Eigen/Dense>

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
class cylinderExtraction
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher cylinder_pub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  bool viewDisplay;
  bool cmdOutput;
  std::string cvwindowName;
  int imageWidth;
  int imageHeight;
public:
  cv::Mat rgbImage_;

  cylinderExtraction() :
    it_(nh_)
  {

    if (!nh_.getParam("viewDisplay", viewDisplay))
    {
      viewDisplay = true;
    }
    if (!nh_.getParam("cmdOutput", cmdOutput))
    {
      cmdOutput = true;
    }

    if (viewDisplay)
    {
      viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer2.reset(new pcl::visualization::PCLVisualizer("3D Viewer Whole"));
    }


    imageWidth = 640;
    imageHeight = 480;
    cvwindowName = "RGB Image window";
    cv::namedWindow(cvwindowName);
    cv::setMouseCallback(cvwindowName, mouseHandle, (void*)&rgbImage_);
    image_sub_rgb_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &cylinderExtraction::rgbCb, this);
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &cylinderExtraction::pointCloudCb, this);
    cylinder_pub_ = nh_.advertise<segmentation::cylinder_data>("cylinder_data", 10);
  }

  ~cylinderExtraction()
  {
    cv::destroyWindow(cvwindowName);
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
  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &input)
  {
        if (!rectDone)
    {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);





    cv::Point topLeft = rectangle_.tl();
    // cv::Point topLeft(10,10);
    int width = rectangle_.width;
    int height = rectangle_.height;
    // int width = 600;
    // int height = 400;
    pcl::PointIndices::Ptr roiIndices(new pcl::PointIndices);
    generateIndices(*roiIndices, topLeft, width, height);

    if (viewDisplay)
    {
      viewer2->removePointCloud("All Points");
      viewer2->addPointCloud<pcl::PointXYZ>(cloud, "All Points");
      viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "All Points");
      viewer2->spinOnce();
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(roiIndices);
    extract.setNegative(false);
    extract.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    // Build a passthrough filter to remove spurious NaNs
    if (cmdOutput)
      std::cerr << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.0);
    pass.filter(*cloud_filtered);
    if (cmdOutput)
      std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;


    // Estimate point normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    if (cmdOutput)
      std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    if (cmdOutput)
      std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
                << std::endl;

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    // http://docs.pointclouds.org/1.7.2/a01146.html
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    if (cmdOutput)
      std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
    // viewer->removePointCloud("sample cloud");
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->spinOnce();

    if (viewDisplay)
    {
      viewer->removePointCloud("sample cloud");
      viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

      viewer->removePointCloud("plane cloud");
      viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "plane cloud");

      //ROS_INFO("Cloud Viewer Entered");
      if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
      else
      {
        if (cmdOutput)
          std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size()
                    << " data points." << std::endl;
        viewer->removePointCloud("cylinder cloud");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "cylinder cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cylinder cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
            "cylinder cloud");
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

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cylinder_extraction");
  cylinderExtraction ce;
  ros::spin();
}

