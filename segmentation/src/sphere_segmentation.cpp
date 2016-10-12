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
#include <segmentation/sphere_data.h>
#include <Eigen/Dense>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>

#if !defined(__SUNPRO_CC) || (__SUNPRO_CC > 0x530)
#include <boost/generator_iterator.hpp>
#endif

const int colors_size = 10;
const double colors[colors_size][3] = {
  {0.0, 1.0, 0.0},
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
class sphereExtraction
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher sphere_pub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  bool viewDisplay;
  bool cmdOutput;
  std::string cvwindowName;
  int imageWidth;
  int imageHeight;
public:
  cv::Mat rgbImage_;

  sphereExtraction() :
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
    image_sub_rgb_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &sphereExtraction::rgbCb, this);
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &sphereExtraction::pointCloudCb, this);
    sphere_pub_ = nh_.advertise<segmentation::sphere_data>("sphere_data", 10);
  }

  ~sphereExtraction()
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

    int count = 0;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_planes;
    int plane_point_size_thresh = 3000;
    while (true)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
      // seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      // seg.setNormalDistanceWeight(0.1);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(10000);
      seg.setDistanceThreshold(0.005);
      seg.setInputCloud(cloud_filtered);
      seg.setInputNormals(cloud_normals);
      // Obtain the plane inliers and coefficients
      seg.segment(*inliers_plane, *coefficients_plane);
      if (cmdOutput)
        std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;


      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers_plane);
      extract.setNegative(false);
      extract.filter(*cloud_plane);
      if (cloud_plane->points.size() > plane_point_size_thresh)
      {
        if (cmdOutput)
          std::cerr << "PointCloud representing the planar component [" << count++ << "] : " << cloud_plane->points.size() << " data points."
                    << std::endl;

        cloud_planes.push_back(cloud_plane);
        // Remove the planar inliers, extract the rest
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        extract_normals.setNegative(true);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.filter(*cloud_normals);
      }
      else
      {
        break;
      }

    }

    // Create the segmentation object for sphere segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.2);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the sphere inliers and coefficients
    // http://docs.pointclouds.org/1.7.2/a01146.html
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    seg.segment(*inliers_sphere, *coefficients_sphere);

    if (cmdOutput)
      std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_sphere);
    extract.setNegative(false);
    extract.filter(*cloud_sphere);

    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
    // viewer->removePointCloud("sample cloud");
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->spinOnce();

    if (viewDisplay)
    {
      viewer->removePointCloud("all cloud");
      viewer->addPointCloud<pcl::PointXYZ>(cloud, "all cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all cloud");

      typedef boost::minstd_rand base_generator_type;
      base_generator_type generator(6u);
      for (int i = 0; i < cloud_planes.size(); i++)
      {
        std::string cloud_plane_name = "plane cloud " + boost::lexical_cast<std::string>(i);

        double r;
        double g;
        double b;
        // if (i < colors_size)
        // {
        //   r = colors[i][0];
        //   g = colors[i][1];
        //   b = colors[i][2];
        // }
        // else
        // {
        //   boost::uniform_real<> uni_dist(0, 1);
        //   boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);
        //   r = uni();
        //   g = uni();
        //   b = uni();
        // }
        r = 0.0;
        g = 1.0;
        b = 0.0;
        viewer->removePointCloud(cloud_plane_name);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_planes[i], cloud_plane_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_plane_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloud_plane_name);
      }

      if (cloud_sphere->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
      else
      {
        if (cmdOutput)
          std::cerr << "PointCloud representing the cylindrical component: " << cloud_sphere->points.size()
                    << " data points." << std::endl;
        viewer->removePointCloud("sphere cloud");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_sphere, "sphere cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sphere cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
            "sphere cloud");
      }
      if (!viewer->wasStopped())
      {
        viewer->spinOnce();
      }

    }

    if (!cloud_sphere->points.empty ())
    {
      Eigen::Vector3d pseudo_center(0, 0, 0);

      int num_points_in_sphere = cloud_sphere->points.size();
      for (int i = 0; i < num_points_in_sphere; i++)
      {
        pseudo_center(0) += cloud_sphere->points[i].x;
        pseudo_center(1) += cloud_sphere->points[i].y;
        pseudo_center(2) += cloud_sphere->points[i].z;
      }
      pseudo_center /= num_points_in_sphere;

      Eigen::Vector3d central_axis(coefficients_sphere->values[3], coefficients_sphere->values[4], coefficients_sphere->values[5]);

      // Project the pseduo_center onto the central_axis
      // https://en.wikipedia.org/wiki/Vector_projection
      Eigen::Vector3d point_on_central_axis(coefficients_sphere->values[0], coefficients_sphere->values[1], coefficients_sphere->values[2]);
      Eigen::Vector3d pseudo_center_vector = pseudo_center - point_on_central_axis;
      Eigen::Vector3d pseudo_center_vector_projection = pseudo_center_vector.dot(central_axis) / central_axis.dot(central_axis) * central_axis;
      Eigen::Vector3d center = pseudo_center_vector_projection + point_on_central_axis;

      std::vector<double> data(4, 0);
      for (int i = 0; i < data.size(); i++)
      {
        data[i] = coefficients_sphere->values[i];
      }


      segmentation::sphere_data msg;
      msg.sphere_data = data;
      sphere_pub_.publish(msg);
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
  ros::init(argc, argv, "sphere_extraction");
  sphereExtraction ce;
  ros::spin();
}

