#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d.hpp>
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    // Read image
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

    // Convert the image to grayscale
    Mat gray;
    cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

  
    // Filter by Area.
    params.filterByArea = false;
    //params.minArea = 10;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;

    // Filter by Convexity
    params.filterByConvexity = false;
    // params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = false;
    // params.minInertiaRatio = 0.7;


    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);   

    // Detect blobs
    std::vector<KeyPoint> keypoints;
    detector->detect(gray, keypoints);


    // Draw blobs
    Mat im_with_keypoints;
    drawKeypoints(gray, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
 
 
    cv_ptr->image=im_with_keypoints;
  
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}