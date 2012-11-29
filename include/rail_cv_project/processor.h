/*!
 * \file processor.h
 * \brief Main image processing class for toy computer object recognition.
 *
 * The processor class reads in streams from the PR2's onboard cameras and attempts to detect the
 * toy computer via multiple steps. Additional information is available in the project's writeup.
 *
 * \author Russell Toris, David Kent, Adrian‎ Boteanu
 * \date November 28, 2012
 */

#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

/*!
 * \struct point
 * A point in 2D space with a counter.
 */
typedef struct
{
  int c; /*!< the count */
  cv::RotatedRect rect; /*!< the bounding box */
} smoothing_point;

/*!
 * \def RATIO_THRESHOLD
 * Ratio cutoff of the bounding box length and width used when filtering contours.
 */
#define RATIO_THRESHOLD 0.7
/*!
 * \def MIN_AREA
 * Minimum area cutoff of the bounding box length and width used when filtering contours.
 */
#define MIN_AREA 1000
/*!
 * \def MAX_AREA
 * Maximum area cutoff of the bounding box length and width used when filtering contours.
 */
#define MAX_AREA 7000

/*!
 * \def SMOOTH_DIST_THRESH
 * Distance threshold used for smoothing.
 */
#define SMOOTH_DIST_THRESH 5
/*!
 * \def MAX_SMOOTHING_COUNT
 * Max count used for smoothing.
 */
#define MAX_SMOOTHING_COUNT 20

/*!
 * \def BLUR_KERNEL
 * Kernel size for blur filter.
 */
#define BLUR_KERNEL 5

/*!
 * \def CANNY_MIN_THRESH
 * Minimum gradient threshold for Canny.
 */
#define CANNY_MIN_THRESH 40
/*!
 * \def CANNY_MAX_THRESH
 * Maximum gradient threshold for Canny.
 */
#define CANNY_MAX_THRESH 120
/*!
 * \def CANNY_KERNEL
 * Kernel size for Canny.
 */
#define CANNY_KERNEL 3

/*!
 * \def ERODE_ELEMENT_SIZE
 * Element size used when we erode the image.
 */
#define ERODE_ELEMENT_SIZE 5
/*!
 * \def DILATE_ELEMENT_SIZE
 * Element size used when we dilate the image.
 */
#define DILATE_ELEMENT_SIZE 3

/*!
 * \def HOUGH_RHO
 * Distance resolution of the Hough transforms accumulator.
 */
#define HOUGH_RHO 1
/*!
 * \def HOUGH_THETA
 * Angle resolution of the Hough transforms accumulator.
 */
#define HOUGH_THETA CV_PI/180.0
/*!
 * \def HOUGH_THRESH
 * Only lines with enough votes will be counted as a line using this as the threshold.
 */
#define HOUGH_THRESH 12
/*!
 * \def HOUGH_MIN_LINE_LENGTH
 * Minimum length of a line that will be counted as a line.
 */
#define HOUGH_MIN_LINE_LENGTH 8
/*!
 * \def HOUGH_MAX_LINE_GAP
 * Maximum gap between two lines to allow for merging lines.
 */
#define HOUGH_MAX_LINE_GAP 3

/*!
 * \def REF_WIDTH
 * Reference rectangle width used in rectangle matching in the feature vector.
 */
#define REF_WIDTH 20
/*!
 * \def REF_HEIGHT
 * Reference rectangle height used in rectangle matching in the feature vector.
 */
#define REF_HEIGHT 28

/*!
 * \def MAX_SIFT
 * SIFT features cutoff size used for scaling in the feature vector.
 */
#define MAX_SIFT 20.0

/*!
 * \def MAX_LINES
 * Maximum number of lines to bound by in an image for the feature vector.
 */
#define MAX_LINES 150.0
/*!
 * \def MAX_AVG_LINE_LENGTH
 * Maximum average line length to bound by in an image for the feature vector.
 */
#define MAX_AVG_LINE_LENGTH 25.0

/*!
 * \def D
 * Dimensionality of the feature vector.
 */
#define D 7

/*!
 * \def COMPUTER
 * Integer label for the toy computer.
 */
#define COMPUTER 0
/*!
 * \def NUM_COMPUTER
 * Number of computer training points for the classifier.
 */
#define NUM_COMPUTER 36
/*!
 * \def TURTLE
 * Integer label for the toy turtle.
 */
#define TURTLE 1
/*!
 * \def NUM_TURTLE
 * Number of turtle training points for the classifier.
 */
#define NUM_TURTLE 39
/*!
 * \def ROBOT
 * Integer label for the toy robot.
 */
#define ROBOT 2
/*!
 * \def NUM_ROBOT
 * Number of robot training points for the classifier.
 */
#define NUM_ROBOT 18
/*!
 * \def BSG
 * Integer label for the toy Battlestar Galactica robot.
 */
#define BSG 3
/*!
 * \def NUM_BSG
 * Number of toy Battlestar Galactica robot training points for the classifier.
 */
#define NUM_BSG 19
/*!
 * \def PENS
 * Integer label for the pens.
 */
#define PENS 4
/*!
 * \def NUM_PENS
 * Number of pen training points for the classifier.
 */
#define NUM_PENS 35
/*!
 * \def COVERS
 * Integer label for the desk covers.
 */
#define COVERS 5
/*!
 * \def NUM_COVERS
 * Number of desk cover training points for the classifier.
 */
#define NUM_COVERS 54
/*!
 * \def OTHER
 * Integer label for other segmented objects.
 */
#define OTHER 6
/*!
 * \def NUM_OTHER
 * Number of other segmented objects training points for the classifier.
 */
#define NUM_OTHER 30

/*!
 * \def OBJECT_FRAME
 * The frame name in the TF tree for the object.
 */
#define OBJECT_FRAME "/computer_object_link"
/*!
 * \def BASE_FRAME
 * The frame name in the TF tree for the robot base.
 */
#define BASE_FRAME "/base_link"
/*!
 * \def CLOUD_FRAME
 * The frame name in the TF tree for the point cloud.
 */
#define CLOUD_FRAME "/head_mount_kinect_rgb_optical_frame"
/*!
 * \def GOAL_FRAME
 * The frame name in the TF tree for the arm movement goal.
 */
#define GOAL_FRAME "/touch_goal"

/*!
 * \def PADDING_X
 * Padding in the X director for the arm goal.
 */
#define PADDING_X -0.2
/*!
 * \def PADDING_Y
 * Padding in the Y director for the arm goal.
 */
#define PADDING_Y -0.1
/*!
 * \def PADDING_Z
 * Padding in the Z director for the arm goal.
 */
#define PADDING_Z -0.1

/*!
 * \class processor
 * \brief Main vision processing object.
 *
 * The processor reading from the PR2's cameras and looking for objects.
 */
class processor
{
public:
  /**
   * Creates a processor. This will subscribe to the necessary image streams and initialize their
   * callback functions.
   */
  processor();

  /**
   * Cleans up any resources from the processor.
   */
  ~processor();

private:
  /**
   * The main callback for the image feed. This will call the necessary processing functions.
   *
   * @param img the ROS image from the PR2
   */
  void feed_cback(const sensor_msgs::Image::ConstPtr& img);

  /**
   * The main callback for the point cloud.
   *
   * @param pc the ROS point cloud from the PR2
   */
  void cloud_cback(const sensor_msgs::PointCloud2::ConstPtr& pc);

  /**
   * Trains the classifier based on the set of training images.
   */
  void train();

  /**
   * Performs a Canny edge operator and probabilistic Hough transform on the given image.
   *
   * @param src the image to perform the operations on
   *
   * @return the bounding boxes found in the image
   */
  std::vector<cv::RotatedRect> segment(cv::Mat const src);

  /**
   * Creates a vector of subsections of the original image based a set
   * of bounding boxes
   *
   * @param src the image to create subsections of
   * @param rectangles the bounding boxes to subsection by
   *
   * @return the subsection images
   */
  std::vector<cv::Mat> split_image(cv::Mat const src, std::vector<cv::RotatedRect> rectangles);

  /**
   * Gets the feature vector for the given image.
   *
   * @param src the image to process
   *
   * @return the feature vector
   */
  std::vector<float> get_feature_vector(cv::Mat const src);

  /**
   * Uses the given bounding box of a possible computer location to smooth out the location.
   *
   * @param r the bounding box
   */
  void smooth(cv::RotatedRect r);

  ros::NodeHandle node; /*!< main ROS node handle */

  ros::Subscriber feed, cloud; /*!< PR2's image and point cloud streams */

  cv::Mat erode_element, dilate_element; /*!< element to use to erode and dilate */

  std::vector<cv::Point> ref_contour; /*!< used for rectangle matching in the feature vector */

  cv::RNG random; /*!< used to generate random colors */

  CvNormalBayesClassifier *bayes;

  bool capture; /*!< used to see if the user wants to capture training data first */
  int capture_count; /*!< used when saving captured images */

  std::vector<smoothing_point> smoothing_points; /*!< used to smooth the object location */
  std::vector<int> non_matches; /*!< used to remove unlikely objects during smoothing */

  int last_x, last_y; /*!< last known location of the object */
  tf::TransformBroadcaster tfb; /*!< used to find the object in 3D space */
  pcl::PointCloud<pcl::PointXYZ> latest_cloud; /*!< robots latest point cloud */
};

/*!
 * Creates and runs the processor node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
