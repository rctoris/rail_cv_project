/*!
 * \file processor.cpp
 * \brief Main image processing class for toy computer object recognition.
 *
 * The processor class reads in streams from the PR2's onboard cameras and attempts to detect the
 * toy computer via multiple steps. Additional information is available in the project's writeup.
 *
 * \author Russell Toris, David Kent, Adrian‎ Boteanu
 * \date November 28, 2012
 */

#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/ros/conversions.h>
#include <rail_cv_project/processor.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>

using namespace std;
using namespace cv;
using namespace pcl;

processor::processor() :
    random(12345)
{
  feed = node.subscribe<sensor_msgs::Image>("/kinect_head/rgb/image_rect_color", 1, &processor::feed_cback, this);
  cloud = node.subscribe<sensor_msgs::PointCloud2>("/kinect_head/depth_registered/points", 1, &processor::cloud_cback,
                                                   this);

  erode_element = getStructuringElement(MORPH_ELLIPSE, Size(2 * ERODE_ELEMENT_SIZE + 1, 2 * ERODE_ELEMENT_SIZE + 1),
                                        Point(ERODE_ELEMENT_SIZE, ERODE_ELEMENT_SIZE));
  dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(2 * DILATE_ELEMENT_SIZE + 1, 2 * DILATE_ELEMENT_SIZE + 1),
                                         Point(DILATE_ELEMENT_SIZE, DILATE_ELEMENT_SIZE));

  // create reference rectangle for comparing contours in the feature vector
  RotatedRect ref_rectangle = RotatedRect(Point2f(1, 1), Size2f(REF_WIDTH, REF_HEIGHT), 90);
  Point2f r_vert[4];
  ref_rectangle.points(r_vert);
  for (int i = 0; i < 4; i++)
    ref_contour.push_back(r_vert[i]);

  // check if we need to capture images first
  string capture_input;
  cout << "Would you like to capture training images first? (Y/n): ";
  cin >> capture_input;
  if (capture_input.compare("Y") == 0 || capture_input.compare("y") == 0)
  {
    capture_count = 0;
    capture = true;
  }
  else
  {
    capture = false;
    last_x = 0;
    last_y = 0;

    // create the classifier
    bayes = new CvNormalBayesClassifier();
    // train the classifier
    train();
  }
}

processor::~processor()
{
  delete bayes;
}

void processor::train()
{
  cout << "-- Begin Model Training --" << endl;
  string path = ros::package::getPath("rail_cv_project");

  Mat training(NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS + NUM_COVERS + NUM_OTHER, D, CV_32FC1);
  Mat resp(NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS + NUM_COVERS + NUM_OTHER, 1, CV_32FC1);
  // load all training images
  for (int i = 0; i < NUM_COMPUTER; i++)
  {
    stringstream s;
    s << path << "/img/computer" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i, j) = v.at(j);
    resp.at<float>(i, 0) = COMPUTER;
  }
  for (int i = 0; i < NUM_TURTLE; i++)
  {
    stringstream s;
    s << path << "/img/turtle" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER, 0) = TURTLE;
  }
  for (int i = 0; i < NUM_ROBOT; i++)
  {
    stringstream s;
    s << path << "/img/robot" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER + NUM_TURTLE, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER + NUM_TURTLE, 0) = ROBOT;
  }
  for (int i = 0; i < NUM_BSG; i++)
  {
    stringstream s;
    s << path << "/img/bsg" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT, 0) = BSG;
  }
  for (int i = 0; i < NUM_PENS; i++)
  {
    stringstream s;
    s << path << "/img/pens" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG, 0) = PENS;
  }
  for (int i = 0; i < NUM_COVERS; i++)
  {
    stringstream s;
    s << path << "/img/cover" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS, 0) = COVERS;
  }
  for (int i = 0; i < NUM_OTHER; i++)
  {
    stringstream s;
    s << path << "/img/other" << i << ".jpg";
    cout << "\tTraining '" << s.str() << "'..." << endl;
    vector<float> v = get_feature_vector(imread(s.str()));
    for (uint j = 0; j < v.size(); j++)
      training.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS + NUM_COVERS, j) = v.at(j);
    resp.at<float>(i + NUM_COMPUTER + NUM_TURTLE + NUM_ROBOT + NUM_BSG + NUM_PENS + NUM_COVERS, 0) = OTHER;
  }

  // train the model
  bayes->train(training, resp);

  cout << "-- Model Trained --" << endl;
}

void processor::feed_cback(const sensor_msgs::Image::ConstPtr& img)
{
  // convert from a ROS image to an OpenCV image
  cv_bridge::CvImagePtr cv_img;
  cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

  // segment the image into bounding boxes on possible objects
  vector<RotatedRect> rectangles = segment(cv_img->image);

  // create a separate image for each segmented section
  vector<Mat> images = split_image(cv_img->image, rectangles);

  // check if we are training, or if we are classifying
  if (capture)
  {
    // save each and wait
    for (uint i = 0; i < images.size(); i++)
    {
      // save the image
      stringstream s;
      s << ros::package::getPath("rail_cv_project") << "/img/IMAGE-" << capture_count++ << ".jpg";
      imwrite(s.str(), images.at(i));
    }

    string capture_input;
    cout << "Continue to capture? (Y/n): ";
    cin >> capture_input;
    if (capture_input.compare("Y") != 0 && capture_input.compare("y") != 0)
      exit(0);
  }
  else
  {
    //initially add all the smoothed points as unvisited
    for (uint i = 0; i < smoothing_points.size(); i++)
      non_matches.push_back(i);

    // classify chunks and find the computer
    Mat possible_matches = cv_img->image.clone();
    Mat classes = cv_img->image.clone();
    for (uint i = 0; i < images.size(); i++)
    {
      // copy the vector into a CV Mat
      vector<float> v = get_feature_vector(images.at(i));
      Mat pred(1, v.size(), CV_32FC1);
      for (uint j = 0; j < v.size(); j++)
        pred.at<float>(0, j) = v.at(j);

      // check if we found a computer
      int classification = (int)bayes->predict(pred);
      if (classification == COMPUTER)
      {
        // add it to the list of possible computer locations
        smooth(rectangles.at(i));

        // draw this on the image as possible locations
        Point2f rect_points[4];
        RotatedRect possible = rectangles.at(i);
        possible.points(rect_points);
        for (int j = 0; j < 4; j++)
          line(possible_matches, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 3);
      }
      // label all classes
      string text = "";
      switch (classification)
      {
        case COMPUTER:
          text = "Toy";
          break;
        case TURTLE:
          text = "Turtle";
          break;
        case ROBOT:
          text = "Robot";
          break;
        case BSG:
          text = "Cylon";
          break;
        case PENS:
          text = "Pens";
          break;
        case COVERS:
          text = "Cover";
          break;
        default:
          text = "Turtle";
          break;
      }
      // bold text
      for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        {
          int center_x = max(0, min((int)rectangles.at(i).center.x + x, classes.cols));
          int center_y = max(0, min((int)rectangles.at(i).center.y + y, classes.rows));
          putText(classes, text, Point(center_x, center_y), FONT_HERSHEY_PLAIN, 2, Scalar(0, 64, 255));
        }

    }

    // clear any zeros in the smoothing vector
    for (int i = non_matches.size() - 1; i >= 0; i--)
    {
      smoothing_points.at(non_matches.at(i)).c -= 2;
      if (smoothing_points.at(non_matches.at(i)).c <= 0)
        smoothing_points.erase(smoothing_points.begin() + non_matches.at(i));
    }
    non_matches.clear();

    // find the location with the most counts
    int max_c = 0;
    RotatedRect best;
    for (uint i = 0; i < smoothing_points.size(); i++)
    {
      if (smoothing_points.at(i).c > max_c)
      {
        best = smoothing_points.at(i).rect;
        max_c = smoothing_points.at(i).c;
      }
    }

    // outline it in the images
    if (max_c > 0 && max_c > MAX_SMOOTHING_COUNT / 4)
    {
      Point2f rect_points[4];
      best.points(rect_points);
      for (int j = 0; j < 4; j++)
      {
        line(possible_matches, rect_points[j], rect_points[(j + 1) % 4], Scalar(0, 255, 0), 4);
        line(cv_img->image, rect_points[j], rect_points[(j + 1) % 4], Scalar(0, 0, 255), 8);
      }

      // check if we should update the TF
      if (sqrt(
          pow((double)(best.center.x - last_x), 2.0) + pow((double)(best.center.y - last_y), 2.0)) > SMOOTH_DIST_THRESH)
      {
        // get the depth point
        if (latest_cloud.isOrganized() && latest_cloud.height > 1)
        {
          PointXYZ p = latest_cloud.at(best.center.x, best.center.y);
          // create a transform for the point
          tf::Transform tf;
          tf.setOrigin(tf::Vector3(p.x, p.y, p.z));
          tf.setRotation(tf::Quaternion(0, 0, 0, 1));
          tfb.sendTransform(tf::StampedTransform(tf, ros::Time::now(), CLOUD_FRAME, OBJECT_FRAME));

          // add a padding point to set as the goal
          tf::Transform tf2;
          tf2.setOrigin(tf::Vector3(p.x + PADDING_X, p.y + PADDING_Y, p.z + PADDING_Z));
          tf2.setRotation(tf::Quaternion(0, 0, 0, 1));
          tfb.sendTransform(tf::StampedTransform(tf2, ros::Time::now(), CLOUD_FRAME, GOAL_FRAME));

          // update our latest points
          last_x = best.center.x;
          last_y = best.center.y;
        }
      }

    }
    imshow("Possible Matches", possible_matches);
    imshow("Classes", classes);
    imshow("Final", cv_img->image);
  }

  // leave the windows open
  waitKey(3);
}

void processor::smooth(RotatedRect r)
{
  // go through the list and check for a match
  bool match = false;
  for (uint i = 0; i < smoothing_points.size(); i++)
  {
    RotatedRect cur = smoothing_points.at(i).rect;
    double dist = sqrt(pow((double)(r.center.x - cur.center.x), 2.0) + pow((double)(r.center.y - cur.center.y), 2.0));
    if (dist < SMOOTH_DIST_THRESH)
    {
      match = true;
      // update the box
      smoothing_points.at(i).rect = r;
      // increment the counter
      smoothing_points.at(i).c = min(MAX_SMOOTHING_COUNT, smoothing_points.at(i).c + 1);

      // check if this is inside the non-matches from a previous check
      for (uint j = 0; j < non_matches.size(); j++)
      {
        if (non_matches.at(j) == (int)i)
        {
          non_matches.erase(non_matches.begin() + j);
          continue;
        }
      }
    }
  }

  // if no match was found, we add it
  if (!match)
  {
    smoothing_point p;
    p.c = 1;
    p.rect = r;
    smoothing_points.push_back(p);
  }
}

vector<RotatedRect> processor::segment(Mat const src)
{
  // blur the image and convert to grayscale
  Mat img;
  cv::cvtColor(src, img, CV_BGR2GRAY);
  blur(img, img, Size(BLUR_KERNEL, BLUR_KERNEL));
  // erode to get rid of details
  erode(img, img, erode_element);

  // run the Canny edge operator, dilate, then run it again to help find better contours
  Canny(img, img, CANNY_MIN_THRESH, CANNY_MAX_THRESH, CANNY_KERNEL);
  imshow("First Canny", img);
  dilate(img, img, dilate_element);
  Canny(img, img, CANNY_MIN_THRESH / 1, CANNY_MAX_THRESH / 1, CANNY_KERNEL);
  imshow("Second Canny", img);

  // find the contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  // search for valid bounding rectangles
  vector<RotatedRect> rectangles;
  Mat final = Mat::zeros(img.size(), CV_8UC3);
  for (uint i = 0; i < contours.size(); i++)
  {
    // find the bounding box
    RotatedRect r = minAreaRect(Mat(contours[i]));

    // filter the bounding box (flip it to be taller rather than wider)
    float width = min(r.size.width, r.size.height);
    float length = max(r.size.width, r.size.height);
    if (width / length >= RATIO_THRESHOLD && width * length >= MIN_AREA && width * length <= MAX_AREA)
    {
      // pick a random color to draw the contour
      Scalar color = Scalar(random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255));
      drawContours(final, contours, i, color, CV_FILLED, 8, hierarchy, 0, Point());

      // add it to the list
      rectangles.push_back(r);
    }
  }

  // only continue if we found some
  if (rectangles.size() > 0)
  {
    // filter any bounding boxes around the same point
    for (uint i = 0; i < rectangles.size() - 1; i++)
    {
      RotatedRect r = rectangles[i];
      vector<int> removeIndeces;
      float maxSize = r.size.width + r.size.height;
      int maxIndex = i;
      for (uint j = i + 1; j < rectangles.size(); j++)
      {
        if (sqrt(pow(r.center.x - rectangles[j].center.x, 2) + pow(r.center.y - rectangles[j].center.y, 2))
            < sqrt(MIN_AREA))
        {
          // take the larger of the two
          int rSize = rectangles[j].size.width + rectangles[j].size.height;
          if (rSize > maxSize)
          {
            removeIndeces.push_back(maxIndex);
            maxSize = rSize;
            maxIndex = j;
          }
          else
            removeIndeces.push_back(j);
        }
      }
      // remove the ones we don't want
      sort(removeIndeces.begin(), removeIndeces.end());
      for (int j = removeIndeces.size() - 1; j >= 0; j--)
        rectangles.erase(rectangles.begin() + removeIndeces[j]);
    }

    // draw them
    for (uint i = 0; i < rectangles.size(); i++)
    {
      Scalar color = Scalar(random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255));
      // draw the bounding box
      Point2f rect_points[4];
      rectangles[i].points(rect_points);
      for (int j = 0; j < 4; j++)
        line(final, rect_points[j], rect_points[(j + 1) % 4], color, 1);
    }
  }

  imshow("Contours", final);

  return rectangles;
}

vector<Mat> processor::split_image(Mat const src, vector<RotatedRect> rectangles)
{
  vector<Mat> images;
  // check all the areas we are interested in
  for (uint i = 0; i < rectangles.size(); i++)
  {
    Point2f pts[4];
    rectangles[i].points(pts);

    // grab the points
    cv::Point2f srcVertices[3];
    srcVertices[0] = pts[0];
    srcVertices[1] = pts[1];
    srcVertices[2] = pts[3];
    Point2f dstVertices[3];
    dstVertices[0] = Point(0, 0);
    dstVertices[1] = Point(rectangles[i].size.width - 1, 0);
    dstVertices[2] = Point(0, rectangles[i].size.height - 1);

    // pick out the area we want from the source
    Mat warpAffineMatrix = getAffineTransform(srcVertices, dstVertices);
    Size size(rectangles[i].size.width, rectangles[i].size.height);
    Mat img;
    warpAffine(src, img, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);

    images.push_back(img);
  }

  return images;
}

vector<float> processor::get_feature_vector(Mat const src)
{
  // average RGB
  float avg_red = 0.0;
  float avg_green = 0.0;
  float avg_blue = 0.0;
  for (int i = 0; i < src.rows; i++)
    for (int j = 0; j < src.cols; j++)
    {
      Vec3b intensity = src.at<Vec3b>(i, j);
      avg_red += intensity.val[2];
      avg_green += intensity.val[1];
      avg_blue += intensity.val[0];
    }
  float area = src.cols * src.rows;
  avg_red /= (area * 255.0);
  avg_blue /= (area * 255.0);
  avg_green /= (area * 255.0);

  // edge and line features
  Mat img1, img2;
  cvtColor(src, img1, CV_BGR2GRAY);
  blur(img1, img1, Size(BLUR_KERNEL, BLUR_KERNEL));
  Canny(img1, img1, CANNY_MIN_THRESH, CANNY_MAX_THRESH, CANNY_KERNEL);

  // find the number of lines in the image
  vector<cv::Vec4i> lines;
  HoughLinesP(img1, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
  float avg_lines = min((double)lines.size(), MAX_LINES) / MAX_LINES;

  // get the average line length
  float avg_line_length = 0.0;
  for (uint i = 0; i < lines.size(); i++)
    avg_line_length += sqrt(pow(lines[i][0] - lines[i][2], 2.0) + pow(lines[i][1] - lines[i][3], 2.0));
  avg_line_length /= lines.size();
  avg_line_length = max(0.0, min((double)avg_line_length, MAX_AVG_LINE_LENGTH)) / MAX_AVG_LINE_LENGTH;

  // rerun Canny to get our "bubbled" contours
  cvtColor(src, img2, CV_BGR2GRAY);
  blur(img2, img2, Size(BLUR_KERNEL, BLUR_KERNEL));
  erode(img2, img2, erode_element);
  Canny(img2, img2, CANNY_MIN_THRESH, CANNY_MAX_THRESH, CANNY_KERNEL);
  dilate(img2, img2, dilate_element);
  Canny(img2, img2, CANNY_MIN_THRESH / 2, CANNY_MAX_THRESH / 2, CANNY_KERNEL);
  // fine the contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(img2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  // compare contours with the rectangle
  float rect_match_feature = 0;
  for (uint i = 0; i < contours.size(); i++)
    if (matchShapes(contours[i], ref_contour, CV_CONTOURS_MATCH_I1, 0) == 0)
      rect_match_feature++;
  if (contours.size() > 0)
    rect_match_feature /= contours.size();
  else
    rect_match_feature = 0;

  // calculate the number of SIFT features
  SiftFeatureDetector detector;
  vector<KeyPoint> keypoints;
  detector.detect(src, keypoints);
  float avg_sift = min((double)keypoints.size(), MAX_SIFT) / MAX_SIFT;

  // create the feature vector
  vector<float> features;
  features.push_back(avg_red);
  features.push_back(avg_green);
  features.push_back(avg_blue);
  features.push_back(avg_lines);
  features.push_back(avg_line_length);
  features.push_back(avg_sift);
  features.push_back(rect_match_feature);

  return features;
}

void processor::cloud_cback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  // copy over the point cloud to save it
  fromROSMsg(*pc, latest_cloud);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "processor");
  processor p;

  // continue until a ctrl-c has occurred
  ros::spin();

  return EXIT_SUCCESS;
}
