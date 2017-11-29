#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_geometry/stereo_camera_model.h>

#include <std_msgs/String.h>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Twist.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace ros;
using namespace std;

ros::Publisher center_pub;
ros::Publisher image_pub;

ros::Publisher vel_pub;

Rect findMinRect(const Mat1b& src)
{
    Mat1f W(src.rows, src.cols, float(0));
    Mat1f H(src.rows, src.cols, float(0));

    Rect maxRect(0,0,0,0);
    float maxArea = 0.f;

    for (int r = 0; r < src.rows; ++r)
    {
        for (int c = 0; c < src.cols; ++c)
        {
            if (src(r, c) == 0)
            {
                H(r, c) = 1.f + ((r>0) ? H(r-1, c) : 0);
                W(r, c) = 1.f + ((c>0) ? W(r, c-1) : 0);
            }

            float minw = W(r,c);
            for (int h = 0; h < H(r, c); ++h)
            {
                minw = min(minw, W(r-h, c));
                float area = (h+1) * minw;
                if (area > maxArea)
                {
                    maxArea = area;
                    maxRect = Rect(Point(c - minw + 1, r - h), Point(c+1, r+1));
                }
            }
        }
    }

    return maxRect;
}


RotatedRect largestRectInNonConvexPoly(const Mat1b& src)
{
    // Create a matrix big enough to not lose points during rotation
    vector<Point> ptz;
    findNonZero(src, ptz);
    Rect bbox = boundingRect(ptz); 
    int maxdim = max(bbox.width, bbox.height);
    Mat1b work(2*maxdim, 2*maxdim, uchar(0));
    src(bbox).copyTo(work(Rect(maxdim - bbox.width/2, maxdim - bbox.height / 2, bbox.width, bbox.height)));

    // Store best data
    Rect bestRect;
    int bestAngle = 0;

    // For each angle
    for (int angle = 0; angle < 14; angle += 15)
    {
        // cout << angle << endl;

        // Rotate the image
        Mat R = getRotationMatrix2D(Point(maxdim,maxdim), angle, 1);
        Mat1b rotated;
        warpAffine(work, rotated, R, work.size());

        // Keep the crop with the polygon
        vector<Point> pts;
        findNonZero(rotated, pts);
        Rect box = boundingRect(pts);
        Mat1b crop = rotated(box).clone();

        // Invert colors
        crop = ~crop; 

         Rect r = findMinRect(crop);

        // If best, save result
        if (r.area() > bestRect.area())
        {
            bestRect = r + box.tl();    // Correct the crop displacement
            bestAngle = angle;
        }
    }

    // Apply the inverse rotation
    Mat Rinv = getRotationMatrix2D(Point(maxdim, maxdim), -bestAngle, 1);
    vector<Point> rectPoints{bestRect.tl(), Point(bestRect.x + bestRect.width, bestRect.y), bestRect.br(), Point(bestRect.x, bestRect.y + bestRect.height)};
    vector<Point> rotatedRectPoints;
    transform(rectPoints, rotatedRectPoints, Rinv);

    // Apply the reverse translations
    for (int i = 0; i < rotatedRectPoints.size(); ++i)
    {
        rotatedRectPoints[i] += bbox.tl() - Point(maxdim - bbox.width / 2, maxdim - bbox.height / 2);
    }

    // Get the rotated rect
    RotatedRect rrect = minAreaRect(rotatedRectPoints);

    return rrect;
}

void dispCallback(const ImageConstPtr& depth_map, const ImageConstPtr& left_rgb_rect) {

  cv_bridge::CvImagePtr cv_ptr, cv_ptr_rect;

  cv_ptr_rect = cv_bridge::toCvCopy(left_rgb_rect, "bgr8");
  cv_ptr = cv_bridge::toCvCopy(depth_map);

  Mat1b img;

  geometry_msgs::Twist twist;

  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.z = 0;

  normalize(cv_ptr->image, img, 0, 255, CV_MINMAX, CV_8UC1);

  Mat dummy = Mat::zeros( img.size(), img.type() );

  double thresh_value = cv::threshold(img, dummy, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  cv::threshold(img, img, (thresh_value / 2), 255, CV_THRESH_BINARY);

  int erosion_size = 3;  			// 5
  Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );

  dilate(img, img, element);

  Point2f center;
  for(int round = 0; round < 1; round++) {
      
    if(countNonZero(img) > 0) {
      RotatedRect r = largestRectInNonConvexPoly(img);
    
      Point2f points[4];
      r.points(points);

      // for (int i = 0; i < 4; ++i) {
        // line(cv_ptr->image, points[i], points[(i + 1) % 4], Scalar(0, 0, 255), 2);
        // line(cv_ptr_rect->image, points[i]*3, points[(i + 1) % 4]*3, Scalar(0, 0, 255), 2);
      // }

      center.x = (points[0].x + points[2].x) / 2.0;
      center.y = (points[0].y + points[2].y) / 2.0;
      // std_msgs::String msg;
      // msg.data = std::to_string(center.x) + ',' + std::to_string(center.y);
    
    // rectangle(img, points[2], points[0], Scalar(0, 0, 0), -1);
    }
    if(center.x < (cv_ptr->image.cols)/2.0){
	twist.linear.y = 0.05;
    }
    if(center.x > (cv_ptr->image.cols)/2.0){
	twist.linear.y = -0.05;
    }
    // if(center.y < (cv_ptr->image.rows)/2.0){
	// twist.linear.z = -0.05;
    // }
    // if(center.y > (cv_ptr->image.rows)/2.0){
	// twist.linear.z = 0.05;
    // }
    if(cv_ptr->image.at<float>(center.x,center.y) > 1.75){
	twist.linear.x = 0.05;
    }

    // center_pub.publish(msg);
    // image_pub.publish(cv_ptr_rect->toImageMsg());
  }
    vel_pub.publish(twist);


  // imshow("window", cv_ptr->image); waitKey(100);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disp_stuff");

  ros::NodeHandle nh_;

  message_filters::Subscriber<Image> img_sub_1(nh_, "/depth_map/image", 10);
  message_filters::Subscriber<Image> img_sub_2(nh_, "/left_rgb_rect/image_rect_color", 10);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync_1(MySyncPolicy(10), img_sub_1, img_sub_2);

  sync_1.registerCallback(boost::bind(&dispCallback, _1, _2));

  // center_pub = nh_.advertise<std_msgs::String>("center", 10);
  // image_pub = nh_.advertise<Image>("the_centered_rect", 1);

  vel_pub = nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);

  ros::spin();

  return 0;
}
