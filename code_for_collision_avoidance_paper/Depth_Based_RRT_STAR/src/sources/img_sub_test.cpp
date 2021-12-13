 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <opencv2/highgui/highgui.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <iostream>
 #include <vector>
 #include <Eigen/Eigen>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {

    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    // std::cout<<cv_bridge::toCvShare(msg, "bgr8")->image<<std::endl;
    // cv::waitKey(30);
    cv::Mat raw_img;
    // Eigen::MatrixXd simg;
//     simg = Eigen::MatrixXd::Zero(10,10);
//     std::cout<<simg<<std::endl;
     raw_img = cv_bridge::toCvShare(msg, "32FC1")->image;
//     for (int i = 0; i < raw_img.rows; i++)
//     {
//         for (int j = 0; j < raw_img.cols; j++)
//         {
//             raw_img.at<uchar>(raw_img.rows, raw_img.cols) = raw_img.at<uchar>(raw_img.rows, raw_img.cols)+200;
//         }
//     }
     cv::imshow("view", raw_img);
    // cv::imshow("view", raw_img);
     cv::imwrite("raw_img.png", raw_img);

 }

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "image_sub");
   ros::NodeHandle nh;
  // cv::namedWindow("view", cv::WINDOW_NORMAL);
   //cv::startWindowThread();
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("/firefly1/camera_front_depth/depth/disparity", 1, imageCallback);
   ros::spin();
   //cv::destroyWindow("view");
 }
