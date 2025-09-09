#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {
public:
    ArucoDetector(ros::NodeHandle& nh) 
        : it_(nh) 
    {
        // Parameters (can be overridden via rosparam)
        nh.param("marker_size", marker_size_, 0.05); // 5 cm default
        nh.param("camera_fx", fx_, 600.0);
        nh.param("camera_fy", fy_, 600.0);
        nh.param("camera_cx", cx_, 480.0);
        nh.param("camera_cy", cy_, 270.0);

        // Camera matrix
        camera_matrix_ = (cv::Mat1d(3,3) << 
                          fx_, 0,   cx_,
                          0,   fy_, cy_,
                          0,   0,   1);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ArucoDetector::imageCallback, this);
        image_pub_ = it_.advertise("aruco_image", 1);
        pose_pub_  = nh.advertise<geometry_msgs::PoseStamped>("aruco_pose", 1);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        ROS_INFO_ONCE("[ArucoDetector]: ArucoDetector initialized:");
        ROS_INFO_ONCE("[ArucoDetector]: - marker_size: %.3f m", marker_size_);
        ROS_INFO_ONCE("[ArucoDetector]: - fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", fx_, fy_, cx_, cy_);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
       
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("[ArucoDetector]: cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(image, dictionary_, corners, ids);

        if (ids.empty()) {
            ROS_WARN_THROTTLE(1.0, "[ArucoDetector]: No markers detected in current frame.");

        } else {

            ROS_INFO_THROTTLE(1.0, "[ArucoDetector]: Detected %lu marker(s).", ids.size());
           
            cv::aruco::drawDetectedMarkers(image, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); i++) {
                ROS_INFO_THROTTLE(1.0, "[ArucoDetector]: Marker ID %d -> Position [x=%.3f, y=%.3f, z=%.3f] m",
                         ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);

                cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ * 0.5);

                // Publish Pose
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header = msg->header;

                pose_msg.pose.position.x = tvecs[i][0];
                pose_msg.pose.position.y = tvecs[i][1];
                pose_msg.pose.position.z = tvecs[i][2];

                cv::Mat R;
                cv::Rodrigues(rvecs[i], R);

                cv::Matx33d rotMat;
                R.copyTo(rotMat);
                double qw = sqrt(1.0 + rotMat(0,0) + rotMat(1,1) + rotMat(2,2)) / 2.0;
                double qx = (rotMat(2,1) - rotMat(1,2)) / (4.0*qw);
                double qy = (rotMat(0,2) - rotMat(2,0)) / (4.0*qw);
                double qz = (rotMat(1,0) - rotMat(0,1)) / (4.0*qw);

                pose_msg.pose.orientation.x = qx;
                pose_msg.pose.orientation.y = qy;
                pose_msg.pose.orientation.z = qz;
                pose_msg.pose.orientation.w = qw;

                pose_pub_.publish(pose_msg);
                ROS_INFO_THROTTLE(1.0, "[ArucoDetector]: Published pose for marker ID %d.", ids[i]);

                // Draw marker center
                cv::Point2f center(0,0);
                for (const auto& pt : corners[i]) center += pt;
                center *= (1.0 / 4.0);
                cv::circle(image, center, 5, cv::Scalar(0,0,255), -1);
            }
        }
        // Publish result image
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        image_pub_.publish(out_msg);
        ROS_INFO_THROTTLE(1.0, "[ArucoDetector]: Published annotated image.");
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_size_;
    double fx_, fy_, cx_, cy_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh("~");
    ArucoDetector detector(nh);
        ros::spin();
    return 0;
}
