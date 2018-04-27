#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

 //ROS Images
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>


//Aruco Messages
#include "aruco_eye_msgs/PointInImage.h"
#include "aruco_eye_msgs/Marker.h"
#include "aruco_eye_msgs/MarkerList.h"
        
#define dx 0.3
#define dy 0.3

using namespace std;

namespace Localization{
class local_position{
    public:
        local_position(ros::NodeHandle& nh,ros::NodeHandle& pnh);
        void MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg);
        void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
        void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat);
        void Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot);
        void Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q);
        void Euler2Rota(geometry_msgs::Vector3 &euler, Eigen::Matrix3d& rot);
        void KalmanFilterInit();
        void PositionKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lx_e);
        void PositionCompKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lcx_e);
        void AttitudeKalmanFilter(geometry_msgs::Point& attitude, Eigen::VectorXd& Ax_e);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber get_marker_pose ;
        ros::Publisher att_uav;
        ros::Publisher att_uav_kf;
        ros::Publisher pos_uav;
        ros::Publisher pos_comp_uav;
        ros::Publisher pos_uav_kf ;
        ros::Publisher pos_comp_uav_kf ;
        ros::Publisher vel_uav;
        ros::Time pos_time_last;
        
        geometry_msgs::PointStamped  att_pub;
        geometry_msgs::PointStamped  att_kf_pub;
        geometry_msgs::Quaternion quat_pub;
        geometry_msgs::PoseStamped  pos_pub;
        geometry_msgs::PoseStamped  pos_comp_pub;
        geometry_msgs::PoseStamped  pos_kf_pub;
        geometry_msgs::PoseStamped  pos_comp_kf_pub;

        geometry_msgs::PointStamped  vel_pub;

        geometry_msgs::Point position_dir;
        geometry_msgs::Point position_dir_kf;
        geometry_msgs::Point position_comp;
        geometry_msgs::Point position_comp_kf;

        
        geometry_msgs::Point position_dir_last;
        // geometry_msgs::Point position_dir_kf_last;
        // geometry_msgs::Point position_comp_last;
        // geometry_msgs::Point position_comp_kf_last;
       
        geometry_msgs::Point velocity_dir_last;
        // geometry_msgs::Point velocity_dir_kf_last;
        // geometry_msgs::Point velocity_comp_last;
        // geometry_msgs::Point velocity_comp_kf_last;

        geometry_msgs::Point velocity_dir;
        // geometry_msgs::Point velocity_dir_kf;
        // geometry_msgs::Point velocity_comp;
        // geometry_msgs::Point velocity_comp_kf;

        geometry_msgs::Point att_last;
        geometry_msgs::Point att_smooth;


        int count_markers, ID;
        double roll, pitch, yaw;
        double bias_cam = 0.1;
        double min_height = -0.8;
        double max_height = -2.5;
        double error_pitch_cam = (90-80.8)/180*3.14159;
        geometry_msgs::Point pos_marker;
        geometry_msgs::Quaternion quat_marker;

        geometry_msgs::Vector3 euler;
        geometry_msgs::Vector3 rpy;
        
        Eigen::Matrix3d rot;
        Eigen::Vector3d pos_in_body;
        Eigen::Vector3d pos_in_tag;




        double low_pass_param_vel= 0.3; // velocity low pass filter parameter
        double low_pass_param_att=0.2; // velocity smooth filter
        // for kalman filter, x -> pos, z -> measurement
        double dt;
        Eigen::MatrixXd I_6;
        Eigen::Matrix3d I_3;
        Eigen::MatrixXd F;
        Eigen::MatrixXd Tao;
        Eigen::MatrixXd H;

        //for attitude
        double Akp, Akq, Akr;
        Eigen::VectorXd Ax_e_;
        Eigen::VectorXd Ax_e;
        Eigen::Vector3d Az_m;
        Eigen::MatrixXd AP_;
        Eigen::MatrixXd AP;
        Eigen::MatrixXd AQ;
        Eigen::Matrix3d AR;
        Eigen::MatrixXd AK;

        //for position 
        double Lkp, Lkq, Lkr;
        Eigen::VectorXd Lx_e_;
        Eigen::VectorXd Lx_e;
        Eigen::Vector3d Lz_m;
        Eigen::MatrixXd LP_;
        Eigen::MatrixXd LP;
        Eigen::MatrixXd LQ;
        Eigen::Matrix3d LR;
        Eigen::MatrixXd LK;

        //for position comp
        double Lckp, Lckq, Lckr;
        Eigen::VectorXd Lcx_e_;
        Eigen::VectorXd Lcx_e;
        Eigen::Vector3d Lcz_m;
        Eigen::MatrixXd LcP_;
        Eigen::MatrixXd LcP;
        Eigen::MatrixXd LcQ;
        Eigen::Matrix3d LcR;
        Eigen::MatrixXd LcK;

};
}






