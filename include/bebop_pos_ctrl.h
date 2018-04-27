
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/TwistStamped.h>
#include <cmath>
typedef std::vector<geometry_msgs::Twist>::iterator VecMovIt;

using namespace std;

namespace Bebop_Ctrl{
class bebop_pos_ctrl{
    public:
        bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);

        void fillPatrolList();
        void Control2Goal(const geometry_msgs::Twist& goal_pose);//-+
        void BebopPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void BebopVelCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void PIDPosControl(const  geometry_msgs::Twist& goal_pose_, 
            const  geometry_msgs::PoseStamped& current_pose_,
            const  geometry_msgs::PointStamped& current_vel_,  
            geometry_msgs::Twist& velocity_ctrl_);

        void PIDinit();
        void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
        void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat);
        void Limitator(double& vx, double& vy, double& vz, double& yawrate);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber get_marker_pose;
        ros::Subscriber get_velocity;
        ros::Publisher  bebop_cmd_vel;
        geometry_msgs::PoseStamped pos_sub;
        geometry_msgs::PointStamped vel_sub;
        geometry_msgs::Twist cmd_vel_pub;

        std::vector<geometry_msgs::Twist> patrol_list_;
        int num_point;
        int Hz;
        int time_hover;
        bool debug = true;
        double min_x = 0.1;
        double max_x = 2.9;
        double min_y = 0.1;
        double max_y = 2.9;
        double min_height = 1.0;
        double max_height = 2.5;
        double patrol_height = -1.2;

        double  K_p_x, K_p_y, K_p_z, K_p_yaw, K_i_x, K_i_y, K_i_z, K_i_yaw, K_d_x, K_d_y, K_d_z, K_d_yaw;
        double  MaxV_xy, MaxV_z , MaxV_yaw, Limit_xy_error_int, Limit_z_error_int, Limit_yaw_error_int;
        double Tolerance_hori_pos,Tolerance_vert_pos,Tolerance_yaw_rad;
        string trajec_filepath;
        double Tag_or_Mocap;// 1.0: tag navgation; -1.0: mocap navgation

        double  error_x_last, error_y_last, error_z_last, error_yaw_last;
        double  error_x_currect, error_y_currect, error_z_currect, error_yaw_currect;
        double  error_x_accu, error_y_accu, error_z_accu, error_yaw_accu;
        double del_t; //second /s
        geometry_msgs::PoseStamped last_pose;
        double yaw_last;
        double yaw_current;
        geometry_msgs::Vector3 euler_last;
        geometry_msgs::Vector3 euler_current;

};
}






