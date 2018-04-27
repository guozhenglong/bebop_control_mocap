#include "localization.h"
namespace Localization
{
    local_position::local_position(ros::NodeHandle& nh,ros::NodeHandle& pnh):nh_(nh)
    {
        pnh.param("Akp", Akp, 1.0);
        pnh.param("Akq", Akq, 1.0);
        pnh.param("Akr", Akr, 1.0);
        pnh.param("Lkp", Lkp, 1.0);
        pnh.param("Lkq", Lkq, 1.0);
        pnh.param("Lkr", Lkr, 1.0);
        pnh.param("Lckp", Lckp, 1.0);
        pnh.param("Lckq", Lckq, 1.0);
        pnh.param("Lckr", Lckr, 1.0);

        pnh.param("low_pass_param_vel", low_pass_param_vel, 0.3);
        pnh.param("low_pass_param_att", low_pass_param_att, 0.3);
        
        local_position::KalmanFilterInit();
        get_marker_pose = nh_.subscribe("/aruco_eye/aruco_observation",1,&local_position::MarkerPoseCallback,this);
        att_uav = nh_.advertise<geometry_msgs::PointStamped>("/att_uav",1);
        att_uav_kf = nh_.advertise<geometry_msgs::PointStamped>("/att_uav_kf",1);
        pos_uav = nh_.advertise<geometry_msgs::PoseStamped>("/pos_uav",1);
        pos_comp_uav = nh_.advertise<geometry_msgs::PoseStamped>("/pos_comp_uav",1);
        pos_uav_kf = nh_.advertise<geometry_msgs::PoseStamped>("/pos_uav_kf",1);
        pos_comp_uav_kf = nh_.advertise<geometry_msgs::PoseStamped>("/pos_comp_uav_kf",1);

        vel_uav = nh_.advertise<geometry_msgs::PointStamped>("/vel_uav",1);

        ros::Rate loopRate(20);
        pos_time_last = ros::Time::now();
        while(ros::ok())
        {
            pos_uav.publish(pos_pub);
            pos_comp_uav.publish(pos_comp_pub);
            att_uav.publish(att_pub);
    
            pos_uav_kf.publish(pos_kf_pub);
            pos_comp_uav_kf.publish(pos_comp_kf_pub);
            att_uav_kf.publish(att_kf_pub);
    
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    void local_position::KalmanFilterInit()
    {
        rot = Eigen::Matrix3d::Zero(3,3);  
        I_6 = Eigen::MatrixXd::Zero(6,6);
        I_3 = Eigen::MatrixXd::Zero(3,3);
        F = Eigen::MatrixXd::Zero(6,6);
        H = Eigen::MatrixXd::Zero(3,6);
        Tao = Eigen::MatrixXd::Zero(6,6);

        Ax_e_= Eigen::VectorXd::Zero(6);
        Ax_e= Eigen::VectorXd::Zero(6);
        AP_ = Eigen::MatrixXd::Zero(6,6);
        AP = Eigen::MatrixXd::Zero(6,6);
        AQ = Eigen::MatrixXd::Zero(6,6);
        AR = Eigen::MatrixXd::Zero(3,3);
        AK = Eigen::MatrixXd::Zero(6,6);

        Lx_e_= Eigen::VectorXd::Zero(6);
        Lx_e= Eigen::VectorXd::Zero(6);
        LP_ = Eigen::MatrixXd::Zero(6,6);
        LP = Eigen::MatrixXd::Zero(6,6);
        LQ = Eigen::MatrixXd::Zero(6,6);
        LR = Eigen::MatrixXd::Zero(3,3);
        LK = Eigen::MatrixXd::Zero(6,6);

        Lcx_e_= Eigen::VectorXd::Zero(6);
        Lcx_e= Eigen::VectorXd::Zero(6);
        LcP_ = Eigen::MatrixXd::Zero(6,6);
        LcP = Eigen::MatrixXd::Zero(6,6);
        LcQ = Eigen::MatrixXd::Zero(6,6);
        LcR = Eigen::MatrixXd::Zero(3,3);
        LcK = Eigen::MatrixXd::Zero(6,6);
        
        dt = 1.0/30.0;
        F<< 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        //F = dt*F + I_6;

        Tao<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        H<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

        I_6<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
       

        I_3<< 1.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 
            0.0, 0.0, 1.0; 
              
        Ax_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        AP = Akp * I_6;
        AQ = Akq * I_6;
        AR = Akr * I_3;


        Lx_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        LP = Lkp * I_6;
        LQ = Lkq * I_6;
        LR = Lkr * I_3;
        LR(2,2) = 2*LR(2,2);

        Lcx_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        LcP = Lckp * I_6;
        LcQ = Lckq * I_6;
        LcR = Lckr * I_3;
        LcR(2,2) = 2*LcR(2,2);


        // position_last and velocity_last zeros.
        position_dir_last.x = 0.0;
        position_dir_last.y = 0.0;
        position_dir_last.z = 0.0;
      
        velocity_dir_last.x = 0.0;
        velocity_dir_last.y = 0.0;
        velocity_dir_last.z = 0.0;

        velocity_dir.x = 0.0;
        velocity_dir.y = 0.0;
        velocity_dir.z = 0.0;
        
        att_last.x = 0.0;
        att_last.y = 0.0;
        att_last.z = 0.0;
    

    }

    void local_position::AttitudeKalmanFilter(geometry_msgs::Point& attitude, Eigen::VectorXd& Ax_e)
    {
         // Kalman Filter
         Az_m(0) = attitude.x;
         Az_m(1) = attitude.y;
         Az_m(2) = attitude.z;
         Ax_e_ = (dt*F + I_6) * Ax_e ;
         AP_   = (dt*F + I_6) * AP * (dt*F + I_6).transpose() + Tao * AQ * Tao.transpose();
         AK    = AP_ * H.transpose() * (H * AP_ * H.transpose() + AR).inverse();
         Ax_e  = Ax_e_ + AK * (Az_m - H * Ax_e_);
         AP    = (I_6 - AK * H) * AP_;
    }

    void local_position::PositionKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lx_e)
    {
         // Kalman Filter
         Lz_m(0) = position.x;
         Lz_m(1) = position.y;
         Lz_m(2) = position.z;
         Lx_e_ = (dt*F + I_6) * Lx_e ;
         LP_   =(dt*F + I_6) * LP * (dt*F + I_6).transpose() + Tao * LQ * Tao.transpose();
         
         LK    = LP_ * H.transpose() * (H * LP_ * H.transpose() + LR).inverse();
         Lx_e  = Lx_e_ + LK * (Lz_m - H * Lx_e_);
         LP    = (I_6 - LK * H) * LP_;
        //  Lx_e(2) =  Lx_e(2)>min_height ?min_height:Lx_e(2); //
        //  Lx_e(2) =  Lx_e(2)<max_height ?max_height:Lx_e(2);  //height < 0

    }
    void local_position::PositionCompKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lcx_e)
    {
         Lcz_m(0) = position.x;
         Lcz_m(1) = position.y;
         Lcz_m(2) = position.z;
         Lcx_e_ = (dt*F + I_6) * Lcx_e ;
         LcP_   = (dt*F + I_6) * LcP * (dt*F + I_6).transpose() + Tao * LcQ * Tao.transpose();
         LcK    = LcP_ * H.transpose() * (H * LcP_ * H.transpose() + LcR).inverse();
         Lcx_e  = Lcx_e_ + LcK * (Lcz_m - H * Lcx_e_);
         LcP    = (I_6 - LcK * H) * LcP_;
        //  Lcx_e(2) =  Lcx_e(2)>min_height ?min_height:Lcx_e(2);
        //  Lcx_e(2) =  Lcx_e(2)<max_height ?max_height:Lcx_e(2);  //height < 0
    }

    void local_position::MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg)
    {
               
        if(!(msg.markers.empty()))
        {
            ros::Time time_stamped = msg.header.stamp;
            count_markers = msg.markers.size();
            position_dir.x = 0.0; 
            position_dir.y = 0.0; 
            position_dir.z = 0.0; 

            for(int i=0;i<count_markers;i++)
            {
                quat_marker =  msg.markers[i].pose.pose.orientation;
                // The camera coordinate is different from the tag coordiante ,
                //so the quaternion includes this transform and we need to compensate it. 
                local_position::Quat_ComPen_Cam2Tag(quat_marker);
                local_position::Quat2Euler(quat_marker,rpy);
                roll  += rpy.x;
                pitch += rpy.y;
                yaw   += rpy.z; 
            }
            roll  /= count_markers;
            pitch /= count_markers;
            yaw   /= count_markers;
            pitch = pitch - error_pitch_cam; // the camera is not straight down, and is about 80 deg instead of 90 deg vertical to ground.
            yaw = -yaw; // the image is opposite to the real scene. The left in image is the right in bebop, the right in image is the left in bebop.
    
            att_pub.header.frame_id = "bebop_att";
            att_pub.header.stamp = time_stamped;
            att_pub.point.x = roll;
            att_pub.point.y = pitch;
            att_pub.point.z = yaw;
     
            // Kalman Filter
            local_position::AttitudeKalmanFilter(att_pub.point,Ax_e);
            att_smooth.x = low_pass_param_att * att_last.x + (1-low_pass_param_att)*att_smooth.x;
            att_smooth.y = low_pass_param_att * att_last.y + (1-low_pass_param_att)*att_smooth.x;
            att_smooth.z = low_pass_param_att * att_last.z + (1-low_pass_param_att)*att_smooth.x;
            
            att_kf_pub.header.frame_id = "bebop_att_kf";
            att_kf_pub.header.stamp = time_stamped;
            // att_kf_pub.point.x = Ax_e(0);
            // att_kf_pub.point.y = Ax_e(1);
            // att_kf_pub.point.z = Ax_e(2);
            
            att_kf_pub.point = att_smooth;

            euler.x = roll;
            euler.y = pitch;
            euler.z = yaw;
            // euler.x = att_smooth.x;
            // euler.y = att_smooth.y;
            // euler.z = att_smooth.z;
            // euler.x = Ax_e(0);
            // euler.y = Ax_e(1);
            // euler.z = Ax_e(2);

            local_position::Euler2Quat(euler, quat_pub);
            //cout<<"Roll =:"<<euler.x<<"     Pitch =:"<<euler.y<<"     Yaw =:"<<euler.z<<endl;
    
            for(int i=0;i<count_markers;i++)
            {
                //the angle of camera pitch limit is 83.0 deg.
                pos_marker = msg.markers[i].pose.pose.position;
                ID = msg.markers[i].id;
                //marker.x point to the y axis of body coordinate; marker.y point to the -x axis
                /*
                tags coordinate is opposite to the body coordinate, when the front of bebop is point north
                Tags coordinate:
                x
                ^    
                |   0   1   2   3   4   5   6   7   8   9
                |  10  11  12  13  14  15  16  17  18  19
                |  .
                |  .
                |  .
                |  90  91  92  93  94  95  96  97  98  99 
                O-------------------------------------------> y 
                */
                //double limit_pitch_cam = (90-80)/180*3.14159;
                // pos_in_body(0) = (- pos_marker.y)*cos(limit_pitch_cam) + (pos_marker.z)*sin(limit_pitch_cam) + bias_cam;
                // pos_in_body(1) = pos_marker.x;
                // pos_in_body(2) = -(- pos_marker.y)*sin(limit_pitch_cam) + (pos_marker.z)*cos(limit_pitch_cam);
                pos_in_body(0) = (- pos_marker.y);
                pos_in_body(1) = pos_marker.x;
                pos_in_body(2) = pos_marker.z;

                position_dir.x += (-pos_in_body(0) + (9-(int)(ID/10))*dx); 
                position_dir.y += (-pos_in_body(1) + (ID%10)*dy);
                position_dir.z += (-pos_in_body(2)); 

                local_position::Euler2Rota(euler,rot);
                //Quat2Rota(quat_ave,rot);
                //pos_in_tag = rot*pos_in_body;
                pos_in_tag = - rot*pos_in_body;
                // cout<<"****************The Matrix is : "<<rot<<endl;
                // cout<<"################The Relative Position is : "<<pos_in_tag<<endl;
                position_comp.x += (pos_in_tag(0) + (9-(int)(ID/10))*dx); 
                position_comp.y += (pos_in_tag(1) + (ID%10)*dy);
                position_comp.z += (pos_in_tag(2));    
            }

            position_dir.x /= count_markers;
            position_dir.x -= bias_cam; 
            position_dir.y /= count_markers;
            position_dir.z /= count_markers;
            pos_pub.header.frame_id = "bebop_pos";
            pos_pub.header.stamp = time_stamped;
            pos_pub.pose.position = position_dir;
            pos_pub.pose.orientation = quat_pub;

            cout<<"current x:"<<position_dir.x<<"   last x:"<<position_dir_last.x<<endl;
            cout<<"dt:"<<(time_stamped-pos_time_last).toSec();
            velocity_dir.x = (position_dir.x - position_dir_last.x)/(time_stamped-pos_time_last).toSec();
            velocity_dir.y = (position_dir.y - position_dir_last.y)/(time_stamped-pos_time_last).toSec();
            velocity_dir.z = (position_dir.z - position_dir_last.z)/(time_stamped-pos_time_last).toSec();
            velocity_dir.x = low_pass_param_vel * velocity_dir_last.x + (1-low_pass_param_vel) * velocity_dir.x;
            velocity_dir.y = low_pass_param_vel * velocity_dir_last.y + (1-low_pass_param_vel) * velocity_dir.y;
            velocity_dir.z = low_pass_param_vel * velocity_dir_last.z + (1-low_pass_param_vel) * velocity_dir.z;

            vel_pub.header.frame_id="bebop_vel";
            vel_pub.header.stamp = time_stamped;
            vel_pub.point = velocity_dir;
            vel_uav.publish(vel_pub);

            position_dir_last = position_dir;
            velocity_dir_last = velocity_dir;
            pos_time_last = time_stamped;


            local_position::PositionKalmanFilter(position_dir, Lx_e);
            pos_kf_pub.pose.position.x = Lx_e(0);
            pos_kf_pub.pose.position.y = Lx_e(1);
            pos_kf_pub.pose.position.z = Lx_e(2);
            pos_kf_pub.header.frame_id = "bebop_pos_kf";
            pos_kf_pub.header.stamp = time_stamped;
            pos_kf_pub.pose.orientation = quat_pub;



            position_comp.x /= count_markers;
            position_comp.x -= bias_cam; 
            position_comp.y /= count_markers;
            position_comp.z /= count_markers;
            pos_comp_pub.header.frame_id = "bebop_pos_comp";
            pos_comp_pub.header.stamp = time_stamped;
            pos_comp_pub.pose.position = position_comp;
            pos_comp_pub.pose.orientation = quat_pub;
            local_position::PositionCompKalmanFilter(position_comp, Lcx_e);
            pos_comp_kf_pub.pose.position.x = Lcx_e(0);
            pos_comp_kf_pub.pose.position.y = Lcx_e(1);
            pos_comp_kf_pub.pose.position.z = Lcx_e(2);
            pos_comp_kf_pub.header.frame_id = "bebop_pos_comp_kf";
            pos_comp_kf_pub.header.stamp = time_stamped;
            pos_comp_kf_pub.pose.orientation = quat_pub;

        }
        else
            ROS_INFO("No tag pose message!");
    }

    void local_position::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
    {
        double q0 = quat.w;
        double q1 = quat.x;
        double q2 = quat.y;
        double q3 = quat.z;
    
        double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
        double t1 = +2.0 * (q1 * q2 + q0 * q3);
        double t2 = -2.0 * (q1 * q3 - q0 * q2);
        double t3 = +2.0 * (q2 * q3 + q0 * q1);
        double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;
    
        // t2 = t2 > 1.0 ? 1.0 : t2;
        // t2 = t2 < -1.0 ? -1.0 : t2; 
        euler.x = asin(t2);
        euler.y = -atan2(t3, t4);
        euler.z = atan2(t1, t0);
    }
    
    void local_position::Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat)
    {
        double fi  = euler.x / 2;
        double theta = euler.y / 2;
        double psi   = euler.z / 2;
        
        quat.w = cos(fi)*cos(theta)*cos(psi) + sin(fi)*sin(theta)*sin(psi);  
        quat.x = sin(fi)*cos(theta)*cos(psi) - cos(fi)*sin(theta)*sin(psi);
        quat.y = cos(fi)*sin(theta)*cos(psi) + sin(fi)*cos(theta)*sin(psi);
        quat.z = cos(fi)*cos(theta)*sin(psi) - sin(fi)*sin(theta)*cos(psi);
    }
    
    void local_position::Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot)
    {
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
    
        rot(0,0)=1-2*(y*y+z*z);
        rot(0,1)=2*(x*y-z*w);
        rot(0,2)=2*(x*z+y*w);
    
        rot(1,0)=2*(x*y+z*w);
        rot(1,1)=1-2*(x*x+z*z);
        rot(1,2)=2*(y*z-x*w);
    
        rot(2,0)=2*(x*z-y*w);
        rot(2,1)=2*(y*z+x*w);
        rot(2,2)=1-2*(x*x+y*y);
    
    }
    
    void local_position::Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q)
    {
        //cam coordinate and tag coordinate: tag coordinate is rotated -90 deg about the x axis of cam coordinate.
        // compensate the rotation, so rotate 90 deg. angle_compensate = 90, cos(angle_compensate/2) = sin(angle_compensate/2) = 0.707
        // x = 1*sin(angle_compensate/2), y = 0, z = 0, w = cos(angle_compensate/2);
        // quaternion1 * quaternion2 = " you can google ! " 
        
        double x = q.x;
        double y = q.y;
        double z = q.z;
        double w = q.w;
        q.x = 0.707 * (x + w);
        q.y = 0.707 * (y + z);
        q.z = 0.707 * (z - y);
        q.w = 0.707 * (w - x);
    }
    
    void local_position::Euler2Rota(geometry_msgs::Vector3 &euler, Eigen::Matrix3d& rot)
    {
        double fi = euler.x;
        double theta = euler.y;
        double psi = euler.z;
        Eigen::Matrix3d Rx(3,3);
        Eigen::Matrix3d Ry(3,3);
        Eigen::Matrix3d Rz(3,3);
        Rz<<cos(psi), sin(psi), 0.0,
            -sin(psi), cos(psi), 0.0,
            0.0, 0.0, 1.0;   
        Ry<<cos(theta), 0.0, -sin(theta),
            0.0, 1.0, 0.0,
            sin(theta), 0.0, cos(theta);
        Rx<<1.0, 0.0, 0.0,
            0.0, cos(fi), sin(fi),
            0.0, -sin(fi), cos(fi);
        rot = Rz*Ry*Rx;
    }

}