#include "bebop_pos_ctrl.h"

namespace Bebop_Ctrl
{
    bebop_pos_ctrl::bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh):nh_(nh)
    {
        pnh.param("time_hover", time_hover, 20000); 
        pnh.param("debug", debug, true); 

        pnh.param("K_p_x", K_p_x, 0.2); 
        pnh.param("K_p_y", K_p_y, 0.2); 
        pnh.param("K_p_z", K_p_z, 0.2); 
        pnh.param("K_p_yaw", K_p_yaw, 0.2);

        pnh.param("K_i_x", K_i_x, 0.001); 
        pnh.param("K_i_y", K_i_y, 0.001); 
        pnh.param("K_i_z", K_i_z, 0.001); 
        pnh.param("K_i_yaw", K_i_yaw, 0.001);

        pnh.param("K_d_x", K_d_x, 0.001); 
        pnh.param("K_d_y", K_d_y, 0.001); 
        pnh.param("K_d_z", K_d_z, 0.001); 
        pnh.param("K_d_yaw", K_d_yaw, 0.001);

        pnh.param("MaxV_xy", MaxV_xy, 0.3); 
        pnh.param("MaxV_z", MaxV_z, 0.2); 
        pnh.param("MaxV_yaw", MaxV_yaw, 0.5);

        pnh.param("Limit_xy_error_int", Limit_xy_error_int, 0.3); 
        pnh.param("Limit_z_error_int", Limit_z_error_int, 0.2); 
        pnh.param("Limit_yaw_error_int", Limit_yaw_error_int, 0.5);

        pnh.param("Tolerance_hori_pos", Tolerance_hori_pos, 0.1); 
        pnh.param("Tolerance_vert_pos", Tolerance_vert_pos, 0.1); 
        pnh.param("Tolerance_yaw_rad", Tolerance_yaw_rad, 0.1);

        pnh.param("Hz", Hz, 20); 
        pnh.param("Tag_or_Mocap", Tag_or_Mocap, 1.0);
        pnh.param<std::string>("trajec_filepath", trajec_filepath, "/home/uav/bebop_ws/src/bebop_pos_ctrl/config/list.yaml");
        std::cout<<"trajec_filepath="<<trajec_filepath<<std::endl;

        



        bebop_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
        get_marker_pose = nh_.subscribe("/pos_uav",2,&bebop_pos_ctrl::BebopPoseCallback,this);
        get_velocity = nh_.subscribe("/vel_uav",2,&bebop_pos_ctrl::BebopVelCallback,this);
        usleep(50000);  // 10000ms can not receive correct data
        ros::spinOnce();

        // for pid controller
        last_pose= pos_sub;
        bebop_pos_ctrl::fillPatrolList();

        VecMovIt it = patrol_list_.begin();
        for(int i = 0; i < num_point; i++)
        {
            bebop_pos_ctrl::PIDinit();
            geometry_msgs::Twist temp_goal = patrol_list_.back();// remember the position arrived
            patrol_list_.pop_back();
            if(debug || 1)
            {
                cout<<"current target position =:"<<endl;
                cout <<"Target_pos_x =:"<< temp_goal.linear.x << "  ";
                cout <<"Target_pos_y =:"<< temp_goal.linear.y << "  ";
                cout <<"Target_pos_z =:"<<  temp_goal.linear.z << "  ";
                cout <<"Angle_yaw =:"<<  temp_goal.angular.z << endl;
            }
            bebop_pos_ctrl::Control2Goal(temp_goal);
        }

    }

    void bebop_pos_ctrl::fillPatrolList()
    {
        geometry_msgs::Twist temp_goal;
       //cv::FileStorage fs("/home/uav/bebop_ws/src/bebop_pos_ctrl/config/list.yaml", cv::FileStorage::READ);
       
       cv::FileStorage fs(trajec_filepath, cv::FileStorage::READ);
        if( !fs.isOpened() ) // if we have file with parameters, read them
        {
            cout<<"ERROR, cannot open list.yaml!"<<endl;
            cout<<"Please check the filepath of list.yaml!"<<endl;
        }
        cv::FileNode list_n = fs["features"];
        cv::FileNodeIterator it = list_n.begin(), it_end = list_n.end();
        num_point = 0;
        for (; it != it_end; ++it)
        {
            num_point ++;
            temp_goal.linear.x   = (double)(*it)["x"];
            temp_goal.linear.y   = (double)(*it)["y"];
            temp_goal.linear.z   = (double)(*it)["z"];
            temp_goal.angular.x   = 0.0;
            temp_goal.angular.y   = 0.0;
            temp_goal.angular.z   = (double)(*it)["yaw"];
            if(debug)
            {
                cout<<"Position "<<num_point<<" =:"<<endl;
                cout <<"Pos_x =:"<< (double)(*it)["x"] << "  ";
                cout <<"Pos_y =:"<< (double)(*it)["y"] << "  ";
                cout <<"Pos_z =:"<<  (double)(*it)["z"] << "  ";
                cout <<"Angle_yaw =:"<<  (double)(*it)["yaw"] << endl;
            }
           
            
            patrol_list_.push_back(temp_goal);
        }
        reverse(patrol_list_.begin(), patrol_list_.end());
        fs.release();
    }

    void bebop_pos_ctrl::Control2Goal(const geometry_msgs::Twist& goal_pose)
    {
        cout<<"entered Control2Goal!"<<endl;
        double get_x ,get_y,get_z, get_yaw; //yaw rad
        double set_x ,set_y,set_z, set_yaw; //yaw rad
        geometry_msgs::Vector3 get_euler;
        ros::Rate loopRate(Hz);
        bebop_pos_ctrl::PIDinit();

        while(ros::ok())
        {
            get_marker_pose = nh_.subscribe("/pos_uav",2,&bebop_pos_ctrl::BebopPoseCallback,this);
            get_velocity = nh_.subscribe("/vel_uav",2,&bebop_pos_ctrl::BebopVelCallback,this);
            usleep(40000);  // 10000ms can not receive correct data
            ros::spinOnce();

            get_x = pos_sub.pose.position.x;
            get_y = pos_sub.pose.position.y;
            get_z = pos_sub.pose.position.z;
            
            bebop_pos_ctrl::Quat2Euler(pos_sub.pose.orientation, get_euler); // rad
            //get_yaw = get_euler.z;
            get_yaw = Tag_or_Mocap* get_euler.z; // adapted to otion capture 
            if(debug)
            {
                cout<<"get_x: "<<get_x<<"     get_y: "<<get_y<<"     get_z: "<<get_z<<"    yaw(rad): "<<get_yaw<<endl;
            }
   
            set_x = goal_pose.linear.x;
            set_y = goal_pose.linear.y;
            set_z = goal_pose.linear.z;
            set_yaw = goal_pose.angular.z;


            // if((abs(get_x) < max_x && abs(get_x) > min_x) && (abs(get_y) < max_y && abs(get_y) > min_y)
            //     && (abs(get_z) < max_height && abs(get_z) > min_height))
            if(1)
            {
                if(abs(get_x - set_x)<Tolerance_hori_pos && abs(get_y - set_y)<Tolerance_hori_pos && \
                abs(get_z-set_z)<Tolerance_vert_pos  && abs(get_yaw-set_yaw)< Tolerance_yaw_rad)
                {
                    cout<<"Jump out the goal control loop!"<<endl;
                    break;
                }
                else
                {      
                    cmd_vel_pub.linear.x = 0;
                    cmd_vel_pub.linear.y = 0; 
                    cmd_vel_pub.linear.z = 0;
                    cmd_vel_pub.angular.x = 0;
                    cmd_vel_pub.angular.y = 0;
                    cmd_vel_pub.angular.z = 0;
                    bebop_pos_ctrl::PIDPosControl(goal_pose, pos_sub, vel_sub, cmd_vel_pub);  
                    bebop_cmd_vel.publish(cmd_vel_pub);
                }
            }
            else
            {
                geometry_msgs::Twist patrol_pose;
                patrol_pose.linear.x = get_x;
                patrol_pose.linear.y = get_y;
                patrol_pose.linear.z = patrol_height;
                patrol_pose.angular.x = 0.0;
                patrol_pose.angular.y = 0.0;
                patrol_pose.angular.z = get_yaw;
                bebop_pos_ctrl::PIDPosControl(patrol_pose, pos_sub, vel_sub, cmd_vel_pub);  
                bebop_cmd_vel.publish(cmd_vel_pub);
            }
            loopRate.sleep();
        }
        cout<<"leave Control2Goal!"<<endl;
    }

    void bebop_pos_ctrl::PIDinit()
    {
        error_x_last=0.0;
        error_y_last=0.0;
        error_z_last=0.0;
        error_yaw_last=0.0;
        error_x_currect=0.0;
        error_y_currect=0.0;
        error_z_currect=0.0;
        error_yaw_currect=0.0;
        error_x_accu=0.0;
        error_y_accu=0.0;
        error_z_accu=0.0;
        error_yaw_accu=0.0;
    }

    void bebop_pos_ctrl::PIDPosControl(const  geometry_msgs::Twist& goal_pose_, 
        const  geometry_msgs::PoseStamped& current_pose_, 
        const  geometry_msgs::PointStamped& current_vel_, 
        geometry_msgs::Twist& velocity_ctrl_)    
    {
              // IN BODY FRAME
        double vel_x = current_vel_.point.x;
        double vel_y = current_vel_.point.y;
        double vel_z = current_vel_.point.z;

        geometry_msgs::Quaternion tmp_q = current_pose_.pose.orientation;
        bebop_pos_ctrl::Quat2Euler(tmp_q, euler_current); // rad
        // yaw_current=euler_current.z;
        yaw_current= Tag_or_Mocap* euler_current.z; //adjusted for motion capture

        error_x_currect = K_p_x* (goal_pose_.linear.x - current_pose_.pose.position.x) - K_d_x*vel_x;
        error_y_currect = K_p_y* (goal_pose_.linear.y - current_pose_.pose.position.y) - K_d_y*vel_y;

        velocity_ctrl_.linear.x = cos(yaw_current) * error_x_currect + sin(yaw_current) * error_y_currect;
        velocity_ctrl_.linear.y = -sin(yaw_current) * error_x_currect + cos(yaw_current) * error_y_currect;
        velocity_ctrl_.linear.z = K_p_z*(goal_pose_.linear.z - current_pose_.pose.position.z) - K_d_z*vel_z;
        velocity_ctrl_.angular.z = K_p_yaw*(goal_pose_.angular.z - yaw_current);

        bebop_pos_ctrl::Limitator(velocity_ctrl_.linear.x, velocity_ctrl_.linear.y, 
            velocity_ctrl_.linear.z, velocity_ctrl_.angular.z); 
        
        if(debug)
        {
            cout<<"current control velocity =:"<<endl;
            cout <<"cmd_Vx =:"<< velocity_ctrl_.linear.x << "  ";
            cout <<"cmd_Vy =:"<< velocity_ctrl_.linear.y << "  ";
            cout <<"cmd_Vz =:"<<  velocity_ctrl_.linear.z << "  ";
            cout <<"cmd_Vyaw =:"<<  velocity_ctrl_.angular.z << endl;    
        }
        
        /*
        linear.x (+)  fly forward
                (-)  fly backward
        linear.y (+)  fly left
                (-)  fly right
        linear.z (+)  fly up
                (-)  fly down
        angular.z(+)  rotate counter clockwise    
                (-)  rotate clockwise 
        roll_degree = linear.y * max_tilt_angle 
        pitch_degree = linear.x * max_tilt_angle 
        ver_vel_m_per_s = linear.z * max_vert_speed 
        rot_vel_deg_per_s = angular.z * max_rot_speed       
        */
        velocity_ctrl_.linear.y= - velocity_ctrl_.linear.y;
        velocity_ctrl_.linear.z= - velocity_ctrl_.linear.z;
        velocity_ctrl_.angular.x = 0.0;
        velocity_ctrl_.angular.y = 0.0;
        velocity_ctrl_.angular.z = - velocity_ctrl_.angular.z;
        last_pose = current_pose_;

    }




    void bebop_pos_ctrl::BebopPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pos_sub = *msg;
    }

    void bebop_pos_ctrl::BebopVelCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        vel_sub = *msg;
    }

    void bebop_pos_ctrl::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
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
        //t2 = t2 > 1.0 ? 1.0 : t2;
        //t2 = t2 < -1.0 ? -1.0 : t2;
        // cout<<"pitch =  :"<< asin(t2)<<endl;
        // cout<<"roll  =  :"<< atan2(t3, t4)<<endl;
        // cout<<"yaw   =  :"<< atan2(t1, t0)<<endl;
        euler.x = asin(t2);
        euler.y = -atan2(t3, t4);
        euler.z = atan2(t1, t0);
    }

    void bebop_pos_ctrl::Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat)
    {
        double fi  = euler.x / 2;
        double theta = euler.y / 2;
        double psi   = euler.z / 2;  
        quat.w = cos(fi)*cos(theta)*cos(psi) + sin(fi)*sin(theta)*sin(psi);  
        quat.x = sin(fi)*cos(theta)*cos(psi) - cos(fi)*sin(theta)*sin(psi);
        quat.y = cos(fi)*sin(theta)*cos(psi) + sin(fi)*cos(theta)*sin(psi);
        quat.z = cos(fi)*cos(theta)*sin(psi) - sin(fi)*sin(theta)*cos(psi);
    }
    void bebop_pos_ctrl::Limitator(double& vx, double& vy, double& vz, double& yawrate)
    {
        if(vx < (- MaxV_xy))
            vx = - MaxV_xy;
        if(vx > MaxV_xy)
            vx = MaxV_xy;

        if(vy < (- MaxV_xy))
            vy = - MaxV_xy;
        if(vy > MaxV_xy)
            vy = MaxV_xy; 

        if(vz < (- MaxV_z))
            vz = - MaxV_z;
        if(vz > MaxV_z)
            vz = MaxV_z;

        if(yawrate < (- MaxV_yaw))
            yawrate = - MaxV_yaw;
        if(yawrate > MaxV_yaw)
            yawrate = MaxV_yaw;     
    }

}