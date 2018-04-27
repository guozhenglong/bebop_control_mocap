#include "bebop_pos_ctrl.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop_pos_ctrl_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Bebop_Ctrl::bebop_pos_ctrl Bebop_Control(nh, pnh);
    return 0;
}

