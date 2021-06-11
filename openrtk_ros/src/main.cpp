/*******************************************************
 * @file main.cpp
 * @author khshen (khshen@aceinna.com)
 * @brief 
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/
#include "driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aceinna_rtk_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //Info  Debug   Warn, Error,Fatal,
    
    RTKDriver* pdriver = new RTKDriver(nh);
    pdriver->Start();
    // ros::spin();
    SAFEDELETE(pdriver);
    return 0;
}
