/*******************************************************
 * @file driver.h
 * @author khshen (khshen@aceinna.com)
 * @brief 
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/

#pragma once
#include <iostream>
using namespace std;
#include <queue>
#include <mutex>
#include <thread>
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "serial/serial.h"
#include "rtk.h"
#include "macro.h"
#include "protocol.h"
#include "openrtk_msg/openrtk_imu.h"
#include "openrtk_msg/openrtk_gnss.h"
#include "openrtk_msg/openrtk_ins.h"
#include <sys/socket.h>     
#include <netinet/in.h>     
#include <arpa/inet.h>  

class RTKDriver
{
public:
    RTKDriver(ros::NodeHandle nh);
    virtual ~RTKDriver();

    void Start();
    void Stop();
    bool Spin();

    static void SigintHandler(int sig);
    void ThreadGetDataUart(void);
    void ThreadGetDataEth(void);
    //void ThreadGetDataCAN(void);

    void ParseFrame(uint8_t* frame, uint16_t len);
    void Handle_RtkGNSSMessage(uint8_t* frame, uint16_t len);
    void Handle_RtkIMUMessage(uint8_t* frame, uint16_t len);
    void Handle_RtkINSMessage(uint8_t* frame, uint16_t len);    /* Process INS Msg */

    uint16_t calcCRC(uint8_t *ptr, uint32_t num);
    string Bytestohexstring(uint8_t* bytes,int bytelength);

private:
    ros::NodeHandle m_nh;
    ros::Publisher rtk_pub_imu;
    ros::Publisher rtk_pub_gnss;
    ros::Publisher rtk_pub_ins;
    CRTK m_rtk;
    string m_topic;
    serial::Serial* m_pserial;

    /*******Eth Port******/
    struct sockaddr_in addr_sensor;  
    struct sockaddr_in addr_server; 
    uint32_t sockstrlen;
    int32_t sock_Cli; 
    int32_t sock_Ser; 
    /*******Eth Port******/

    std::mutex m_mt_buf;
    queue<uint8_t> m_uartBuf;
    queue<uint8_t> m_ethBuf;

    bool m_uartBexit;
    bool m_EthBexit;
    std::mutex m_uart_exit;
    std::mutex m_eth_exit;
    std::thread m_GetUartDataThread;
    std::thread m_GetEthDataThread;
    std::thread m_GetCANDataThread;
};

