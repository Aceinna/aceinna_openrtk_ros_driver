/*******************************************************
 * @file protocol.h
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

#pragma pack(1)

struct S_a1
{
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    corrRates[3];
    float    accels[3];
    uint8_t  ekfOpMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_s1
{
    uint32_t tstmp;
    double   dbTstmp;
    float    accel_g[3];
    float    rate_dps[3];
    float    mag_G[3];
    float    temp_C;
};

struct S_e1
{
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    mags[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_e2
{
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    accelBias[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    velocity[3];
    float    mags[3];
    double   pos[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
};

struct S_a2
{
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    yaw;
    float    corrRates[3];
    float    accels[3];
};

/*****IMU Data Pack In RTK*******/
typedef struct RTKIMU
{
    uint16_t    gps_week;           /* [Name]:GPS WEEK; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    uint32_t    gps_millisecs;      /* [Name]:GPS Seconds in Week; [Unit]:ms; [Range]:NULL; [InitialValue]: NULL */
    float       x_acceleration;     /* [Name]:x Acceletion; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       y_acceleration;     /* [Name]:y Acceletion; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       z_acceleration;     /* [Name]:z Acceletion; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       x_gyro_rate;        /* [Name]:x Angular velocity; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       y_gyro_rate;        /* [Name]:y Angular velocity; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       z_gyro_rate;        /* [Name]:z Angular velocity; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */    
}stRTKIMUS1;

/*** RTK IMU Message Struct***/
typedef struct RTKIMU_MSG
{
    uint8_t     Header[2];              /* IMU Message Header: 0x55, 0x55 */
    uint8_t     FrameType[2];           /* Frame Type, IMU Message: 0x73, 0x31 ("s1") */
    uint8_t     FrameLen;               /* IMU Message Len : 30 */
    stRTKIMUS1  Msg;                    /* IMU Message Buffer */
    uint8_t     Crc[2];                 /* IMU Messge CRC: CRC_L, CRC_H */
}stRTKIMU_MSG;

/******GNSS Data Pack In RTK******/
typedef struct RTKGNSS
{
    uint16_t    gps_week;           /* [Name]:GPS WEEK; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    uint32_t    gps_millisecs;      /* [Name]:GPS Seconds in Week; [Unit]:ms; [Range]:NULL; [InitialValue]: NULL */
    uint8_t     position_type;      /* [Name]:Position Mode; [Unit]:NULL; [Range]:0: NULL; 1:Single Point; 
                                                4:Fixed; 5: Floating Point; [InitialValue]: NULL */
    double      latitude;           /* [Name]:latitude ; [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      longitude;          /* [Name]:longitude; [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      height;             /* [Name]:height;    [Unit]:m; [Range]:NULL; [InitialValue]: NULL */
    float       latitude_std_deviation;       /* [Name]:latitude standard deviation; [Unit]:m; [Range]:NULL; [InitialValue]: NULL  */
    float       longitude_std_deviation;      /* [Name]:longitude standard_deviation; [Unit]:m; [Range]:NULL; [InitialValue]: NULL  */
    float       height_std_deviation;         /* [Name]:height standard deviation; [Unit]:m; [Range]:NULL; [InitialValue]: NULL  */
    uint8_t     num_of_satellites;            /* [Name]:number of satellites Received; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL  */
    uint8_t     num_satellite_in_solution;    /* [Name]:Number of satellites used for solving ; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL  */
    float       hdop;               /* [Name]:Horizontal component precision factor ; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    float       diffage;            /* [Name]:Differential time difference ; [Unit]:s; [Range]:NULL; [InitialValue]: NULL */
    float       north_vel;          /* [Name]:North speed ; [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL */
    float       east_vel;           /* [Name]:East speed ;  [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */
    float       up_vel;             /* [Name]:Sky speed ;   [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */
    float       north_vel_std_deviation;        /* [Name]:North speed standard deviation ; [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
    float       east_vel_std_deviation;         /* [Name]:East speed standard deviation ;  [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
    float       up_vel_std_deviation;           /* [Name]:Sky speed standard deviation ;   [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
}stRTKGNSSG1;

/*** RTK GNSS Message Struct***/
typedef struct RTKGNSS_MSG
{
    uint8_t     Header[2];              /* GNSS Message Header: 0x55, 0x55 */
    uint8_t     FrameType[2];           /* Frame Type, GNSS Message: 0x67, 0x31 ("g1") */
    uint8_t     FrameLen;               /* GNSS Message Len : 77 */
    stRTKGNSSG1 Msg;                    /* GNSS Message Buffer */
    uint8_t     Crc[2];                 /* GNSS Messge CRC: CRC_L, CRC_H */
}stRTKGNSS_MSG;

/******INS Data Pack In RTK******/
typedef struct RTKINS
{
    uint16_t    gps_week;           /* [Name]:GPS WEEK; [Unit]:NULL; [Range]:NULL; [InitialValue]: NULL */
    uint32_t    gps_millisecs;      /* [Name]:GPS Seconds in Week; [Unit]:ms; [Range]:NULL; [InitialValue]: NULL */
    uint8_t     ins_status;         /* [Name]:INS State; [Unit]:NULL; [Range]:0: NULL; 1:In Alignment ; 2:solution unreliable ; 
                                                3: solution good; 4: Pure INS solution ;[InitialValue]: NULL */
    uint8_t     ins_position_type;  /* [Name]:INS Type ; [Unit]:deg; [Range]:0: NULL; 1:Pseudo-range single point and INS; 4:RTK fixed solution and INS; 
                                                5: RTK float point solution and INS; [InitialValue]: NULL */
    double      latitude;           /* [Name]:latitude;    [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      longitude;          /* [Name]:longitude;   [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      height;             /* [Name]:height;      [Unit]:m;   [Range]:NULL; [InitialValue]: NULL */
    double      north_vel;          /* [Name]:North speed; [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL */
    double      east_vel;           /* [Name]:East speed;  [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */
    double      up_vel;             /* [Name]:Sky speed;   [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */
    double      roll;               /* [Name]:Roll Angle;  [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      pitch;              /* [Name]:Pitch Angle; [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    double      heading;            /* [Name]:Yaw Angle;   [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */
    float       latitude_std_deviation;    /* [Name]:latitude standard deviation;  [Unit]:m;    [Range]:NULL; [InitialValue]: NULL  */
    float       longitude_std_deviation;   /* [Name]:longitude standard_deviation; [Unit]:m;    [Range]:NULL; [InitialValue]: NULL  */
    float       height_std_deviation;      /* [Name]:height standard deviation;    [Unit]:m;    [Range]:NULL; [InitialValue]: NULL  */
    float       north_vel_std_deviation;   /* [Name]:North speed standard deviation; [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
    float       east_vel_std_deviation;    /* [Name]:East speed standard deviation;  [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
    float       up_vel_std_deviation;      /* [Name]:Sky speed standard deviation;   [Unit]:m/s; [Range]:NULL; [InitialValue]: NULL  */      
    float       roll_std_deviation;        /* [Name]:North speed standard deviation; [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */      
    float       pitch_std_deviation;       /* [Name]:East speed standard deviation;  [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */      
    float       heading_std_deviation;     /* [Name]:Sky speed standard deviation;   [Unit]:deg; [Range]:NULL; [InitialValue]: NULL  */      
}stRTKINSI1;

/*** RTK INS Message Struct***/
typedef struct RTKINS_MSG
{
    uint8_t     Header[2];              /* INS Message Header: 0x55, 0x55 */
    uint8_t     FrameType[2];           /* Frame Type, INS Message: 0x69, 0x31 ("i1") */
    uint8_t     FrameLen;               /* INS Message Len : 116 */
    stRTKINSI1  Msg;                    /* INS Message Buffer */
    uint8_t     Crc[2];                 /* INS Messge CRC: CRC_L, CRC_H */
}stRTKINS_MSG;

#pragma pack ()

