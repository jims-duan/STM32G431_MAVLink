#ifndef _ODOM_GPS_H_
#define _ODOM_GPS_H_

#include "main.h"

// UBX_NAV消息长度
#define UBX_NAV_DOP_LEN         26
#define UBX_NAV_POSLLH_LEN      36
#define UBX_NAV_SOL_LEN         60
#define UBX_NAV_STATUS_LEN      24
#define UBX_NAV_VELNED_LEN      44

typedef struct
{
    uint8_t crc_a;
    uint8_t crc_b;
} CompassCRC;

typedef struct
{
    int week;
    long week_millis;
} Week_Date;

typedef struct
{
    double llh_lon;
    double llh_lat;
    double llh_alt;

    double speed;
    double gspeed;
    double heading;
} gps_structure_data;
extern gps_structure_data gps_data;

typedef struct
{
    double x;
    double y;
    double z;
} ECEF;

typedef struct
{
    double vN; // 北向速度 m/s
    double vE; // 东向速度 m/s
    double vU; // 垂直向上 m/s
} NEDVel;

typedef struct
{
    double vx;
    double vy;
    double vz;
} ECEFVel;

typedef struct
{
    uint32_t compass_tick;
    uint32_t nav_100ms_tick;
    uint32_t nav_1000ms_tick;
} Odom_GPS_FSM_Structure;
extern Odom_GPS_FSM_Structure odom_gps_fsm_struct;

void Odom_GPS_FSM_Run(uint32_t tick);



#endif
