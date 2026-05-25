#ifndef _CONFIG_H
#define _CONFIG_H

#include "main.h"

/*---------------------------模拟GPS模块参数---------------------------*/
#define GPS_POSITION_K        1.0f           // 里程计到GPS的位置系数
#define GPS_VELOCITY_K        1.0f           // 里程计到GPS的速度系数
/*---------------------------模拟GPS模块参数---------------------------*/


/*----------------------------PID模块参数-----------------------------*/
#define PID_XY_POSITION_P     1.5f           // xy位置环PID P系数
#define PID_XY_POSITION_I     0.0f           // xy位置环PID I系数
#define PID_XY_POSITION_D     0.0f           // xy位置环PID D系数

#define PID_XY_VELOCITY_P   230.0f           // xy速度环PID P系数
#define PID_XY_VELOCITY_I     0.12f           // xy速度环PID I系数
#define PID_XY_VELOCITY_D   1380.0f           // xy速度环PID D系数
/////////////////////////////////////////////////////////////
#define PID_Z_POSITION_P      370.0f           // z位置环PID P系数
#define PID_Z_POSITION_I      0.21f           // z位置环PID I系数
#define PID_Z_POSITION_D      1360.0f           // z位置环PID D系数

#define PID_Z_VELOCITY_P    300.0f           // z速度环PID P系数
#define PID_Z_VELOCITY_I      0.0f           // z速度环PID I系数
#define PID_Z_VELOCITY_D      0.0f           // z速度环PID D系数
/*----------------------------PID模块参数-----------------------------*/


/*---------------------------MavLink模块参数--------------------------*/
#define ODOM_POSITION_FILTER  0.8f           // 里程计位置滤波(一阶低通)系数
#define ODOM_VELOCITY_FILTER  0.6f           // 里程计速度滤波(一阶低通)系数
/*---------------------------MavLink模块参数--------------------------*/


#endif

