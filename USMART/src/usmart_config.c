#include "usmart.h"
#include "usmart_str.h" 

////////////////////////////用户配置区///////////////////////////////////////////////
//这下面要包含所用到的函数所申明的头文件(用户自己添加) 
#include "main.h"
#include <stdlib.h>
#include "uav_pid.h"


void usmart_xy_p_pid(char *p,char *i,char *d)
{
	pid_init(&huav_pid.PositionPID[0], (float)atof(p), (float)atof(i), (float)atof(d));
	pid_init(&huav_pid.PositionPID[1], (float)atof(p), (float)atof(i), (float)atof(d));
}
void usmart_xy_s_pid(char *p,char *i,char *d)
{
	pid_init(&huav_pid.SpeedPID[0], (float)atof(p), (float)atof(i), (float)atof(d));
	pid_init(&huav_pid.SpeedPID[1], (float)atof(p), (float)atof(i), (float)atof(d));
}

void usmart_z_p_pid(char *p,char *i,char *d)
{
	pid_init(&huav_pid.PositionPID[2], (float)atof(p), (float)atof(i), (float)atof(d));
}
void usmart_z_s_pid(char *p,char *i,char *d)
{
	pid_init(&huav_pid.SpeedPID[2], (float)atof(p), (float)atof(i), (float)atof(d));
}
//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//如果使能了读写操作
	{(void*)read_addr,(const uint8_t*)"u32 read_addr(u32 addr)"},
	{(void*)write_addr,(const uint8_t*)"void write_addr(u32 addr,u32 val)}"},	 
#endif
	// (void*)delay_ms,"void delay_ms(uint16_t nms)",
	{(void*)usmart_xy_p_pid,	(const uint8_t*)"void usmart_xy_p_pid(char *p,char *i,char *d)"},
	{(void*)usmart_xy_s_pid,	(const uint8_t*)"void usmart_xy_s_pid(char *p,char *i,char *d)"},
	{(void*)usmart_z_p_pid,	(const uint8_t*)"void usmart_z_p_pid(char *p,char *i,char *d)"},
	{(void*)usmart_z_s_pid,	(const uint8_t*)"void usmart_z_s_pid(char *p,char *i,char *d)"},
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//函数控制管理器初始化
//得到各个受控函数的名字
//得到函数总数量
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//函数数量
	0,	  	//参数数量
	0,	 	//函数ID
	1,		//参数显示类型,0,10进制;1,16进制
	0,		//参数类型.bitx:,0,数字;1,字符串	    
	{0},	  	//每个参数的长度暂存表,需要MAX_PARM个0初始化
	{0},		//函数的参数,需要PARM_LEN个0初始化
};   


