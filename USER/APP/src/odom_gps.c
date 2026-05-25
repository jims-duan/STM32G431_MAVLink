#include "odom_gps.h"
#include "system.h"
#include <math.h>
#include "MavLinkAPP.h"
#include "usart.h"
#include "sbus.h"
#include "ring_buffer.h"
#include "usart_it.h"
#include "arm_math.h"

// GPS发送缓冲区
uint8_t gps_send_buffer[1024];

gps_structure_data gps_data;
Odom_GPS_FSM_Structure odom_gps_fsm_struct = {0};

uint8_t* calculateMagneticFieldFromHeading(double yaw);
uint8_t* buildNAV_VELNED(uint32_t iTOW, int32_t velN, int32_t velE, int32_t velD,
                                                  uint32_t speed, uint32_t gSpeed, int32_t heading,
                                                  uint32_t sAcc, uint32_t cAcc);
uint8_t* buildNAV_STATUS(uint32_t iTOW, uint8_t gpsFix, uint8_t flags,
                                                  uint8_t fixStat, uint8_t flags2,
                                                  uint32_t ttff, uint32_t msss);
uint8_t* buildNAV_SOL(uint32_t iTOW, int32_t fTOW, int16_t week,
                                               uint8_t gpsFix, uint8_t flags,
                                               int32_t ecefX, int32_t ecefY, int32_t ecefZ,
                                               uint32_t pAcc,
                                               int32_t ecefVX, int32_t ecefVY, int32_t ecefVZ,
                                               uint32_t sAcc, uint16_t pDOP, uint8_t numSV);
ECEFVel nedToEcefVel(double lon, double lat, const NEDVel* vel);
ECEF llhToEcef(double lon, double lat, double alt);
void convertSLAMToGPS(double slam_x, double slam_y, double slam_z,
                                   int32_t* longitude, int32_t* latitude,
                                   int32_t* height, int32_t* hMSL);
uint8_t* buildNAV_POSLLH(uint32_t iTOW, int32_t longitude, int32_t latitude,
                            int32_t height, int32_t hMSL, uint32_t hAcc, uint32_t vAcc);
uint8_t* buildNAV_DOP(uint32_t iTOW, uint16_t gDOP, uint16_t pDOP, 
    uint16_t tDOP, uint16_t vDOP, uint16_t hDOP, 
    uint16_t nDOP, uint16_t eDOP);
Week_Date weeksBetweenDates(void);


void Odom_GPS_FSM_Run(uint32_t tick)
{
    Week_Date _week;
    static uint8_t gps_on = 0;

    if (sbus_encode_frame_t.channels[9] == 1946)    // 打开GPS开关(B)
    {
        gps_on = 1;
    }
    else
    {        
        gps_on = 0;
    }

    uint16_t size = RingBuff_GetSize(&u1TxRingBufferStruct);
    if (size > 0 && (USART1_Struct.TxCompleteFlag == 1))
    {
        uint16_t read_len = RingBuff_ReadBytes(&u1TxRingBufferStruct, gps_send_buffer, size);

        USART1_Struct.TxCompleteFlag = 0;
        HAL_UART_Transmit_DMA(&huart1, gps_send_buffer, read_len);
    }

    if ((get_tick_diff(tick, odom_gps_fsm_struct.compass_tick) >= 33) && gps_on)
    {
        odom_gps_fsm_struct.compass_tick = tick;

        // 计算磁场分量并构建罗盘数据帧
        uint8_t* compass_frame = calculateMagneticFieldFromHeading(mavlink_struct.odom_yaw_deg);
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,compass_frame,12);
        // HAL_UART_Transmit(&huart1, compass_frame, 12, HAL_MAX_DELAY); // 发送12字节的罗盘数据帧
    }

    if ((get_tick_diff(tick, odom_gps_fsm_struct.nav_100ms_tick) >= 20) && gps_on)
    {
        _week = weeksBetweenDates(); // 获取周时间

        odom_gps_fsm_struct.nav_100ms_tick = tick;

        // 重新合成帧
        uint8_t* rebuilt_frame;

        // 从SLAM坐标转换为GPS坐标
        int32_t longitude, latitude, height, hMSL;
        convertSLAMToGPS(
            mavlink_struct.odom_position_x * GPS_POSITION_K, 
            mavlink_struct.odom_position_y * GPS_POSITION_K, 
            mavlink_struct.odom_position_z * GPS_POSITION_K,
            &longitude, &latitude, &height, &hMSL);
        
        gps_data.llh_lon = longitude * 1e-7;
        gps_data.llh_lat = latitude * 1e-7;
        gps_data.llh_alt = hMSL / 1000.0;

        // 重新合成帧（使用转换后的GPS坐标）
        rebuilt_frame = buildNAV_POSLLH(
            _week.week_millis, 
            longitude, latitude,
            height, hMSL,
            0.416/0.001, 0.523/0.001);  // 水平精度和垂直精度
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,rebuilt_frame, UBX_NAV_POSLLH_LEN);
        // HAL_UART_Transmit(&huart1, rebuilt_frame, UBX_NAV_POSLLH_LEN, HAL_MAX_DELAY);

        ECEF ecef = llhToEcef(gps_data.llh_lon,gps_data.llh_lat,gps_data.llh_alt);
        NEDVel ned_v;
        ned_v.vN = mavlink_struct.odom_velocity_y * GPS_VELOCITY_K;
        ned_v.vE = mavlink_struct.odom_velocity_x * GPS_VELOCITY_K;
        ned_v.vU = mavlink_struct.odom_velocity_z * GPS_VELOCITY_K;
        ECEFVel ecef_v = nedToEcefVel(gps_data.llh_lon,gps_data.llh_lat, &ned_v);

        // 重新合成帧
        rebuilt_frame = buildNAV_SOL(
            _week.week_millis ,      // iTOW
            0,              // fTOW
            _week.week ,           // week
            0x03,              // gpsFix (3D定位)
            0xDF,           // flags
            ecef.x*100,     // ecefX (cm)
            ecef.y*100,     // ecefY (cm)
            ecef.z*100,     // ecefZ (cm)
            0.5*100,            // pAcc (cm)
            ecef_v.vx*100,            // ecefVX (cm/s)
            ecef_v.vy*100,            // ecefVY (cm/s)
            ecef_v.vz*100,            // ecefVZ (cm/s)
            0.11*100,             // sAcc (cm/s)
            0.77*100,            // pDOP
            38              // numSV
        );
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,rebuilt_frame, UBX_NAV_SOL_LEN);
        // HAL_UART_Transmit(&huart1, rebuilt_frame, UBX_NAV_SOL_LEN, HAL_MAX_DELAY);

        // 重新合成帧
        rebuilt_frame = buildNAV_STATUS(
            _week.week_millis,      // iTOW
            0x03,              // gpsFix (3D定位)
            0x03,           // flags (定位有效、差分解算、周数有效、时间有效)
            0xDF,           // fixStat (差分应用、载波解算)
            0x00,           // flags2
            500,           // ttff (5秒)
            38          // msss (100秒)
        );
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,rebuilt_frame, UBX_NAV_STATUS_LEN);
        // HAL_UART_Transmit(&huart1, rebuilt_frame, UBX_NAV_STATUS_LEN, HAL_MAX_DELAY);

        gps_data.speed = sqrt(ned_v.vN*ned_v.vN + ned_v.vE*ned_v.vE + ned_v.vU*ned_v.vU); 
        gps_data.gspeed = sqrt(ned_v.vN*ned_v.vN + ned_v.vE*ned_v.vE);

        gps_data.heading = mavlink_struct.odom_yaw_deg;

        // 重新合成帧
        rebuilt_frame = buildNAV_VELNED(
            _week.week_millis,      // iTOW
            ned_v.vN*100,            // velN (1 m/s = 100 cm/s)
            ned_v.vE*100,            // velE (2 m/s = 200 cm/s)
            -ned_v.vU*100,            // velD (-0.5 m/s = -50 cm/s)
            gps_data.speed*100,            // speed (2.5 m/s = 250 cm/s)
            gps_data.gspeed*100,            // gSpeed (2.24 m/s = 224 cm/s)
            gps_data.heading/1e-5,         // heading (45度 * 1e5)
            0.3*100,             // sAcc (0.5 m/s = 50 cm/s)
            1.4/1e-5           // cAcc (0.1度 * 1e5)
        );
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,rebuilt_frame, UBX_NAV_VELNED_LEN);
        // HAL_UART_Transmit(&huart1, rebuilt_frame, UBX_NAV_VELNED_LEN, HAL_MAX_DELAY);
    }

    if ((get_tick_diff(tick, odom_gps_fsm_struct.nav_1000ms_tick) >= 1000) && gps_on)
    {
        odom_gps_fsm_struct.nav_1000ms_tick = tick;
        
        // 合成DOP帧
        uint8_t* rebuilt_frame = buildNAV_DOP(
            _week.week_millis , 0.93 * 100, 0.891 * 100, 0.63 * 100, 
            0.68 * 100, 0.52 * 100, 0.35 * 100, 0.32 * 100);
        // 写入缓冲区
        RingBuff_WriteBytes(&u1TxRingBufferStruct,rebuilt_frame, UBX_NAV_DOP_LEN);
        // HAL_UART_Transmit(&huart1, rebuilt_frame, UBX_NAV_DOP_LEN, HAL_MAX_DELAY);
    }
}

Week_Date weeksBetweenDates(void)
{
    Week_Date _week;
    
    // 定义GPS时间起点：1980年1月6日 00:00:00 UTC
    // 计算从1970-01-01到1980-01-06的天数
    // 1980-01-06是GPS起始时间
    
    // 1970-01-01 到 1980-01-06 的秒数
    // 闰年：1972,1976,1980(1月6日还没到2月29，所以不算)
    // 天数计算：365*10 + 2 (1972,1976两个闰年) + 5天(1月1日到1月6日)
    // = 3650 + 2 + 5 = 3657天
    // 获取当前系统运行时间（毫秒）
    uint32_t current_tick = HAL_GetTick();  // 从系统启动开始的毫秒数
    
    // 注意：HAL_GetTick()只能获取系统运行时间，不是真实UTC时间
    // 如果需要真实GPS时间，需要设置一个基准时间
    
    // 假设系统启动时的GPS周数和周内毫秒数已知
    // 示例：假设系统启动时 GPS Week = 2000, Week Millis = 0
    static uint32_t boot_gps_week = 2000;        // 系统启动时的GPS周数
    static uint32_t boot_gps_millis = 0;         // 系统启动时的周内毫秒数
    
    // 计算当前GPS时间
    uint32_t total_millis = boot_gps_millis + current_tick;
    
    _week.week = boot_gps_week + (total_millis / (7 * 24 * 3600 * 1000));
    _week.week_millis = total_millis % (7 * 24 * 3600 * 1000);
    
    return _week;
}

/**
 * @brief 合成UBX-NAV-DOP消息 (0x01 0x04) - 从单独参数
 * @param iTOW GPS时间戳 (ms)
 * @param gDOP 几何精度因子 (实际值*100，例如1.25对应125)
 * @param pDOP 位置精度因子 (实际值*100)
 * @param tDOP 时间精度因子 (实际值*100)
 * @param vDOP 垂直精度因子 (实际值*100)
 * @param hDOP 水平精度因子 (实际值*100)
 * @param nDOP 北向精度因子 (实际值*100)
 * @param eDOP 东向精度因子 (实际值*100)
 * @return 完整的UBX帧数据（包含帧头B5 62和CRC）
 */
uint8_t* buildNAV_DOP(uint32_t iTOW, uint16_t gDOP, uint16_t pDOP, 
    uint16_t tDOP, uint16_t vDOP, uint16_t hDOP, 
    uint16_t nDOP, uint16_t eDOP)
{
    static uint8_t frame[UBX_NAV_DOP_LEN];
    int index = 0;

    // UBX帧头
    frame[index++] = 0xB5;  // frame[0]
    frame[index++] = 0x62;  // frame[1]

    // CLASS和ID
    frame[index++] = 0x01;  // frame[2] NAV CLASS
    frame[index++] = 0x04;  // frame[3] DOP ID

    // 载荷长度 (18字节，小端序)
    uint16_t payload_length = 18;
    frame[index++] = payload_length & 0xFF;        // frame[4] // LEN_L
    frame[index++] = (payload_length >> 8) & 0xFF; // frame[5] // LEN_H

    // 载荷数据（小端序）
    // iTOW - GPS时间戳 (4字节)
    frame[index++] = iTOW & 0xFF;   // frame[6]
    frame[index++] = (iTOW >> 8) & 0xFF;    // frame[7]
    frame[index++] = (iTOW >> 16) & 0xFF;   // frame[8]
    frame[index++] = (iTOW >> 24) & 0xFF;   // frame[9]

    // gDOP - 几何精度因子 (2字节)
    frame[index++] = gDOP & 0xFF;   // frame[10]
    frame[index++] = (gDOP >> 8) & 0xFF;    // frame[11]

    // pDOP - 位置精度因子 (2字节)
    frame[index++] = pDOP & 0xFF;   // frame[12]
    frame[index++] = (pDOP >> 8) & 0xFF;    // frame[13]

    // tDOP - 时间精度因子 (2字节)
    frame[index++] = tDOP & 0xFF;   // frame[14]
    frame[index++] = (tDOP >> 8) & 0xFF;    // frame[15]

    // vDOP - 垂直精度因子 (2字节)
    frame[index++] = vDOP & 0xFF;   // frame[16]
    frame[index++] = (vDOP >> 8) & 0xFF;    // frame[17]

    // hDOP - 水平精度因子 (2字节)
    frame[index++] = hDOP & 0xFF;   // frame[18]
    frame[index++] = (hDOP >> 8) & 0xFF;    // frame[19]

    // nDOP - 北向精度因子 (2字节)
    frame[index++] = nDOP & 0xFF;   // frame[20]
    frame[index++] = (nDOP >> 8) & 0xFF;    // frame[21]

    // eDOP - 东向精度因子 (2字节)
    frame[index++] = eDOP & 0xFF;   // frame[22]
    frame[index++] = (eDOP >> 8) & 0xFF;    // frame[23]

    // 计算CRC（从CLASS开始到载荷结束）
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < index; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }

    // 添加CRC
    frame[index++] = ck_a;  // frame[24]
    frame[index++] = ck_b;  // frame[25]

    return frame;
}

/**
 * @brief 计算大地水准面分离（简化EGM96模型）
 * @param lon_rad 经度（弧度）
 * @param lat_rad 纬度（弧度）
 * @return 大地水准面分离（米），正值表示大地水准面在椭球面上方
 */
double calculateGeoidSeparation(double lon_rad, double lat_rad)
{
    // 转换为度
    double lon_deg = lon_rad * 180.0 / M_PI;
    double lat_deg = lat_rad * 180.0 / M_PI;
    
    // 计算原点的大地水准面分离（使用简化模型）
    double origin_lon_rad = 106.0f * 1e-7 * M_PI / 180.0;
    double origin_lat_rad = 26.0f * 1e-7 * M_PI / 180.0;
    
    // 简化的EGM96近似（仅用于演示，实际应使用完整模型）
    // 对于中国地区，大地水准面分离通常在-30m到+50m之间
    double origin_lat_deg = origin_lat_rad * 180.0 / M_PI;
    double origin_lon_deg = origin_lon_rad * 180.0 / M_PI;
    
    // 简化的多项式近似（基于中国地区的典型值）
    double geoid_sep_origin = -30.0 + 0.5 * origin_lat_deg - 0.01 * origin_lon_deg;
    
    // 在小范围内（<10km），假设大地水准面分离变化很小，使用原点值
    // 计算当前位置与原点的距离（粗略估计）
    double delta_lat_rad = lat_rad - origin_lat_rad;
    double delta_lon_rad = lon_rad - origin_lon_rad;
    double distance_km = sqrt(pow(delta_lat_rad * 111.0, 2) + 
                                pow(delta_lon_rad * 111.0 * arm_cos_f32(origin_lat_rad), 2));
    
    // 如果距离小于10km，使用原点的大地水准面分离值
    if (distance_km < 10.0) {
        return geoid_sep_origin;
    }
    
    // 对于大范围，使用简化的全局模型
    double geoid_sep = -30.0 + 0.5 * lat_deg - 0.01 * lon_deg;
    
    // 限制在合理范围内（全球大地水准面分离通常在-100m到+100m之间）
    if (geoid_sep > 100.0) geoid_sep = 100.0;
    if (geoid_sep < -100.0) geoid_sep = -100.0;
    
    return geoid_sep;
}

/**
 * @brief 从SLAM的xyz坐标转换为GPS经纬度、海拔、椭球高
 * @param slam_x SLAM坐标系X坐标 (米)
 * @param slam_y SLAM坐标系Y坐标 (米)
 * @param slam_z SLAM坐标系Z坐标 (米)
 * @param longitude 输出的经度 (度 * 1e-7)
 * @param latitude 输出的纬度 (度 * 1e-7)
 * @param height 输出的椭球高 (mm)
 * @param hMSL 输出的海拔高 (mm)
 * @return true 转换成功, false 原点未设置
 */
void convertSLAMToGPS(double slam_x, double slam_y, double slam_z,
                                   int32_t* longitude, int32_t* latitude,
                                   int32_t* height, int32_t* hMSL)
{
    
    // 将原点经纬度转换为弧度
    double origin_lon_rad = 106.0f * M_PI / 180.0;
    double origin_lat_rad = 26.0f * M_PI / 180.0;
    
    // WGS84椭球参数
    const double a = 6378137.0;           // 长半轴 (米)
    const double e2 = 6.69437999014e-3;   // 第一偏心率平方
    
    // 计算原点处的子午圈曲率半径和卯酉圈曲率半径
    double sin_lat = arm_sin_f32(origin_lat_rad);
    double cos_lat = arm_cos_f32(origin_lat_rad);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);  // 卯酉圈曲率半径
    double M = a * (1.0 - e2) / pow(1.0 - e2 * sin_lat * sin_lat, 1.5);  // 子午圈曲率半径
    
    // 假设SLAM坐标系为ENU（东-北-天）：X=东，Y=北，Z=天
    double delta_east = slam_x;   // 东向偏移（米）
    double delta_north = slam_y;  // 北向偏移（米）
    double delta_up = slam_z;     // 天向偏移（米）
    
    // 计算经纬度偏移（小范围近似）
    double delta_lat_rad = delta_north / M;  // 纬度偏移（弧度）
    double delta_lon_rad = delta_east / (N * cos_lat);  // 经度偏移（弧度）
    
    // 计算新的经纬度（弧度）
    double new_lat_rad = origin_lat_rad + delta_lat_rad;
    double new_lon_rad = origin_lon_rad + delta_lon_rad;
    
    // 转换为度并乘以1e7
    *latitude = (int32_t)(new_lat_rad * 180.0 / M_PI * 1e7);
    *longitude = (int32_t)(new_lon_rad * 180.0 / M_PI * 1e7);
    
    // 计算高度
    // 海拔 = 原点海拔 + Z偏移
    *hMSL = 1000.0f * 1000.0 + (int32_t)(delta_up * 1000.0);  // 转换为mm
    
    // 计算大地水准面分离（使用新的经纬度）
    // 注意：大地水准面分离 N = 椭球高 - 海拔（N = h - H）
    // 如果N为正，说明椭球高 > 海拔，大地水准面在椭球面上方
    // 如果N为负，说明椭球高 < 海拔，大地水准面在椭球面下方
    double geoid_sep_m = calculateGeoidSeparation(new_lon_rad, new_lat_rad);
    
    // 椭球高 = 海拔 + 大地水准面分离
    // 因为：N = h - H，所以 h = H + N
    *height = *hMSL + (int32_t)(geoid_sep_m * 1000.0);  // 转换为mm
}

/**
 * @brief 合成UBX-NAV-POSLLH消息 (0x01 0x02) - 从单独参数
 * @param iTOW GPS时间戳 (ms)
 * @param longitude 经度 (度 * 1e-7)
 * @param latitude 纬度 (度 * 1e-7)
 * @param height 椭球高 (mm)
 * @param hMSL 平均海平面高 (mm)
 * @param hAcc 水平精度估计 (mm)
 * @param vAcc 垂直精度估计 (mm)
 * @return 完整的UBX帧数据（包含帧头B5 62和CRC）
 */
uint8_t* buildNAV_POSLLH(uint32_t iTOW, int32_t longitude, int32_t latitude,
                            int32_t height, int32_t hMSL, uint32_t hAcc, uint32_t vAcc)
{
    static uint8_t frame[UBX_NAV_POSLLH_LEN];
    size_t index = 0;
    
    // UBX帧头
    frame[index++] = 0xB5;  // frame[0]
    frame[index++] = 0x62;  // frame[1]
    
    // CLASS和ID
    frame[index++] = 0x01;  // frame[2] - NAV CLASS
    frame[index++] = 0x02;  // frame[3] - POSLLH ID
    
    // 载荷长度 (28字节，小端序)
    uint16_t payload_length = 28;
    frame[index++] = payload_length & 0xFF;        // frame[4] - LEN_L
    frame[index++] = (payload_length >> 8) & 0xFF; // frame[5] - LEN_H
    
    // 载荷数据（小端序）
    // iTOW - GPS时间戳 (4字节)
    frame[index++] = iTOW & 0xFF;   // frame[6]
    frame[index++] = (iTOW >> 8) & 0xFF;    // frame[7]
    frame[index++] = (iTOW >> 16) & 0xFF; // frame[8]
    frame[index++] = (iTOW >> 24) & 0xFF; // frame[9]

    // Longitude - 经度 (4字节, int32_t)
    frame[index++] = longitude & 0xFF;  // frame[10]
    frame[index++] = (longitude >> 8) & 0xFF; // frame[11]
    frame[index++] = (longitude >> 16) & 0xFF; // frame[12]
    frame[index++] = (longitude >> 24) & 0xFF; // frame[13]
    
    // Latitude - 纬度 (4字节, int32_t)
    frame[index++] = latitude & 0xFF;   // frame[14]
    frame[index++] = (latitude >> 8) & 0xFF; // frame[15]
    frame[index++] = (latitude >> 16) & 0xFF; // frame[16]
    frame[index++] = (latitude >> 24) & 0xFF; // frame[17]
    
    // Height - 椭球高 (4字节, int32_t)
    frame[index++] = height & 0xFF; // frame[18]
    frame[index++] = (height >> 8) & 0xFF; // frame[19]
    frame[index++] = (height >> 16) & 0xFF; // frame[20]
    frame[index++] = (height >> 24) & 0xFF; // frame[21]
    
    // hMSL - 平均海平面高 (4字节, int32_t)
    frame[index++] = hMSL & 0xFF; // frame[22]
    frame[index++] = (hMSL >> 8) & 0xFF; // frame[23]
    frame[index++] = (hMSL >> 16) & 0xFF; // frame[24]
    frame[index++] = (hMSL >> 24) & 0xFF; // frame[25]
    
    // hAcc - 水平精度估计 (4字节, uint32_t)
    frame[index++] = hAcc & 0xFF; // frame[26]
    frame[index++] = (hAcc >> 8) & 0xFF; // frame[27]
    frame[index++] = (hAcc >> 16) & 0xFF; // frame[28]
    frame[index++] = (hAcc >> 24) & 0xFF; // frame[29]
    
    // vAcc - 垂直精度估计 (4字节, uint32_t)
    frame[index++] = vAcc & 0xFF; // frame[30]
    frame[index++] = (vAcc >> 8) & 0xFF; // frame[31]
    frame[index++] = (vAcc >> 16) & 0xFF; // frame[32]
    frame[index++] = (vAcc >> 24) & 0xFF; // frame[33]
    
    // 计算CRC（从CLASS开始到载荷结束）
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < index; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    
    // 添加CRC
    frame[index++] = ck_a;  // frame[34]
    frame[index++] = ck_b;  // frame[35]
    
    return frame;
}

ECEF llhToEcef(double lon, double lat, double alt)
{
    const double a = 6378137.0;                // 长半轴
    const double f = 1.0 / 298.257223563;      // 扁率
    const double e2 = f * (2 - f);             // 第一偏心率平方

    double lat_rad = lat * M_PI / 180.0;   // 度转弧度
    double lon_rad = lon * M_PI / 180.0;

    double sin_lat = arm_sin_f32(lat_rad);
    double cos_lat = arm_cos_f32(lat_rad);
    double sin_lon = arm_sin_f32(lon_rad);
    double cos_lon = arm_cos_f32(lon_rad);

    double N = a / sqrt(1 - e2 * sin_lat * sin_lat);

    ECEF ecef;
    ecef.x = (N + alt) * cos_lat * cos_lon;
    ecef.y = (N + alt) * cos_lat * sin_lon;
    ecef.z = (N * (1 - e2) + alt) * sin_lat;

    return ecef;
}

ECEFVel nedToEcefVel(double lon, double lat, const NEDVel* vel)
{
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    ECEFVel ecef;
    ecef.vx = -arm_sin_f32(lon_rad) * vel->vE - arm_sin_f32(lat_rad) * arm_cos_f32(lon_rad) * vel->vN + arm_cos_f32(lat_rad) * arm_cos_f32(lon_rad) * vel->vU;
    ecef.vy =  arm_cos_f32(lon_rad) * vel->vE - arm_sin_f32(lat_rad) * arm_sin_f32(lon_rad) * vel->vN + arm_cos_f32(lat_rad) * arm_sin_f32(lon_rad) * vel->vU;
    ecef.vz =                   arm_cos_f32(lat_rad) * vel->vN + arm_sin_f32(lat_rad) * vel->vU;
    return ecef;
}

/**
 * @brief 合成UBX-NAV-SOL消息 (0x01 0x06) - 从单独参数
 * @param iTOW GPS时间戳 (ms)
 * @param fTOW 小数部分时间戳 (ns)
 * @param week GPS周数
 * @param gpsFix GPS定位状态
 * @param flags 状态标志
 * @param ecefX ECEF X坐标 (cm)
 * @param ecefY ECEF Y坐标 (cm)
 * @param ecefZ ECEF Z坐标 (cm)
 * @param pAcc 位置精度估计 (cm)
 * @param ecefVX ECEF X速度 (cm/s)
 * @param ecefVY ECEF Y速度 (cm/s)
 * @param ecefVZ ECEF Z速度 (cm/s)
 * @param sAcc 速度精度估计 (cm/s)
 * @param pDOP 位置精度因子
 * @param numSV 使用的卫星数量
 * @return 完整的UBX帧数据（包含帧头B5 62和CRC）
 */
uint8_t* buildNAV_SOL(uint32_t iTOW, int32_t fTOW, int16_t week,
                                               uint8_t gpsFix, uint8_t flags,
                                               int32_t ecefX, int32_t ecefY, int32_t ecefZ,
                                               uint32_t pAcc,
                                               int32_t ecefVX, int32_t ecefVY, int32_t ecefVZ,
                                               uint32_t sAcc, uint16_t pDOP, uint8_t numSV)
{
    static uint8_t frame[UBX_NAV_SOL_LEN];
    size_t index = 0;

    // UBX帧头
    frame[index++] = 0xB5;  // frame[0]
    frame[index++] = 0x62;  // frame[1]
    
    // CLASS和ID
    frame[index++] = 0x01;  // frame[2] NAV CLASS
    frame[index++] = 0x06;  // frame[3] SOL ID
    
    // 载荷长度 (52字节，小端序)
    uint16_t payload_length = 52;
    frame[index++] = payload_length & 0xFF;        // frame[4] LEN_L
    frame[index++] = (payload_length >> 8) & 0xFF; // frame[5] LEN_H
    
    // 载荷数据（小端序）
    // iTOW - GPS时间戳 (4字节)
    frame[index++] = iTOW & 0xFF;   // frame[6]
    frame[index++] = (iTOW >> 8) & 0xFF; // frame[7]
    frame[index++] = (iTOW >> 16) & 0xFF; // frame[8]
    frame[index++] = (iTOW >> 24) & 0xFF; // frame[9]
    
    // fTOW - 小数部分时间戳 (4字节, int32_t)
    frame[index++] = fTOW & 0xFF; // frame[10]
    frame[index++] = (fTOW >> 8) & 0xFF; // frame[11]
    frame[index++] = (fTOW >> 16) & 0xFF; // frame[12]
    frame[index++] = (fTOW >> 24) & 0xFF; // frame[13]
    
    // week - GPS周数 (2字节, int16_t)
    frame[index++] = week & 0xFF; // frame[14]
    frame[index++] = (week >> 8) & 0xFF; // frame[15]
    
    // gpsFix - GPS定位状态 (1字节)
    frame[index++] = gpsFix;    // frame[16]
    
    // flags - 状态标志 (1字节)
    frame[index++] = flags;     // frame[17]
    
    // ecefX - ECEF X坐标 (4字节, int32_t)
    frame[index++] = ecefX & 0xFF;  // frame[18]
    frame[index++] = (ecefX >> 8) & 0xFF; // frame[19]
    frame[index++] = (ecefX >> 16) & 0xFF; // frame[20]
    frame[index++] = (ecefX >> 24) & 0xFF; // frame[21]
    
    // ecefY - ECEF Y坐标 (4字节, int32_t)
    frame[index++] = ecefY & 0xFF;  // frame[22]
    frame[index++] = (ecefY >> 8) & 0xFF; // frame[23]
    frame[index++] = (ecefY >> 16) & 0xFF; // frame[24]
    frame[index++] = (ecefY >> 24) & 0xFF; // frame[25]
    
    // ecefZ - ECEF Z坐标 (4字节, int32_t)
    frame[index++] = ecefZ & 0xFF; // frame[26]
    frame[index++] = (ecefZ >> 8) & 0xFF; // frame[27]
    frame[index++] = (ecefZ >> 16) & 0xFF; // frame[28]
    frame[index++] = (ecefZ >> 24) & 0xFF; // frame[29]
    
    // pAcc - 位置精度估计 (4字节, uint32_t)
    frame[index++] = pAcc & 0xFF;   // frame[30]
    frame[index++] = (pAcc >> 8) & 0xFF; // frame[31]
    frame[index++] = (pAcc >> 16) & 0xFF; // frame[32]
    frame[index++] = (pAcc >> 24) & 0xFF; // frame[33]
    
    // ecefVX - ECEF X速度 (4字节, int32_t)
    frame[index++] = ecefVX & 0xFF; // frame[34]
    frame[index++] = (ecefVX >> 8) & 0xFF; // frame[35]
    frame[index++] = (ecefVX >> 16) & 0xFF; // frame[36]
    frame[index++] = (ecefVX >> 24) & 0xFF; // frame[37]
    
    // ecefVY - ECEF Y速度 (4字节, int32_t)
    frame[index++] = ecefVY & 0xFF; // frame[38]
    frame[index++] = (ecefVY >> 8) & 0xFF; // frame[39]
    frame[index++] = (ecefVY >> 16) & 0xFF; // frame[40]
    frame[index++] = (ecefVY >> 24) & 0xFF; // frame[41]
    
    // ecefVZ - ECEF Z速度 (4字节, int32_t)
    frame[index++] = ecefVZ & 0xFF; // frame[42]
    frame[index++] = (ecefVZ >> 8) & 0xFF; // frame[43]
    frame[index++] = (ecefVZ >> 16) & 0xFF; // frame[44]
    frame[index++] = (ecefVZ >> 24) & 0xFF; // frame[45]
    
    // sAcc - 速度精度估计 (4字节, uint32_t)
    frame[index++] = sAcc & 0xFF; // frame[46]
    frame[index++] = (sAcc >> 8) & 0xFF; // frame[47]
    frame[index++] = (sAcc >> 16) & 0xFF; // frame[48]
    frame[index++] = (sAcc >> 24) & 0xFF; // frame[49]
    
    // pDOP - 位置精度因子 (2字节, uint16_t)
    frame[index++] = pDOP & 0xFF; // frame[50]
    frame[index++] = (pDOP >> 8) & 0xFF; // frame[51]
    
    // reserved1 - 保留字节 (1字节)
    frame[index++] = 0x00;  // frame[52]
    
    // numSV - 使用的卫星数量 (1字节)
    frame[index++] = numSV; // frame[53]
    
    // reserved2 - 保留字节 (4字节)
    frame[index++] = 0x00; // frame[54]
    frame[index++] = 0x00; // frame[55]
    frame[index++] = 0x00; // frame[56]
    frame[index++] = 0x00; // frame[57]
    
    // 计算CRC（从CLASS开始到载荷结束）
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < index; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    
    // 添加CRC
    frame[index++] = ck_a;  // frame[58]
    frame[index++] = ck_b;  // frame[59]
    
    return frame;
}

/**
 * @brief 合成UBX-NAV-STATUS消息 (0x01 0x03) - 从单独参数
 * @param iTOW GPS时间戳 (ms)
 * @param gpsFix GPS定位状态
 * @param flags 状态标志
 * @param fixStat 修复状态
 * @param flags2 额外标志
 * @param ttff 首次定位时间 (ms)
 * @param msss 启动后的时间 (ms)
 * @return 完整的UBX帧数据（包含帧头B5 62和CRC）
 */
uint8_t* buildNAV_STATUS(uint32_t iTOW, uint8_t gpsFix, uint8_t flags,
                                                  uint8_t fixStat, uint8_t flags2,
                                                  uint32_t ttff, uint32_t msss)
{
    static uint8_t frame[UBX_NAV_STATUS_LEN];
    size_t index = 0;

    // UBX帧头
    frame[index++] = 0xB5;  // frame[0]
    frame[index++] = 0x62;  // frame[1]
    
    // CLASS和ID
    frame[index++] = 0x01;  // frame[2] NAV CLASS
    frame[index++] = 0x03;  // frame[3] STATUS ID
    
    // 载荷长度 (16字节，小端序)
    uint16_t payload_length = 16;
    frame[index++] = payload_length & 0xFF;        // frame[4] LEN_L
    frame[index++] = (payload_length >> 8) & 0xFF; // frame[5] LEN_H
    
    // 载荷数据（小端序）
    // iTOW - GPS时间戳 (4字节)
    frame[index++] = iTOW & 0xFF;  // frame[6]
    frame[index++] = (iTOW >> 8) & 0xFF; // frame[7]
    frame[index++] = (iTOW >> 16) & 0xFF; // frame[8]
    frame[index++] = (iTOW >> 24) & 0xFF; // frame[9]
    
    // gpsFix - GPS定位状态 (1字节)
    frame[index++] = gpsFix;    // frame[10]
    
    // flags - 状态标志 (1字节)
    frame[index++] = flags;    // frame[11]
    
    // fixStat - 修复状态 (1字节)
    frame[index++] = fixStat;   // frame[12]
    
    // flags2 - 额外标志 (1字节)
    frame[index++] = flags2;    // frame[13]
    
    // ttff - 首次定位时间 (4字节, uint32_t)
    frame[index++] = ttff & 0xFF;   // frame[14]
    frame[index++] = (ttff >> 8) & 0xFF; // frame[15]
    frame[index++] = (ttff >> 16) & 0xFF; // frame[16]
    frame[index++] = (ttff >> 24) & 0xFF; // frame[17]
    
    // msss - 启动后的时间 (4字节, uint32_t)
    frame[index++] = msss & 0xFF; // frame[18]
    frame[index++] = (msss >> 8) & 0xFF; // frame[19]
    frame[index++] = (msss >> 16) & 0xFF; // frame[20]
    frame[index++] = (msss >> 24) & 0xFF; // frame[21]
    
    // 计算CRC（从CLASS开始到载荷结束）
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < index; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    
    // 添加CRC
    frame[index++] = ck_a;  // frame[22]
    frame[index++] = ck_b;  // frame[23]
    
    return frame;
}

/**
 * @brief 合成UBX-NAV-VELNED消息 (0x01 0x12) - 从单独参数
 * @param iTOW GPS时间戳 (ms)
 * @param velN 北向速度 (cm/s)
 * @param velE 东向速度 (cm/s)
 * @param velD 地向速度 (cm/s)
 * @param speed 3D速度 (cm/s)
 * @param gSpeed 地面速度 (cm/s)
 * @param heading 航向 (度 * 1e-5)
 * @param sAcc 速度精度估计 (cm/s)
 * @param cAcc 航向精度估计 (度 * 1e-5)
 * @return 完整的UBX帧数据（包含帧头B5 62和CRC）
 */
uint8_t* buildNAV_VELNED(uint32_t iTOW, int32_t velN, int32_t velE, int32_t velD,
                                                  uint32_t speed, uint32_t gSpeed, int32_t heading,
                                                  uint32_t sAcc, uint32_t cAcc)
{
    static uint8_t frame[UBX_NAV_VELNED_LEN];
    size_t index = 0;
    
    // UBX帧头
    frame[index++] = 0xB5;  // frame[0]
    frame[index++] = 0x62;  // frame[1]
    
    // CLASS和ID
    frame[index++] = 0x01;  // frame[2] NAV CLASS
    frame[index++] = 0x12;  // frame[3] VELNED ID
    
    // 载荷长度 (36字节，小端序)
    uint16_t payload_length = 36;
    frame[index++] = payload_length & 0xFF;        // frame[4] LEN_L
    frame[index++] = (payload_length >> 8) & 0xFF; // frame[5] LEN_H
    
    // 载荷数据（小端序）
    // iTOW - GPS时间戳 (4字节)
    frame[index++] = iTOW & 0xFF;   // frame[6]
    frame[index++] = (iTOW >> 8) & 0xFF;    // frame[7]
    frame[index++] = (iTOW >> 16) & 0xFF;   // frame[8]
    frame[index++] = (iTOW >> 24) & 0xFF;   // frame[9]
    
    // velN - 北向速度 (4字节, int32_t)
    frame[index++] = velN & 0xFF;   // frame[10]
    frame[index++] = (velN >> 8) & 0xFF;    // frame[11]
    frame[index++] = (velN >> 16) & 0xFF;   // frame[12]
    frame[index++] = (velN >> 24) & 0xFF;   // frame[13]
    
    // velE - 东向速度 (4字节, int32_t)
    frame[index++] = velE & 0xFF;   // frame[14]
    frame[index++] = (velE >> 8) & 0xFF;    // frame[15]
    frame[index++] = (velE >> 16) & 0xFF;   // frame[16]
    frame[index++] = (velE >> 24) & 0xFF;   // frame[17]
    
    // velD - 地向速度 (4字节, int32_t)
    frame[index++] = velD & 0xFF;   // frame[18]
    frame[index++] = (velD >> 8) & 0xFF;    // frame[19]
    frame[index++] = (velD >> 16) & 0xFF;   // frame[20]
    frame[index++] = (velD >> 24) & 0xFF;   // frame[21]
    
    // speed - 3D速度 (4字节, uint32_t)
    frame[index++] = speed & 0xFF;  // frame[22]
    frame[index++] = (speed >> 8) & 0xFF; // frame[23]
    frame[index++] = (speed >> 16) & 0xFF; // frame[24]
    frame[index++] = (speed >> 24) & 0xFF; // frame[25]
    
    // gSpeed - 地面速度 (4字节, uint32_t)
    frame[index++] = gSpeed & 0xFF; // frame[26]
    frame[index++] = (gSpeed >> 8) & 0xFF; // frame[27]
    frame[index++] = (gSpeed >> 16) & 0xFF; // frame[28]
    frame[index++] = (gSpeed >> 24) & 0xFF; // frame[29]
    
    // heading - 航向 (4字节, int32_t)
    frame[index++] = heading & 0xFF;    // frame[30]
    frame[index++] = (heading >> 8) & 0xFF; // frame[31]
    frame[index++] = (heading >> 16) & 0xFF; // frame[32]
    frame[index++] = (heading >> 24) & 0xFF; // frame[33]
    
    // sAcc - 速度精度估计 (4字节, uint32_t)
    frame[index++] = sAcc & 0xFF; // frame[34]
    frame[index++] = (sAcc >> 8) & 0xFF; // frame[35]
    frame[index++] = (sAcc >> 16) & 0xFF; // frame[36]
    frame[index++] = (sAcc >> 24) & 0xFF; // frame[37]
    
    // cAcc - 航向精度估计 (4字节, uint32_t)
    frame[index++] = cAcc & 0xFF; // frame[38]
    frame[index++] = (cAcc >> 8) & 0xFF; // frame[39]
    frame[index++] = (cAcc >> 16) & 0xFF; // frame[40]
    frame[index++] = (cAcc >> 24) & 0xFF; // frame[41]
    
    // 计算CRC（从CLASS开始到载荷结束）
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < index; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    
    // 添加CRC
    frame[index++] = ck_a;  // frame[42]
    frame[index++] = ck_b;  // frame[43]
    
    return frame;
}

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
static void calculate_fletcher_checksum(uint8_t* data, uint8_t* ck_a, uint8_t* ck_b)
{
    *ck_a = 0;
    *ck_b = 0;

    for (uint16_t i = 0; i < 6; i++) {
        *ck_a = (*ck_a + data[4 + i]) & 0xFF;
        *ck_b = (*ck_b + *ck_a) & 0xFF;
    }
}

static CompassCRC print_checksum_comparison(uint8_t* received)
{
    CompassCRC crc;

    // 计算校验位
    uint8_t calculated_ck_a, calculated_ck_b;
    calculate_fletcher_checksum(received, &calculated_ck_a, &calculated_ck_b);

    crc.crc_a = calculated_ck_a;
    crc.crc_b = calculated_ck_b;

    return crc;
}

static uint8_t* buildCompassFrame(int16_t x, int16_t y, int16_t z)
{
    static uint8_t frame[12];
    int index = 0;

    frame[index++] = 0x55;
    frame[index++] = 0xAA;
    frame[index++] = 0x01;
    frame[index++] = 0x6; // 数据长度

    // 写入小端x
    frame[index++] = (uint8_t)(x & 0xFF);
    frame[index++] = (uint8_t)((x >> 8) & 0xFF);
    // 写入小端y
    frame[index++] = (uint8_t)(y & 0xFF);
    frame[index++] = (uint8_t)((y >> 8) & 0xFF);
    // 写入小端z
    frame[index++] = (uint8_t)(z & 0xFF);
    frame[index++] = (uint8_t)((z >> 8) & 0xFF);

    // 计算CRC
    CompassCRC crc = print_checksum_comparison(frame);
    
    // 添加CRC
    frame[index++] = crc.crc_a;
    frame[index++] = crc.crc_b;

    return frame;
}

uint8_t* calculateMagneticFieldFromHeading(double yaw)
{
    // 从已知航向角 _Yaw 反推磁罗盘 xyz 分量
    // _Yaw 是度（北为0°顺时针，0-360度）
    double heading_from_north = yaw;
    
    // 确保角度在0-360度范围内
    heading_from_north = fmod(heading_from_north + 360.0, 360.0);
    
    // 反向推导：从 heading_from_north 反推 atan2(mag_y, mag_x)
    // heading_from_north = (180 - atan2(mag_y, mag_x) * 180/π) % 360
    // 所以：atan2(mag_y, mag_x) = (180 - heading_from_north) * π/180
    double atan2_angle_rad = (180.0 - heading_from_north) * M_PI / 180.0;
    
    // 磁场强度（根据实际测量值，约为645）
    double mag_strength = 645.0;
    
    // 计算水平磁场分量
    double mag_x = mag_strength * arm_cos_f32(atan2_angle_rad);
    double mag_y = mag_strength * arm_sin_f32(atan2_angle_rad);
    double mag_z = 0.0;  // 垂直分量，假设传感器水平
    
    // 转换为 int16_t
    int16_t mag_x_int16 = (int16_t)(mag_x);
    int16_t mag_y_int16 = (int16_t)(mag_y);
    int16_t mag_z_int16 = (int16_t)(mag_z);

    // 构建完整帧（包含校验）
    uint8_t* compass = buildCompassFrame(mag_x_int16, mag_y_int16, mag_z_int16);
    return compass;
}

