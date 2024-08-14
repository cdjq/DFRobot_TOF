#ifndef _DFROBOT_TOF_H_
#define _DFROBOT_TOF_H_
#include "Arduino.h"
#include "Wire.h"

//#define ENABLE_DBG ///< 打开这个宏, 可以看到程序的详细运行过程
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class DFRobot_tof{
public:
    /**
     * @fn DFRobot_tof
     * @brief tof 传感器的构造函数
     * @param pWire 通信协议初始化
     */
    DFRobot_tof(uint8_t addr = 0x30, TwoWire *pWire=&Wire);

    uint8_t begin(void);

    /**
     * @fn getAllDataConfig
     * @brief 获取全部数据的配置
     * @param matrix 配置传感器采样矩阵
     * @param threshold 配置传感器报警阈值，范围50~3000，低于50按照原始数据输出
     * @return 返回配置状态
     * @retval 0 成功
     * @retval 1 失败
     */
    uint8_t getAllDataConfig(uint8_t matrix, uint16_t threshold = 0);

    /**
     * @fn configAvoidance
     * @brief 初始化避障
     * @param wall 配置避障距离,单位：厘米
     */
    uint8_t configAvoidance(uint8_t wall);

    /**
     * @fn getAllData
     * @brief 获取全部数据
     * @param buf 存储数据
     */
    uint8_t getAllData(void *buf);

    /**
     * @fn getFixedPointData
     * @brief 获取指定点的数据
     * @param x 坐标x
     * @param y 坐标y
     * @return 返回获取的数据
     */
    uint16_t getFixedPointData(uint8_t x, uint8_t y);

    /**
     * @fn getLineData
     * @brief 获取指定行数据
     * @param line 行号
     * @param buf 存储数据
     */
    uint8_t getLineData(uint8_t line, void *buf);

    /**
     * @fn getListData
     * @brief 获取指定列数据
     * @param list 列号
     * @param buf 存储数据
     */
    uint8_t getListData(uint8_t list, void *buf);

    /**
     * @fn getAvoid
     * @brief 获取避障返回的建议
     * @param dir 建议小车前进的方向
     * @param urgency 紧急避险状态
     * @return 返回获取状态
     */
    uint8_t getAvoid(uint8_t *dir, uint8_t *urgency);

    /**
     * @fn getObstacleDistance
     * @brief 获取障碍物不同位置的距离
     * @param L 左侧距离障碍物距离
     * @param M 中间距离障碍物距离
     * @param R 右侧距离障碍物距离
     */
    uint8_t getObstacleDistance(uint16_t *L, uint16_t *M, uint16_t *R); 

private:
    TwoWire *_pWire;
    uint8_t _addr;
    uint32_t _timeout; ///< Time of receive timeout
    void sendPacket(void *pkt, int length, bool stop);
    void *recvPacket(uint8_t cmd, uint8_t *errorCode);
    int recvData(void *data, int len);
};

#endif
