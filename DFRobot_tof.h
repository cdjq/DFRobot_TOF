/*!
 * @file DFRobot_tof.h
 * @brief This is the method documentation file for DFRobot_tof
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [TangJie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2023-12-7
 * @url https://github.com/DFRobot/DFRobot_TOF
 */
#ifndef _DFROBOT_TOF_H_
#define _DFROBOT_TOF_H_
#include "Arduino.h"
#include "Wire.h"

//#define ENABLE_DBG ///< Enable this macro to view the detailed execution process of the program.
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

 /**
  * @brief Select to get the obstacle position.
  */
typedef enum{
    eLeft, 
    eMiddle,
    eRight,
}eDir_t;

class DFRobot_TOF{
public:
    
    /**
     * @fn DFRobot_tof
     * @brief Constructor for the TOF sensor
     * @param pWire Communication protocol initialization
     */
    DFRobot_TOF(uint8_t addr = 0x30, TwoWire *pWire=&Wire);
    
    /**
     * @fn begin
     * @brief Initializes the sensor
     * @return Returns the initialization status
     */
    uint8_t begin(void);

    /**
     * @fn getAllDataConfig
     * @brief Configures the retrieval of all data
     * @param matrix Configuration matrix for sensor sampling
     * @param threshold Sensor alarm threshold, range from 50 to 3000; below 50, output raw data
     * @return Returns the configuration status
     * @retval 0 Success
     * @retval 1 Failure
     */
    uint8_t getAllDataConfig(uint8_t matrix, uint16_t threshold = 0);

    /**
     * @fn configAvoidance
     * @brief Initializes obstacle avoidance
     * @param wall Configures the obstacle avoidance distance, in centimeters
     */
    uint8_t configAvoidance(uint8_t wall);

    /**
     * @fn getAllData
     * @brief Retrieves all data
     * @param buf Buffer to store the data
     */
    uint8_t getAllData(void *buf);

    /**
     * @fn getFixedPointData
     * @brief Retrieves data for a specific point
     * @param x X coordinate
     * @param y Y coordinate
     * @return Returns the retrieved data
     */
    uint16_t getFixedPointData(uint8_t x, uint8_t y);

    /**
     * @fn requestObstacleSensorData
     * @brief Requests obstacle avoidance data
     * @return Returns the retrieval status
     */
    uint8_t requestObstacleSensorData(void);

    /**
     * @fn getDir
     * @brief Retrieves obstacle avoidance direction suggestions
     * @return Returns the avoidance suggestions
     */
    uint8_t getDir(void);

    /**
     * @fn getEmergencyFlag
     * @brief Retrieves the emergency obstacle avoidance flag
     * @return Returns the obstacle avoidance flag
     */
    uint8_t getEmergencyFlag(void);

    /**
     * @fn getObstacleDistance
     * @brief Requests the distance to obstacles
     * @return Returns the request status
     */
    uint8_t requestObstacleDistance(void); 

    /**
     * @fn getDistance
     * @brief Retrieves the distance
     * @return Returns the distance
     */
    uint16_t getDistance(eDir_t dir);

    
private:
    TwoWire *_pWire;
    uint8_t _addr;
    uint32_t _timeout; ///< Time of receive timeout
    void sendPacket(void *pkt, int length, bool stop);
    void *recvPacket(uint8_t cmd, uint8_t *errorCode);
    int recvData(void *data, int len);
};

#endif
