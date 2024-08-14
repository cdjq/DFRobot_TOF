/**!
 * @file getFixedPoint.ino
 * @brief 这是一个获取tof指定点数据的demo,运行demo将获取指定点的数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [tangjie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2024-08-14
 * @url https://github.com/DFRobot/DFRobot_TOF
 */
#include "DFRobot_tof.h"
#define DATA8X8 ///获取8x8矩阵数据
#ifdef DATA8X8
#define DATA 8
#else
#define DATA 4
#endif
DFRobot_tof tof;
uint16_t buf[64];
void setup(void){
  Serial.begin(115200);
  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
   Serial.println("begin success");
  while(tof.getAllDataConfig(DATA,0) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
  
  
}

void loop(void){
    uint16_t data = tof.getFixedPointData(1,0);
    Serial.print(data);
    Serial.println(" mm");
    delay(10);
}
