/**!
 * @file getObstacleDistance.ino
 * @brief 这是一个获取tof障碍物距离的demo,运行demo将打印障碍物距离
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
uint16_t buf[DATA];
char report[128];
void setup(void){
  Serial.begin(115200);
  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
   Serial.println("begin success");
 //配置避障距离15厘米
  while(tof.configAvoidance(15) != 0){
    Serial.println("init avoid error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
  
}

void loop(void){
  uint16_t L = 0, M = 0, R = 0;
  tof.getObstacleDistance(&L,&M,&R);
  Serial.print("L: ");
  Serial.print(L);
  Serial.print(" ,M: ");
  Serial.print(M);
  Serial.print(" ,R: ");
  Serial.print(R);
  
  Serial.println("------------------------------");
  delay(10);
}
