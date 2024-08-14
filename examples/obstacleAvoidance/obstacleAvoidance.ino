/**!
 * @file obstacleAvoidance.ino
 * @brief 这是一个基础避障的demo,运行demo将进行基础避障
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [tangjie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2024-08-14
 * @url https://github.com/DFRobot/DFRobot_TOF
 */
#include "DFRobot_tof.h"

DFRobot_tof tof;
uint16_t buf[2];

#define NUM_SECTORS 8

#define L_M  1
#define R_M 2
#define CW 0
#define CCW 1

uint8_t nowDirection = 0;

#define FORWARD  2
#define LEFT  4
#define RIGHT  1

void motor(uint8_t speed, uint8_t dir, uint8_t mt){
    Wire.beginTransmission(0x10);
    if(mt == L_M){
        Wire.write((uint8_t)0);
        Wire.write(dir);
        Wire.write(speed);
    }else if(mt == R_M){
        Wire.write((uint8_t)2);
        Wire.write(dir);
        Wire.write(speed);
    }
    Wire.endTransmission();
}

// 前进
void moveForward() {
  motor(60, CW, L_M);
  motor(60, CW, R_M);
  
}

// 后退
void moveBackward() {
 
  motor(40, CCW, L_M);
  motor(40, CCW, R_M);
 
}

// 左转
void turnLeft() {
  motor(40, CCW, L_M);
  motor(40, CW, R_M);
}

// 右转
void turnRight() {
  motor(40, CW, L_M);
  motor(40, CCW, R_M);
 
}

//左右轮速差
// 左转
void turnLeft1(uint8_t speed) {
  motor(20, CW, L_M);
  motor(120, CW, R_M);
}

// 右转
void turnRight1(uint8_t speed) {
  motor(120, CW, L_M);
  motor(20, CW, R_M);
 
}


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
  uint8_t dir = 0,urgency = 0;
  //获取返回避障建议
  tof.getAvoid(&dir,&urgency);
 switch(dir){
    case FORWARD:
      moveForward();
      break;
    case RIGHT:
      if(urgency > 0){
        turnRight1(buf[1]);
      }else{
        turnRight();
      }
      break;
    case LEFT:
      if(urgency > 0){
        turnLeft1(buf[1]);
      }else{
        turnLeft();
      }
      break;
    default:
      break;
  }
  delay(1);
}
