#include "DFRobot_tof.h"
#define _8X8
#ifdef _8X8
#define DATA 8
#else
#define DATA 4
#endif

/**
 * @brief 根据小车车身12x12算出的
 * @n 后续需要改为动态算法
 * @n 几根射线判断车能通过的最小值
 * 
 */
#if 0
#define LINE2MIN    762
#define LINE4MIN    300
#define LINE6MIN    152
#define LINE8MIN    104
#else 
#define LINE2MIN    280
#define LINE4MIN    180
#define LINE6MIN    132
#define LINE8MIN    100
#define LINE1MIN    100//左上角和右上角阈值判断

#endif

DFRobot_tof tof;
uint16_t buf[DATA*DATA];
char report[128];

uint16_t histogram[8];
uint16_t sector_count[8];
uint16_t sector_count1[8];

uint8_t _face1 = 0;
uint8_t _face2 = 0;
uint8_t _face3 = 0;
uint8_t _face4 = 0;

int sum1 = 0,sum2= 0;

#define NUM_SECTORS 8

#define L_M  1
#define R_M 2
#define CW 0
#define CCW 1

uint8_t nowDirection = 0;

#define FORWARD  0b010
#define BACKWARD  2
#define LEFT  0b100
#define RIGHT  0b001

uint8_t sumData = 0;
int sumBuf[10][8]={0};

int lastHistogram[8]={0};

bool LineAferTurn = false;

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
  motor(speed*4+20, CW, R_M);
}

// 右转
void turnRight1(uint8_t speed) {
  motor(speed*4+20, CW, L_M);
  motor(20, CW, R_M);
 
}

// 停止
void stop() {
  motor(0, CW, L_M);
  motor(0, CW, R_M);
}

uint8_t state = 0;
uint8_t state1 = 0;//上层转换标志位
uint8_t stateMotor = 0;
void setup(void){
  Serial.begin(115200);
  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
   Serial.println("begin success");
  while(tof.initSensor(0,DATA,0) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
  
  
}

/**
 * @brief 平滑数据
 * 
 */
static void SmoothingData(){
  for (int i = 3; i < 5; i++) {
    for (int j = 0; j < 8; j++) {
      float angle = (j - (8 / 2)) * 8.5;
      int sector = (int)(angle / 8.5);
      sector += 4;
      if (sector >= NUM_SECTORS) {
         sector = NUM_SECTORS - 1;
      }
            histogram[sector] += buf[i*8 + j];
            sector_count[sector]++;
        }
    }
    for (int i = 0; i < NUM_SECTORS; i++) {
        if (sector_count[i] > 0) {
            histogram[i] /= sector_count[i];
        }
    }
}


/**
 * @brief 求取n个射线的和，并且当射线的最大值大于传入参数时，以传入参数为准
 * 
 * @param begin 
 * @param end 
 * @param max 
 * @return uint16_t 
 */
static uint16_t raySum(uint8_t begin, uint8_t end, uint16_t max){
  uint16_t tempSum = 0;
  for(uint8_t i = begin; i< end; i++){
    if(histogram[i] > max){
      tempSum += max;
    }else{
      tempSum += histogram[i];
    }
  }
  return tempSum;
}

/**
 * @brief 求两个射线的差值
 * 
 * @param first 
 * @param second 
 * @return int 
 */
static int rayDiff(uint8_t first, uint8_t second){
  return (int)(histogram[first] - histogram[second]);
}

/**
 * @brief 获取某个数组的某段范围内的最小值
 * 
 * @param array 
 * @param start 
 * @param end 
 * @return int 
 */
static int findMin(uint16_t *array, int start, int end) {
  if (start > end) {
    // 如果起始索引大于结束索引，返回一个大值表示错误
    return -1; 
  }
  
  int minValue = array[start];
  
  for (int i = start + 1; i <= end; i++) {
    if (array[i] < minValue) {
      minValue = array[i];
    }
  }
  return minValue;
}

static void printBinary(uint8_t value) {
  for (int i = 2; i >= 0; i--) {
    if (value & (1 << i)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
  Serial.println();
}

#if 0
static void firstFloor(int tempSub){
  if(abs(tempSub)  > 100){
    Serial.println("100");
    if(tempSub > 0){
      turnLeft1(15);
    }else{
      turnRight1(15);
    }
  }else if(abs(tempSub)  > 200){
    Serial.println("200");
    if(tempSub > 0){
      turnLeft1(20);
    }else{
      turnRight1(20);
    }
  }else if(abs(tempSub)  > 300){
    Serial.println("300");
    if(tempSub > 0){
      turnLeft1(30);
    }else{
      turnRight1(30);
    }
  }else  if(abs(tempSub)  > 400){
      if(tempSub > 0){
      turnLeft1(35);
    }else{
      turnRight1(35);
    }
  }else{
    moveForward();
  }
}

static void secondFloor(){
  int sub = histogram[0] - histogram[7];
  int sub1 = histogram[1] - histogram[6];
  if(abs(sub)  > 100 && abs(sub1)  > 100){//检测阈值
    if(sub > 0){
      state = 0;
    }else{
       state = 1;
    }
  }else{

  } 
  if(histogram[2] > 200 && histogram[3] > 200 && histogram[4] > 200 && histogram[5] > 200 ){//直行  
    }else{
      if(state == 0){
        turnLeft();
      }else if(state == 1){
        turnRight();
      }
  }
}

static void logicalProcess(){

}

#else
static uint8_t firstFlootDir = 0;
static uint8_t firstFlootSpeed = 0;
static void firstFloor(int tempSub){
  //根据最外层的情况，反馈一个行进趋势，左、中、右
  firstFlootDir = 0;
  firstFlootSpeed = 0;
  if(abs(tempSub) > 100){
    if(tempSub > 0){//左转趋势
      firstFlootDir = 0b100;
    }else{//右转趋势
      firstFlootDir = 0b001;
    }
  }else{//直行趋势
    firstFlootDir = 0b010;
  }


  if(abs(tempSub)  > 100){
    firstFlootSpeed = 15;
  }else if(abs(tempSub)  > 200){
    firstFlootSpeed = 15;
  }else if(abs(tempSub)  > 300){
    firstFlootSpeed = 15;
  }else  if(abs(tempSub)  > 400){
    firstFlootSpeed = 15;
  }else{
    
  }
}


static uint8_t secondFlootDir = 0;
static uint8_t secondFlootValue = 0;
/**
 * @brief 第二层处理
 * @n 中间4条射线的值大于200？时认为可直行
 * @n 
 * 
 */
static void secondFloor(){

  int diff07 = rayDiff(0, 7);
  int diff16 = rayDiff(1, 6);

  int sum01 = raySum(0, 2, 300);
  int sum67 = raySum(6, 8, 300);
  secondFlootDir = 0;
  secondFlootValue  =0;
  //什么样的判断能够判断我可以左转，可以右转
  if(abs(diff07)  > 100 && abs(diff16)  > 100){//检测阈值

    if(diff07 > 0){//左转
      secondFlootDir |= 0b100;
    }else{//右转
      secondFlootDir |= 0b001;
    }

  }else{
    
    if(sum01 < 300){
      secondFlootDir &= 0b011;
    }else if(sum01 > 500){
      secondFlootDir |= 0b100;
     }

    if(sum67 < 300){
      secondFlootDir &= 0b110;
    }else if(sum01 > 500){
      secondFlootDir |= 0b001;
    }

  } 
  if(histogram[2] > 200 && histogram[3] > 200 && histogram[4] > 200 && histogram[5] > 200 ){//中路畅通，可直行
    secondFlootDir |= 0b010;
  }else{
    if((secondFlootDir & 0b101) != 0x00 ){//第2层告诉在不能直行的情况下，如果可以左右转，则是原地转
      secondFlootValue = 1;
    }
  }
}

/**
 * @brief 将射线分为多层，进行处理
 * @n 默认值，所有层均可以直行、左转、右转
 * @n 每一层根据实际情况，关闭直行、左转、右转
 */
static uint8_t flootDir07 = 0b111;
static uint8_t flootDir16 = 0b111;
static uint8_t flootDir25 = 0b111;
static uint8_t flootDir34 = 0b111;

//边缘碰撞
static uint8_t flootDir0 = 0b111;
static uint8_t flootDir7 = 0b111;
//转向趋势
static uint8_t dirTrend = 0b111;
static uint8_t dirTrend1 = 0b111;
static void logicalFloor(){
  flootDir07 = 0b111;
  flootDir16 = 0b111;
  flootDir25 = 0b111;
  flootDir34 = 0b111;
  dirTrend = 0b111;
  if(findMin(histogram,0,7) < LINE8MIN){//判断8根射线的最小值小于限制，则说明前面全被挡了
    flootDir07 &= 0b101;//禁止直行
  }

  if(findMin(histogram,1,6) < LINE6MIN){//判断6根射线的最小值小于限制，则说明6根射线层被挡了
     flootDir16 &= 0b101;//该层禁止直行
     if(histogram[0] < LINE6MIN){//禁止左转
        flootDir16 &= 0b011;
     }
     if(histogram[7] < LINE6MIN){//禁止右转
        flootDir16 &= 0b110;
     }
  }else{//该层允许直行
    if(histogram[0] < histogram[1] ){//禁止左转
      flootDir16 &= 0b011;
    }
    if(histogram[7] < histogram[6]){//禁止右转
      flootDir16 &= 0b110;
    }
  }

  if(findMin(histogram,2,5) < LINE4MIN){//判断4根射线的最小值小于限制，则说明4根射线层被挡了
    flootDir25 &= 0b101;//该层禁止直行
    if((histogram[0] < LINE4MIN) || (histogram[1] < LINE4MIN)){//禁止左转
      #if 0
        if(buf[7] < LINE1MIN || buf[63] < LINE1MIN){//表示最上层检测到碰撞则只能直行
          flootDir25 &= 0b010;//该层禁止左转
        }else{
          flootDir25 &= 0b011;//该层禁止左转
        }
      #else 
        flootDir25 &= 0b011;//该层禁止左转
      #endif
      
    }
    if((histogram[6] < LINE4MIN) || (histogram[7] < LINE4MIN)){//禁止右转
    #if 0
      if(buf[0] < LINE1MIN || buf[56] < LINE1MIN){//表示最上层检测到碰撞则只能直行
          flootDir25 &= 0b010;//该层禁止左转
        }else{
          flootDir25 &= 0b110;//该层禁止右转
        }
      #else
       flootDir25 &= 0b110;//该层禁止右转
      #endif
      
    }
  }else{//该层允许直行
    if((histogram[0] < LINE4MIN) || (histogram[1] < LINE4MIN)){//禁止左转
      flootDir25 &= 0b011;//该层禁止左转
    }
    if((histogram[6] < LINE4MIN) || (histogram[7] < LINE4MIN)){//禁止右转
      flootDir25 &= 0b110;//该层禁止右转
    }
  }
  if(findMin(histogram,3,4) < LINE2MIN){//判断2根射线的最小值小于限制，则说明2根射线成被挡了
     flootDir34 &= 0b101;//该层禁止直行
    if((histogram[1] < LINE2MIN) || (histogram[2] < LINE2MIN)){//禁止左转
      #if 0
        if(buf[7] < LINE1MIN || buf[63] < LINE1MIN){//表示最上层检测到碰撞则只能直行
          flootDir34 &= 0b010;//该层禁止左转
        }else{
          flootDir34 &= 0b011;//该层禁止左转
        }
      #else
        flootDir34 &= 0b011;//该层禁止左转
      #endif
    }
    if((histogram[5] < LINE2MIN) || (histogram[6] < LINE2MIN)){//禁止右转
      #if 0
        if(buf[0] < LINE1MIN || buf[56] < LINE1MIN){//表示最上层检测到碰撞则只能直行
          flootDir34 &= 0b010;//该层禁止左转
        }else{
          flootDir34 &= 0b110;//该层禁止右转
        }
      #else
       flootDir34 &= 0b110;//该层禁止右转
      #endif 
    }
  }else{//直行射程范围内没有被挡
    if((histogram[1] < LINE2MIN) || (histogram[2] < LINE2MIN)){//禁止左转
      flootDir34 &= 0b011;//该层禁止左转
    }
    if((histogram[5] < LINE2MIN) || (histogram[6] < LINE2MIN)){//禁止右转
      flootDir34 &= 0b110;//该层禁止右转
    }
  }
  int diff07 = rayDiff(0, 7);
  int diff16 = rayDiff(1, 6);
  if((diff07 > 100) && (diff16 > 100)){//左转趋势
    dirTrend &= 0b110;//关闭右转趋势
    dirTrend1 = dirTrend;
  }else if((diff07 < -100) && (diff16 < -100)){
    dirTrend &= 0b011;//关闭左转趋势
    dirTrend1 = dirTrend;
  }else{
    dirTrend = dirTrend1;
    }
  Serial.print("07:");printBinary(flootDir07);
  Serial.print("16:");printBinary(flootDir16);
  Serial.print("25:");printBinary(flootDir25);
  Serial.print("34:");printBinary(flootDir34);
  Serial.print("FF:");printBinary(dirTrend);
}

/**
 * @brief 逻辑处理，真正的控制命令执行处
 * 
 */
static void logicalProcess(){
#if 0
  if((firstFlootDir & secondFlootDir) == 0){//第一层和第二层的预期完全不一致，则听第二层的
    if((secondFlootDir >> 1) & 0x01){//判断是否有直行，有则直行
      moveForward();
    }else{//在不能直行的情况下，只能原地左转或右转
      if((secondFlootDir) & 0x01 ){//判断不能直行的情况下，是否可以右转
        turnRight();
      }else{//则左转
        turnLeft();
      }
    }
  }else{//有相同预期的情况下，则直行都认可的方向
    switch((firstFlootDir & secondFlootDir)){
      case 0b001:
        //右转
        if(secondFlootValue){
          turnRight();
        }else{
          turnRight1(firstFlootSpeed);
        }
        break;
      case 0b010:
        //直行
        moveForward();
        break;
      case 0b100:
        //左转
        if(secondFlootValue){
          turnLeft();
        }else{
          turnLeft1(firstFlootSpeed);
        }
        break;
    }
  }
#else
  uint8_t realDir = FORWARD;
  uint8_t realSpeed = 0;
  if(((flootDir07 >> 1) & (flootDir16 >> 1) & (flootDir25 >> 1) & (flootDir34 >> 1) & 0x01) == 0x01){//所有层都告诉我可以直行
    // if(buf[63] < 100 || buf[7] < 100){//如果左边最低点会碰撞
    //   realDir = LEFT;
    // }else if(buf[56] < 100 || buf[0] < 100){
    //   realDir = RIGHT;
    // }else{
      realDir = FORWARD;
      Serial.println("FORWARD");
   // }
    
  }else if((flootDir07 >> 2) & (flootDir16 >> 2) & (flootDir25 >> 2) & (flootDir34 >> 2)){//所有层都告诉我可以左转，那我就左转
    realDir = LEFT;
    realSpeed = 15;
  }else if((flootDir07 & 0x01) & (flootDir16 & 0x01) & (flootDir25 & 0x01) & (flootDir34 & 0x01)){//所有层都告诉我可以右转，那我就右转
    realDir = RIGHT;
    realSpeed = 15;
  }else{
    if(((flootDir07>>1) & 0x01) != 0x01){//该层不可直行，原地左转或右转
      if((dirTrend >> 2)){//左转趋势
        realDir = LEFT;
      }else{
        realDir = RIGHT;
      }
      realSpeed = 0;
    }else if(((flootDir16 >> 1) & 0x01) != 0x01){//该层不可直行
      if((flootDir16 & dirTrend)>>2){//左转趋势同时该层支持左转
        #if 0
            if(buf[56] < 100 || buf[0] < 100){
              realDir = RIGHT;
            }else{
              realDir = LEFT;
            }
        #else
            realDir = LEFT;
        #endif
      }else if((flootDir16 & dirTrend) & 0x01){//右转趋势同时该层支持右转
        #if 0
            if(buf[63] < 100 || buf[7] < 100){
              
              realDir = LEFT;
            }else{
              realDir = RIGHT;
            }
        #else
            realDir = RIGHT;
        #endif
      }else{
        if(dirTrend >> 2){//左转
          realDir = LEFT;
         }else if(dirTrend &0x01){//右转
            realDir = RIGHT;
         }else{
          realDir = LEFT;
          Serial.println("flootDir16 error");
          }
      }
      LineAferTurn = false;
      realSpeed = 0;
    }else if(((flootDir25>>1) & 0x01) != 0x01){//该层不可直行
      if(LineAferTurn){
        Serial.println("LAT=true");
        realDir = FORWARD;
      }else{
        if(flootDir25 == 0x00 && histogram[3] > 320){
          LineAferTurn = true;
          Serial.println("enable=true");
          //realDir = FORWARD;
        }else{
          if((flootDir25 & dirTrend) >> 2){///左转趋势同时该层支持左转
          #if 0
            if(buf[56] < 100 || buf[0] < 100){
              realDir = RIGHT;
            }else{
              realDir = LEFT;
            }
          #else
            realDir = LEFT;
          #endif
          }else if(((flootDir25 & dirTrend)& 0x01)){////右转趋势同时该层支持右转
            #if 0
            if(buf[63] < 100 || buf[7] < 100){
              realDir = LEFT;
            }else{
              realDir = RIGHT;
            }
          #else
            realDir = RIGHT;
          #endif
          }else{//flootDir25 数据为000 根据趋势做方向判断
            if(dirTrend >> 2){//左转
              realDir = LEFT;
             }else if(dirTrend &0x01){//右转
                realDir = RIGHT;
             }else{
              realDir = LEFT;
              Serial.println("flootDir25 error");
              }
          }
          realSpeed = 15;
        }
      }
    }else if(((flootDir34 >> 1 ) &0x01) != 0x01){//该层不能直行
      if((flootDir34 & dirTrend)>>2){///左转趋势同时该层支持左转
        #if 0
            if(buf[56] < 100 || buf[0] < 100){
              realDir = RIGHT;
            }else{
              realDir = LEFT;
            }
          #else
            realDir = LEFT;
          #endif
      }else if((flootDir34 & dirTrend) & 0x01){//右转趋势同时该层支持右转
        #if 0
            if(buf[63] < 100 || buf[7] < 100){
              realDir = LEFT;
            }else{
              realDir = RIGHT;
            }
          #else
            realDir = RIGHT;
          #endif
      }
    }else{//则按默认直行即可
      realDir = FORWARD;
    }
    
  }
  Serial.print("Dir=");printBinary(realDir);
  Serial.print("Speed=");Serial.println(realSpeed);
  switch(realDir){
    case FORWARD:
      moveForward();
      break;
    case RIGHT:
      if(realSpeed > 0){
        turnRight1(realSpeed);
      }else{
        turnRight();
      }
      break;
    case LEFT:
      if(realSpeed > 0){
        turnLeft1(realSpeed);
      }else{
        turnLeft();
      }
      break;
    default:
      break;
  }
#endif
}

#endif



void loop(void){
  tof.getAllData(buf);
  _face1 = 0;
  _face2 = 0;
  _face3 = 0;
  sum1 = 0;
  sum2 = 0;
  
  memset(sector_count, 0, sizeof(sector_count));
  memset(histogram, 0, sizeof(histogram));
  //平滑数据
  SmoothingData();
  
  /*
  for(uint8_t i = 0;i < 8;i++){
    sumBuf[sumData][i] = histogram[i];
  }
  
  sumData = (sumData + 1) % 10;

  for(uint8_t i = 0;i < 4; i++){
    for(uint8_t j = 0;j < 10;j++){
      sum1 +=sumBuf[j][i];
    }
  }

  for(uint8_t i = 4;i < 8; i++){
    for(uint8_t j = 0;j < 10;j++){
      sum2 +=sumBuf[j][i];
    }
     
  }
  Serial.print("L4 = ");
  Serial.print(sum1);
  Serial.print(",R4 = ");
  Serial.print(sum2);*/
  for(uint8_t i = 0; i < 8; i++){
    for(uint8_t j = 0; j < 8; j++){
      Serial.print(buf[i*8+j]);
      Serial.print(",");
    }
    Serial.println(" ");
  }
  Serial.println("------------------------------");
  //数据切片
 // SlicingData();
  /*
  //计算左边4跟射线的和，单根射线最大限制300
  sum1 = raySum(0 ,4, 300);
  //计算右边4跟射线的和，单根射线最大限制300
  sum2 = raySum(4 ,8, 300);
  int sub2 = sum1 - sum2;
  Serial.print("L4 = ");
  Serial.print(sum1);
  Serial.print(",R4 = ");
  Serial.print(sum2);
  Serial.print(",sub =");
  Serial.println(sub2);

  //firstFloor(sub2);
  //secondFloor();
  */
  logicalFloor();
  logicalProcess();
//   int sub2 = histogram[0] - histogram[1];
//   float angleDegrees = 30.0;
//   // 将角度转换为弧度
//   float angleRadians = radians(angleDegrees);
//   float tanValue = tan(angleRadians);
//   //float cosValue  = cos(angleRadians);
//   float len0 = histogram[0] * tanValue;
//   //float hight0 = histogram[0] * cosValue;
//   Serial.print("len0 =");
//   Serial.print(len0);
// //  Serial.print(",hight0 =");
// //  Serial.println(hight0);

//   angleDegrees = 90.0 -(60.0 + (60.0 / 7.0));
//   // 将角度转换为弧度
//   angleRadians = radians(angleDegrees);
//   tanValue = tan(angleRadians);
//   //cosValue  = cos(angleRadians);
//   float len1 = histogram[1] * tanValue;
// //  float hight1 = histogram[1] * cosValue;
//   Serial.print("len1 =");
//   Serial.print(len1);
//  Serial.print(",hight1 =");
//  Serial.println(hight1);
  
//  Serial.print("lastHistogram[0] = ");
//  Serial.print(lastHistogram[0]);
  
  lastHistogram[0] = histogram[0];
 
  delay(1);
  
}