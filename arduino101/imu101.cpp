/**
 * Genuino 101 + BMI150 IMU
 * 输出频率: 900Hz
 * 使用MadgwickAHRS计算欧拉角(Yaw, Pitch, Roll)
 * 修改点：
 * - 频率提升至900Hz
 * - 波特率提升至460800以支持900Hz数据传输
 * - 输出格式保持"AT,yaw,pitch,roll\r\n"
 */

#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

/**
 * 将原始加速度值转换成G单位的函数
 * 原理：
 * ±2G量程，对应raw值[-32768,32767]线性映射到[-2g,+2g]
 * 公式：a = (raw / 32768.0) * 2.0 g
 */
float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

/**
 * 将原始陀螺仪值转换成度/秒(deg/s)
 * 原理：
 * ±250 deg/s量程，对应[-32768,32767]
 * 公式：g = (raw / 32768.0) * 250 deg/s
 */
float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void setup() {
  Serial.begin(460800);  // 提升波特率到460800

  CurieIMU.begin();

  // 设置陀螺仪和加速度计采样频率为900Hz
  CurieIMU.setGyroRate(900);
  CurieIMU.setAccelerometerRate(900);

  // 初始化Madgwick滤波器, 参数为更新频率900Hz
  filter.begin(900);

  // 设置加速度计 ±2G
  CurieIMU.setAccelerometerRange(2);
  // 设置陀螺仪 ±250 deg/s
  CurieIMU.setGyroRange(250);

  // 计算采样间隔: 1秒=1000000微秒
  // 900Hz频率 => 1/900秒约0.001111秒=1111微秒
  microsPerReading = 1000000UL / 900UL;
  microsPrevious = micros();
}

void loop() {
  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    // 读取原始数据
    int aix, aiy, aiz;
    int gix, giy, giz;
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // 转换为物理单位
    float ax = convertRawAcceleration(aix);
    float ay = convertRawAcceleration(aiy);
    float az = convertRawAcceleration(aiz);
    float gx = convertRawGyro(gix);
    float gy = convertRawGyro(giy);
    float gz = convertRawGyro(giz);

    // 使用Madgwick算法更新姿态
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // 获取欧拉角(roll, pitch, yaw)
    float roll    = filter.getRoll();
    float pitch   = filter.getPitch();
    float heading = filter.getYaw(); // 作为yaw

    // 输出格式保持："AT,yaw值,pitch值,roll值\r\n"
    Serial.print("AT,");
    Serial.print(heading, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.print(roll, 2);
    Serial.print("\r\n");

    microsPrevious += microsPerReading;
  }
}


//    # 正则表达式修改为匹配"AT,yaw,pitch,roll"格式，并捕获Yaw, Pitch, Roll
//    IMU_PATTERN = re.compile(
//        r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)"
//    )