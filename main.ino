/* 深圳市普中科技有限公司（PRECHIN 普中）
   技术支持：www.prechin.net
 * 
 * 实验名称：超声波测距实验
 * 
 * 接线说明：HC-SR04超声波模块-->ESP32 IO
             (VCC)-->(5V)
             (Trig)-->(4)
             (Echo)-->(27)
             (GND)-->(GND)
 * 
 * 实验现象：程序下载成功后，软件串口控制台间隔一段时间输出超声波模块测量距离
 * 
 * 注意事项：需要在软件中选择"项目"-->"加载库"-->"添加一个.ZIP库..."-->选择到本实验目录下的1个压缩文件包“HCSR04-master.zip”安装即可。
 *          该库使用方法可参考：https://github.com/Teknologiskolen/HCSR04/commits?author=theresetmaster
 */

#include "public.h"
#include <afstandssensor.h>
#include "Arduino.h"
#include <ESP32Servo.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define OLED_SDA 12
#define OLED_SCL 14

Adafruit_SH1106 display(12, 14);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

Servo myServo;

int servoPin=17;

// AfstandsSensor(triggerPin, echoPin);
AfstandsSensor afstandssensor(4, 27);

void drawCircle(void) {
  for (int16_t i = 15; i < display.height(); i +=16) {
    display.drawCircle(display.width() / 2, display.height(), i, WHITE);
    //display.writeFillRect(i,16,4,4, WHITE); //模拟有目标
    display.display();
  }
}  

void drawLineRays(int degree)
{
  int16_t step_angle = degree;
  for(int16_t i=0; i< (180/step_angle);i++)
  {
    int16_t x = display.width()/2 - display.width()*cos(i*step_angle)/2;
    int16_t y = display.height() - display.height()*sin(i*step_angle);
    display.drawLine(display.width()/2, display.height(), x, y, WHITE);
    Serial.printf("x = %d ,y = %d\n", x,y);
    display.display();

  }  
}


void setup(){
  Serial.begin(115200);
// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.clearDisplay();  
  drawCircle();
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  Serial.printf("w = %d ,h = %d\n", display.width(),display.height());
  myServo.attach(servoPin);
  myServo.write(0);
  
}

int count = 0; //
const int SCALE_DEGREE = 10;
int jumps = 180/SCALE_DEGREE;
float last_distance = 0.0;
void loop(){
  if(count%(2*jumps) == 0)  
    {
      display.clearDisplay();  
      drawCircle();
    }  
    
  int cur_degree = (count%jumps) *SCALE_DEGREE;
  if(count%(2*jumps) >= jumps)  // 一个周期包括0~180和180~0 来回2程
    cur_degree = 180- cur_degree;
  
  float distance = afstandssensor.afstandCM();
  if(distance < 0.0)
  {    
    distance = afstandssensor.afstandCM();
    Serial.printf("distance < 0,once again %.2f\n",distance);
  }

  if(distance > 0.0)
  {
    
  
  //雷达测距范围 0~400cm，映射到0~64像素，
  // Serial.print("x = ");
  // Serial.println(cos((float)cur_degree/180.0*PI));
  // Serial.print("y = ");
  // Serial.println(sin((float)cur_degree/180.0*PI));
  float x = (distance*cos((float)cur_degree/180.0*PI)/400.0)*64;
  float y = (distance*sin((float)cur_degree/180.0*PI)/400.0)*64;
  
  //显示屏的坐标原点在平面的左上角
  //雷达测距的坐标原点在底边中点
  //雷达坐标系的点需要进行放射变换到屏幕坐标系中
  /**
  P' = | cos(theta)  -sin(theta)  tx |   | x |
       | sin(theta)   cos(theta)  ty | * | y |
       |     0            0        1 |   | 1 |
    theta 是两个坐标系的旋转角度， tx，ty是2个坐标系原点的平移量
    theta = 180degree , tx = 64, ty=64
    | -1  0  64|
    | 0  -1  64|
    | 0   0   1|
  **/
    int x_ = 64-(int)x;
    int y_ = 64-(int)y;

    Serial.print("x = ");
    Serial.print(x);
    Serial.print("  y = ");
    Serial.println(y);
    Serial.printf("count= %d degree =%d distance =%.2fCM  (%d,%d)\r\n",count ,cur_degree, distance,x_,y_);
    display.writeFillRect(x_,y_,2,2, WHITE);
    display.display();

  
  }
  else
  {
      delay(50);
  }
  Serial.printf("degree =%d\n" ,cur_degree); 
  myServo.write(cur_degree);
      delay(50);

  count++;
  
  
}

