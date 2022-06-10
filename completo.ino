#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>
#include <math.h>


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int pin1 = 9;
int pin2 = 10;
int pin3 = 11;
int pin4 = 12;

int aumento=10;
int aumentoPinza = 70;

int aum1 = -10;
int aum2 = 70;

int contadorx = 40;
int contadory = 120;
int contadorz = 40;
int contadorw = 120;

int a1 = 0.081;
int a2 = 0.071; 
double pos_x;
double pos_y;
double pos_z;
int costheta2;
int theta1;
int theta2;      

String caso;
String caso2;
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
//#define ENC_IN_LEFT_A 2
//#define ENC_IN_RIGHT_A 3

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.

#define EN_L 8
#define IN1_L 7
#define IN2_L 6
#define EN_R 3
#define IN1_R 5
#define IN2_R 4
double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.03375, wheel_sep = 0.1533;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub5("/turtlebot_cmdVel", &messageCb );

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);



void messageS( const std_msgs::String& msg) {
  caso2 = msg.data;

}

void messageA( const std_msgs::String& msg) {

  //  aumento= int(msg.data);

}

void messageC(const geometry_msgs::Twist& msg) {
  pos_x = msg.linear.x;
  pos_y = msg.linear.y;
  costheta2 = (1/(2*a1*a2))*((pow(pos_x,2)+pow(pos_y,2))-(pow(a1,2)+pow(a2,2)));
  theta1 = (acos((1/(pos_x*pos_x+pos_y*pos_y))*((pos_x*(a1+a2*costheta2))+(pos_y*a2*sqrt(1-(costheta2*costheta2))))))*(180/3.14);
  theta2 = (acos(1/(2*a1*a2)*((pow(pos_x,2)+pow(pos_y,2))-(pow(a1,2)+pow(a2,2)))))*(180/3.14);

}

void messageCS( const std_msgs::String& msg) {
  caso = msg.data;

}

ros::Subscriber<std_msgs::String> sub("/robot_manipulator_teleop", &messageS );
ros::Subscriber<std_msgs::String> sub2("/angulo", &messageA);
ros::Subscriber<geometry_msgs::Twist> sub3("/robot_manipulator_planner", &messageC );
ros::Subscriber<std_msgs::String> sub4("/robot_manipulator_camara", &messageCS);

geometry_msgs::Twist posicion_brazo;
ros::Publisher Pos_Bra("/turtlebot_position", &posicion_brazo);

void setup(){
  Motors_init();
  nh.subscribe(sub);
  Serial.begin(57600);

  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo4.attach(pin4);

  servo1.write(40);
  servo2.write(180);
  servo3.write(130);
  servo4.write(120);

  nh.initNode();
  nh.subscribe(sub5);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);

  nh.initNode();
  nh.advertise(Pos_Bra);
  posicion_brazo.linear.x=0;
  posicion_brazo.linear.y=0;
  Pos_Bra.publish(&posicion_brazo);
}

void loop(){
  MotorL(w_l*5);
  MotorR(w_r*5);
  nh.spinOnce();

  // Record the time
  currentMillis = millis();
  if(caso2 == "u") {
    for(int i=contadorx;i<=contadorx+aumento;i++){
      servo1.write(i);   
    }
    contadorx=contadorx+aumento; 
    posicion_brazo.linear.x=-contadorx;
    Pos_Bra.publish(&posicion_brazo);
    delay(150);
  }
  else if (caso2 == "j") {
    for(int j=contadorx;j>=contadorx-aumento;j--){
      servo1.write(j);   
    }
    contadorx=contadorx-aumento;
    posicion_brazo.linear.x=-contadorx;
    Pos_Bra.publish(&posicion_brazo);
    delay(150);
  }
  else if (caso2 == "i") {
    for(int k=contadory;k<=contadory+aumento;k++){
      servo2.write(180);   
    }
    contadory=contadory+aumento;
    posicion_brazo.linear.y=-contadory;
    Pos_Bra.publish(&posicion_brazo);
    delay(150);
  }
  else if (caso2 == "k") {
    for(int l=contadory;l>=contadory-aumento;l--){
      servo2.write(0);   
    }
    contadory=contadory-aumento;
    posicion_brazo.linear.y=-contadory;
    Pos_Bra.publish(&posicion_brazo);
    delay(150);
  }

  else if (caso2 == "o") {
    for(int a=contadorz;a<=contadorz+aumento;a++){
      servo3.write(180);   
    }
    contadorz=contadorz+aumento;
    delay(150);
  }
  else if (caso2 == "l") {
    for(int b=contadorz;b>=contadorz-aumento;b--){
      servo3.write(0);   
    }
    contadorz=contadorz-aumento;
    delay(150);
  }

  else if (caso2 == "y") {
    for(int c=contadorw;c<=contadorw+aumentoPinza;c++){
      servo4.write(c);   
    }
    contadorw=contadorw+aumentoPinza;
    delay(150);
  }
  else if (caso2 == "h") {
    for(int d=contadorw;d>=contadorw-aumentoPinza;d--){
      servo4.write(d);   
    }
    contadorw=contadorw-aumentoPinza;
    delay(150);
  }

  if((pos_x && pos_y) > 0.0){
    servo1.write(theta2);
    servo2.write(theta2);
    delay(150);

  }

  if (caso == "izq1arr1") {
    servo1.write(10+aum1);
    servo3.write(aum2);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq1arr2") {
    servo1.write(10+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq1ctr") {
    servo1.write(10+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq1aba2") {
    servo1.write(10+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq1aba1") {
    servo1.write(10+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq2arr2") {
    servo1.write(20+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq2ctr") {
    servo1.write(20+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq2aba2") {
    servo1.write(20+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq2aba1") {
    servo1.write(20+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  else if (caso == "izq3arr1") {
    servo1.write(25+aum1);
    servo3.write(aum2);
    //servo2.write(120);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq3arr2") {
    servo1.write(25+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq3ctr") {
    servo1.write(25+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq3aba2") {
    servo1.write(25+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "izq3aba1") {
    servo1.write(25+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  else if (caso == "ctrarr1") {
    servo1.write(aum1);
    servo2.write(aum2);
    //servo3.write(120);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "ctrarr2") {
    servo1.write(aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "ctrctr") {
    servo1.write(aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "ctraba2") {
    servo1.write(aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "ctraba1") {
    servo1.write(aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  else if (caso == "der1arr1") {
    servo1.write(40+aum1);
    servo3.write(aum2);
    //servo2.write(120);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der1arr2") {
    servo1.write(40+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der1ctr") {
    servo1.write(40+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der1aba2") {
    servo1.write(40+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der1aba1") {
    servo1.write(40+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  else if (caso == "der2arr1") {
    servo1.write(50+aum1);
    servo3.write(aum2);
    //servo2.write(120);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der2arr2") {
    servo1.write(50+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der2ctr") {
    servo1.write(50+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der2aba2") {
    servo1.write(50+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der2aba1") {
    servo1.write(50+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  else if (caso == "der3arr1") {
    servo1.write(60+aum1);
    servo3.write(aum2);
    //servo2.write(120);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der3arr2") {
    servo1.write(60+aum1);
    servo3.write(aum2);
    //servo2.write(135);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der3ctr") {
    servo1.write(60+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der3aba2") {
    servo1.write(60+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }
  else if (caso == "der3aba1") {
    servo1.write(60+aum1);
    servo3.write(aum2);
    //servo2.write(140);
    delay(500);
    //servo4.write(80);
  }

  nh.spinOnce();
}

void Motors_init(){
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}
void MotorL(int Pulse_Width1){
  if (Pulse_Width1 > 0){
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }
  if (Pulse_Width1 < 0){
    Pulse_Width1=abs(Pulse_Width1);
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }
  if (Pulse_Width1 == 0){
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
}
void MotorR(int Pulse_Width2){
  if (Pulse_Width2 > 0){
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
  if (Pulse_Width2 < 0){
    Pulse_Width2=abs(Pulse_Width2);
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }
  if (Pulse_Width2 == 0){
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
}

