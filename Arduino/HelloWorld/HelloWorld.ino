

/*-
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
///#include <std_msgs/Bool.h>

#define SPEED 170
#define TURN_SPEED 170
#define vel_max 1.0


#define speedPinR 12  //  Front Wheel PWM pin connect Model-Y M_B ENA 
#define RightMotorDirPin1 7    //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define RightMotorDirPin2 8   //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
#define LeftMotorDirPin1 5    //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2 6   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 11   //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 4   //  Rear Wheel PWM pin connect Left Model-Y M_A ENA 
#define RightMotorDirPin1B 26    //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define RightMotorDirPin2B 28    //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1) 
#define LeftMotorDirPin1B 22    //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define LeftMotorDirPin2B 24  //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 3    //  Rear Wheel PWM pin connect Model-Y M_A ENB

ros::NodeHandle  nh;

const float pi=3.14159;

float linear_x=0.00;//speed x [m]
float linear_y=0.00;//speed y [m]
float linear_z=0.00;//speed y [m]
float angle_z=0.00;//speed z [rad/s]

bool control_mode;

float w[4]         = {0.0001,0.0001,0.0001,0.0001} ;
float motor_vel[4] = {0.001 ,0.001 ,0.001 ,0.001 } ;
float a[4][3];
const float alpha=45;

const float wheel_radius=0.035;
const float ww=0.1;//width/2 [m]
const float lw=0.075;//length/2 [m]

std_msgs::Int32 mode_msg;
//ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

//----------------------------ROS  callback----------------------------------------------------
void messageCb_auto(const geometry_msgs::Twist& twist) {
  if(mode_msg.data==2){
    char log_msg[100]; // ログメッセージを格納するための十分な大きさの文字列バッファ
    nh.loginfo("auto_mode");

    // Twist メッセージの内容を文字列にフォーマット
    sprintf(log_msg, "Received Twist: linear x: %f, y: %f, z: %f; angular z: %f",
            twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);

    // フォーマットされた文字列をログに出力
    nh.loginfo(log_msg);
    linear_x = twist.linear.x;
    linear_y = twist.linear.y;
    linear_z = twist.linear.z;
    angle_z = twist.angular.z;
    //nh.loginfo("topic echo");
  }
}

void messageCb_joy(const geometry_msgs::Twist& twist) {
  if(mode_msg.data==1){
    nh.loginfo("joy_mode");
    linear_x = twist.linear.x;
    linear_y = twist.linear.y;
    linear_z = twist.linear.z;
    angle_z = twist.angular.z;
    //nh.loginfo("topic echo");
  }
}


void modeCb(const std_msgs::Int32& mode){
  int received_value = mode.data;
  //nh.loginfo("Received mode: ");
  mode_msg = mode; 
//  if(received_value == 1){
//    nh.loginfo("Mode is auto");
//  }else if(received_value == 2){
//  nh.loginfo("Mode is joy");
//  }else{
//    nh.loginfo("Mode is neither auto nor joy");
//  }

}

void stop_Stop()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);

  stop_Stop();
}

void front_L(int front_L_pwm){
  //Serial.println("front_L_pwm");
  //Serial.println(front_L_pwm);
  if(front_L_pwm >= 0){
    digitalWrite(LeftMotorDirPin1,LOW);
    digitalWrite(LeftMotorDirPin2,HIGH);
  }else{
    digitalWrite(LeftMotorDirPin1,HIGH);
    digitalWrite(LeftMotorDirPin2,LOW);
  }
  analogWrite(speedPinL,255*abs(front_L_pwm)*0.01);
}

void front_R(int front_R_pwm){
  //Serial.println("front_R_pwm");
  //Serial.println(front_R_pwm);
  if(front_R_pwm >= 0){
    digitalWrite(RightMotorDirPin1,LOW);
    digitalWrite(RightMotorDirPin2,HIGH);
  }else{
    digitalWrite(RightMotorDirPin1,HIGH);
    digitalWrite(RightMotorDirPin2,LOW);
  }
  analogWrite(speedPinR,255*abs(front_R_pwm)*0.01);
}

void back_L(int back_L_pwm){
  //Serial.println("back_L_pwm");
  //Serial.println(back_L_pwm);
  if(back_L_pwm >= 0){
    digitalWrite(LeftMotorDirPin1B,HIGH);
    digitalWrite(LeftMotorDirPin2B,LOW);
  }else{
    digitalWrite(LeftMotorDirPin1B,LOW);
    digitalWrite(LeftMotorDirPin2B,HIGH);
  }
  analogWrite(speedPinLB,255*abs(back_L_pwm)*0.01);
}

void back_R(int back_R_pwm){
 // Serial.println("back_R_pwm");
 // Serial.println(back_R_pwm);
  if(back_R_pwm >= 0){
    digitalWrite(RightMotorDirPin1B,LOW);
    digitalWrite(RightMotorDirPin2B,HIGH);
  }else{
    digitalWrite(RightMotorDirPin1B,HIGH);
    digitalWrite(RightMotorDirPin2B,LOW);
  }
  //Serial.println((int)(255*abs(back_R_pwm)*0.01));
  analogWrite(speedPinRB,(int)(255*abs(back_R_pwm)*0.01));
}

float front_L_pwm = 0;
float front_R_pwm = 0;
float back_L_pwm = 0;
float back_R_pwm = 0;

void convert_to_pwm(float* motor_vel){
  front_L_pwm = motor_vel[0]/vel_max*100;
  front_R_pwm = motor_vel[1]/vel_max*100;
  back_L_pwm = motor_vel[2]/vel_max*100;
  back_R_pwm = motor_vel[3]/vel_max*100;

  front_L(front_L_pwm);
  delay(10);
  front_R(front_R_pwm);
  delay(10);
  back_L(back_L_pwm);
  delay(10);
  back_R(back_R_pwm);
  delay(10);
}


void speed_cal(){
  motor_vel[0]=(a[0][0]*linear_x + a[0][1]*linear_y + a[0][2]*angle_z) *wheel_radius*pi ; //[m/s]
  motor_vel[1]=(a[1][0]*linear_x + a[1][1]*linear_y + a[1][2]*angle_z) *wheel_radius*pi;  //[m/s]
  motor_vel[2]=(a[2][0]*linear_x + a[2][1]*linear_y + a[2][2]*angle_z) *wheel_radius*pi ; //[m/s]
  motor_vel[3]=(a[3][0]*linear_x + a[3][1]*linear_y + a[3][2]*angle_z) *wheel_radius*pi ; //[m/s]
  convert_to_pwm(motor_vel);
}




//subscribe
ros::Subscriber<geometry_msgs::Twist> sub_auto("cmd_vel_auto", &messageCb_auto);
ros::Subscriber<geometry_msgs::Twist> sub_joy("cmd_vel_joy", &messageCb_joy);
ros::Subscriber<std_msgs::Int32>  sub_mode("mode", &modeCb);



void setup()
{
  init_GPIO();
  nh.getHardware()->setBaud(115200);
  //Serial.begin(9600);
  nh.initNode();
  //nh.advertise(chatter);
  nh.subscribe(sub_joy);
  nh.subscribe(sub_auto);
  nh.subscribe(sub_mode);

  a[0][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[0][1]= -1/wheel_radius ; a[0][2]=(-1*(lw+ww)/(wheel_radius));
  a[1][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[1][1]= 1/wheel_radius ; a[1][2]=    (lw+ww)/(wheel_radius) ;
  a[2][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[2][1]= 1/wheel_radius ; a[2][2]=(-1*(lw+ww)/(wheel_radius));
  a[3][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[3][1]= -1/wheel_radius ; a[3][2]=    (lw+ww)/(wheel_radius) ;

}

void loop()
{
  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  nh.spinOnce();
  speed_cal();
  delay(0);
}
