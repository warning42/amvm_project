#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <DynamixelWorkbench.h>
#include <Servo.h>


// system define
const int BAUDRATE = 57600;

// sonic define
const int trigPin_front = 50;
const int echoPin_front = 52;
float distance_front_tmp; //for sonic distance_front

ros::NodeHandle nh;

std_msgs::Float64 distance_front;
ros::Publisher sonic_chatter_front("sonic_chatter_front", &distance_front);

///////////////////////////////sonic///////////////////////////////
void sonic_setup(const int trigPin, const int echoPin){
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
}

float sonic_distance(const int trigPin, const int echoPin){
  float distance;
  long duration;
    
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration*0.034/2;

  return distance;
}
///////////////////////////////////////////////////////////////////

// motor define
const int BAUDRATE_TO_DXL = 1000000;
const int LEFT_ID = 1;
const int RIGHT_ID = 2;

DynamixelWorkbench dxl_wb;

int motor_L_sub = 0;
void motorLCallback(const std_msgs::Int32& velL){
    motor_L_sub = velL.data;
}

int motor_R_sub = 0;
void motorRCallback(const std_msgs::Int32& velR){
    motor_R_sub = velR.data;
}

ros::Subscriber<std_msgs::Int32> motorL_chatter("motorL_chatter", motorLCallback);
ros::Subscriber<std_msgs::Int32> motorR_chatter("motorR_chatter", motorRCallback);

///////////////////////////////motor///////////////////////////////
void motor_pre_setup(const int BAUDRATE_TO_DXL, const int motor_ID_1, const int motor_ID_2){
  dxl_wb.begin("", BAUDRATE_TO_DXL);
  dxl_wb.ping(motor_ID_1);
  dxl_wb.ping(motor_ID_2);
  dxl_wb.jointMode(motor_ID_1);
  dxl_wb.jointMode(motor_ID_2);
}

void motor_velocity_setup(const int motor_ID_1, const int motor_ID_2){  
  dxl_wb.wheelMode(motor_ID_1);
  dxl_wb.wheelMode(motor_ID_2);
  dxl_wb.goalVelocity(motor_ID_1, 0);  
  dxl_wb.goalVelocity(motor_ID_2, 0);  
}

void motor_veleocity_control(const int motor_ID, const int motor_vel){
  dxl_wb.goalVelocity(motor_ID, motor_vel);  
}
///////////////////////////////////////////////////////////////////

///////////////////////////////servo///////////////////////////////
Servo servo_1;

void servoCallback(const std_msgs::Int32& endF){
    end_flag = endF.data;
}

ros::Subscriber<std_msgs::Int32> servo_chatter("servo_chatter", servoCallback);
///////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(BAUDRATE);

  nh.initNode();

  /////// sonic chatter advertisement
  nh.advertise(sonic_chatter_front);

  /////// motor setup
  motor_pre_setup(BAUDRATE_TO_DXL, LEFT_ID, RIGHT_ID);
  motor_velocity_setup(LEFT_ID, RIGHT_ID);
  motor_veleocity_control(LEFT_ID, 0);
  motor_veleocity_control(RIGHT_ID, 0);

  /////// sonic setup
  sonic_setup(trigPin_front, echoPin_front);

  /////// servo motor setup
  servo_1.attach(6);
  servo_1.write(90);
}

int sonic_cnt = 0;
void loop() {
  if(sonic_cnt == 50){
    sonic_cnt = 0;
    distance_front_tmp = sonic_distance(trigPin_front, echoPin_front);
    distance_front.data = distance_front_tmp;
    sonic_chatter_front.publish(&distance_front);
  }
  else{
    sonic_cnt++;
    /////// new part - motor subscribe
    /////// subscribe motor data
    nh.subscribe(motorL_chatter);
    nh.subscribe(motorR_chatter);
    motor_veleocity_control(LEFT_ID, motor_L_sub);
    motor_veleocity_control(RIGHT_ID, motor_R_sub);
  }
  
  //////// subscribe servo motor data
  nh.subscribe(servo_chatter);
  if(end_flag == 1){
    servo_1.write(0);
  }
  else{
    servo_1.write(90);
  }
  nh.spinOnce();
}
