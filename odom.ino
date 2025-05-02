#include <math.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

//Cấu hình chân đọc Encoder
volatile int encoder1PinA = 2;
volatile int encoder1PinB = 3;

volatile int encoder2PinA = 18;
volatile int encoder2PinB = 19;

// Cấu hình chân động cơ
const int enA = 9;
const int enB = 10;
const int LEFT_MOTOR_IN1 = 6;  
const int LEFT_MOTOR_IN2 = 5;  
const int RIGHT_MOTOR_IN1 = 7;  
const int RIGHT_MOTOR_IN2 = 8; 

// --- Cấu hình Encoder và Bánh Xe ---
const float encoder_resolution_left = 381.0; // số xung encoder/vòng bánh trái
const float encoder_resolution_right = 371.0;
const float wheel_radius = 0.03;       // bán kính bánh xe (m)
const float wheel_base = 0.1665;         // khoảng cách giữa hai bánh xe  (m)

// --- Biến đếm xung Encoder ---
volatile long pulse_l = 0;
volatile long pulse_r = 0;
long pulse_l_prev = 0;
long pulse_r_prev = 0;

// --- Biến vị trí và hướng ---
float x = 0.0;          // Vị trí x (m)
float y = 0.0;          // Vị trí y (m)
float theta = 0.0;      // Hướng (radian)

// Biến lưu tốc độ bánh xe
float left = 0;
float right = 0;

// --- ROS ---
ros::NodeHandle nh;
geometry_msgs::Point pose_msg;
ros::Publisher pose_pub("odom_raw", &pose_msg); // Đổi tên topic từ "odom" sang "odom_raw"


void robot_control(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 2) {
     left = msg.data[0];
     right = msg.data[1];
    if (left >= 0) {
    digitalWrite(LEFT_MOTOR_IN1, 0); 
    digitalWrite(LEFT_MOTOR_IN2, 1);
    analogWrite(enA, left);
  }
  else if(left <= 0) {
    digitalWrite(LEFT_MOTOR_IN1, 1); 
    digitalWrite(LEFT_MOTOR_IN2, 0);
    analogWrite(enA, abs(left));
  }

  // Điều khiển động cơ phải
   if (right >= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, 0); 
    digitalWrite(RIGHT_MOTOR_IN2, 1);
    analogWrite(enB, right);
  }
  else if(right <= 0) {
    digitalWrite(RIGHT_MOTOR_IN1, 1); 
    digitalWrite(RIGHT_MOTOR_IN2, 0);
    analogWrite(enB, abs(right));
  }
    
  }
}
ros::Subscriber<std_msgs::Float32MultiArray> control_sub("robot_control", &robot_control);

//Khai báo node truyền giá trị khoảng cách bánh xe
std_msgs::Float32MultiArray distance_node;
ros::Publisher arduino_node("distance_node", &distance_node);

void odometry()
{
  // --- Đọc giá trị encoder hiện tại ---
  long current_pulse_l = pulse_l;
  long current_pulse_r = pulse_r;

  // --- Tính sự thay đổi số xung ---
  long delta_pulse_l = current_pulse_l - pulse_l_prev;
  long delta_pulse_r = current_pulse_r - pulse_r_prev;

  // --- Tính khoảng cách di chuyển của mỗi bánh xe ---
  float distance_l = (2 * M_PI * wheel_radius * delta_pulse_l) / encoder_resolution_left;
  float distance_r = (2 * M_PI * wheel_radius * delta_pulse_r) / encoder_resolution_right;

  distance_node.data_length = 2;
  distance_node.data = new float[distance_node.data_length]; // Cấp phát bộ nhớ cho mảng data
  distance_node.data[0] = distance_l;
  distance_node.data[1] = distance_r;

  arduino_node.publish(&distance_node);
  delete[] distance_node.data;
  
  // --- Tính sự thay đổi vị trí và hướng của robot (Differential Drive Kinematics) ---
  float delta_s = (distance_l + distance_r) / 2.0;
  float delta_theta = (distance_r - distance_l) / wheel_base;

  x += delta_s * cos(theta);
  y += delta_s * sin(theta);
  theta += delta_theta;

  // --- Giới hạn góc theta trong khoảng -PI đến PI (tùy chọn) ---
  if (theta > M_PI) theta -= 2 * M_PI;
  if (theta < -M_PI) theta += 2 * M_PI;

  // --- Publish thông tin vị trí và hướng lên ROS ---
  pose_msg.x = x;
  pose_msg.y = y;
  pose_msg.z = theta; // Sử dụng z để truyền hướng (yaw)
  pose_pub.publish(&pose_msg);

  // --- Cập nhật giá trị encoder trước ---
  pulse_l_prev = current_pulse_l;
  pulse_r_prev = current_pulse_r;

}

// --- Hàm xử lý ngắt Encoder  ---
void count_r()
{
  if (digitalRead(encoder1PinB) == LOW)
  {
     pulse_r ++;
  }
  else
  {
    pulse_r --;
  }
}

void count_l()
{
  if (digitalRead(encoder2PinA) == LOW)
  {    
    pulse_l --;
  }
  else
  {
    pulse_l ++;
  }
}

void setup() {
  nh.initNode();
  nh.advertise(pose_pub); // Đã sửa topic name sang odom_raw trong khai báo trên
  nh.subscribe(control_sub);  
  nh.advertise(arduino_node);
  // Cấu hình chân động cơ
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), count_r, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), count_l, RISING);

  Serial.begin(57600);
  Serial.println("Odometry Started");
}

void loop() {
  nh.spinOnce();
 
  odometry();

   delay(100);
}
