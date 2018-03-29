/*
  Line Follower Robot Code Using PID Algorithm
  Author: Sayed Mohammed Tasmimul Huda
  Khulna Univarsity of Engineering and Technology
  ECE: 2K16
 
 */
#include<SoftwareSerial.h>
SoftwareSerial mySerial(10,11); //pin of arduino
//motor control pin
#define LM1 2
#define LM2 3
#define RM1 6
#define RM2 7
#define E1 A0  //pmw pin for left motor
#define E2 A1   //pmw pin for ri
#define ON 1
#define OFF 0
#define fullWhiteSurface ON

int s1, s2, s3, s4, s5, s6, s7; //
#define max_speed 135
#define TS 120,
#define base_speed 150
int set_point;
int currentpoint;
int last_proportional;
double Kp, Kd, Ki;
int PID_output;
float sum;
int right_motor_speed;
int left_motor_speed;
int proportional, integral, derivative;
long int ir_avg;
long int ir_sum;
int sensors[7] = {0, 0, 0, 0, 0, 0, 0};
int s[7] = {1,2,3.5,4,4.5,6,7};
void setup()
{
  Serial.begin(9600);
  mySerial.begin(4800);   //default communication speed of the sensor
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  delay(1000);
}

void read_ir()
{
  mySerial.write('1');
  s1 = mySerial.read();
  delay(8);
  mySerial.write('2');
  s2 = mySerial.read();
  delay(8);
  mySerial.write('3');
  s3 = mySerial.read();
  delay(8);
  mySerial.write('4');
  s4 = mySerial.read();
  delay(8);
  mySerial.write('5');
  s5 = mySerial.read();
  delay(8);
  mySerial.write('6');
  s6 = mySerial.read();
  delay(8);
  mySerial.write('7');
  s7 = mySerial.read();
  Serial.print ("  ");    ///*change this to 1,2,3,4,5,6 or 7 to get individual sensor reading*/
  delay(8);
  sum = s1 + s2 + s3 + s4 + s5 + s6 + s7;
  Serial.print ("  ");
  Serial.print(s2);  // s1
  Serial.print ("  ");
  Serial.print(s3);  // s2
  Serial.print ("  ");
  Serial.print(s4);  // s3
  Serial.print ("  ");
  Serial.print(s5);  // s4
  Serial.print ("  ");
  Serial.print(s6);  // s5
  Serial.print ("  ");
  Serial.print(s7);  // s6
  Serial.print(" ");
  Serial.print(s1);  // s7
  Serial.println("  ");
  //Serial.print("sum:");
  //Serial.print(sum);
  // Serial.println(" ");
}
void calc_ir()
{
  long int sensors[] = {s2, s3, s4, s5, s6, s7, s1};
  Serial.print("sensor value:");
  for (int i = 0; i < 7; i++)
  {
    Serial.print(sensors[i]); Serial.print(" ");
  }
  Serial.println("");
  ir_avg = 0;
  ir_sum = 0;
  for (int i = 0; i < 7; i++)
  {
    ir_avg += int(sensors[i]) * int(s[i]); //
    //ir_sum +=int(sensors[i]);
    delay(8);
    Serial.print(" ");Serial.print(ir_avg);//Serial.print(" ");Serial.print(ir_sum);Serial.println(" ");
  }


  for(int i=1; i<6; i++)
  {
    ir_avg+=sensors[i]*(i+10)*10;
    ir_sum+=sensors[i];
    delay(8);
    Serial.print(ir_avg);
    Serial.print(" ");
   // Serial.println();
    //Serial.print(ir_sum);
   // Serial.print(" ");
  }
 Serial.println();
  Serial.print("ir_avg:");
    Serial.print(ir_avg);
    Serial.println();
    Serial.print("ir_sum:");
    Serial.print(ir_sum);
  Serial.println(" ");
  Serial.print("sp:");
  Serial.print(ir_avg/ir_sum);
  Serial.println(" ");
  //Serial.print("sensor sum:");Serial.print(ir_sum);Serial.println(" ");
  //delay(40);
}

void pid_calc()
{
  Kp = 20;//Ki=0.1;Kd=50;
  set_point =135;
  //currentpoint = int(ir_avg/50);
  currentpoint = int(ir_avg/ir_sum);
  proportional = currentpoint - set_point;
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  PID_output = double(proportional * Kp + integral * Ki + derivative * Kd);
  last_proportional = proportional;

  Serial.print(" ");/* Serial.print("ir_avg:");Serial.print(ir_avg);Serial.println(" ");Serial.print("sensor sum:");Serial.print(ir_sum);*/
  Serial.println(" "); Serial.print("setpoint:"); Serial.print(set_point); Serial.println(" "); Serial.print("currentpoint:"); Serial.print(currentpoint); Serial.println(" "); Serial.print("proportional:"); Serial.print(proportional); Serial.println(" ");
  //Serial.print("derivative:");Serial.print(derivative);Serial.println(" ");Serial.print("integral:");Serial.print(integral);Serial.println(" ");
  Serial.print("pid output:"); Serial.print(PID_output); Serial.println(" ");
    
}

void calc_turn()
{
  //Restricting the error value between +256.
  if (PID_output < -256)
  {
    PID_output = -256;
  }
  if (PID_output > 256)
  {
    PID_output =   256;
  }
  // If PID_output is less than zero calculate right turn speed values
  if (PID_output < 0)
  {
    right_motor_speed = max_speed ;
    left_motor_speed = max_speed+ PID_output;
  }
  // If error_value is greater than zero calculate left turn values
  else
  {
    right_motor_speed = max_speed- PID_output;


    left_motor_speed = max_speed ;
  }
}
void small_forward()
{

}

void left_turn()
{
}

void right_turn()
{

}

void motor_drive(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  analogWrite(E1, right_motor_speed);
  analogWrite(E2, left_motor_speed);
  Serial.println(" ");
  Serial.print("right motor speed:"); 
  Serial.print(right_motor_speed); 
  Serial.println(" ");
  Serial.print("left motor speed:"); 
  Serial.print(left_motor_speed); 
  Serial.println(" ");
}
void loop()
{
  //delay(4000);
  read_ir();
  calc_ir();
  pid_calc();
  calc_turn();
  right_turn();
  left_turn();
  //if (ir_avg <= 980 || ir_avg >=6800)
    //motor_drive(0, 0);
  //else
    motor_drive(right_motor_speed,left_motor_speed);
//delay(5000);
//delay(5000);
}

