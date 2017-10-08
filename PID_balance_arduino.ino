
#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
   to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;


float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
/////////////////PID CONSTANTS/////////////////
double kp = 3.55; //3.55
double ki = 0.006; //0.003
double kd = 0.001; //2.05
///////////////////////////////////////////////

double throttle = 1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
//balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
  right_prop.attach(10); //attatch the right motor to pin 3
  left_prop.attach(3);  //attatch the left motor to pin 5

  time = millis(); //Start counting time in milliseconds
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  delay(7000);
}//end of setup void

void loop() {
  throttle = map(pulseIn(A1, HIGH, 25000), 1000, 1900, 1100, 2000);
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  Acc_rawX = Wire.read() << 8 | Wire.read(); //each value needs two registres
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true); //Just 4 registers

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
    the raw value by 131 because that's the value that the datasheet gives us*/

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX / 131.0;
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY / 131.0;

  /*---X axis angle---*/
  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];

  /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
  //Serial.println(Total_angle[1]);



  /*///////////////////////////P I D///////////////////////////////////*/
  /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/

  /*First calculate the error between the desired angle and
    the real measured angle*/
  error = Total_angle[1] - desired_angle;

  /*Next the proportional value of the PID is just a proportional constant
    multiplied by the error*/

  pid_p = kp * error;

  if (-3 < error < 3)
  {
    pid_i = pid_i + (ki * error);
  }

  pid_d = kd * ((error - previous_error) / elapsedTime);

  /*The final PID values is the sum of each of this 3 parts*/
  PID = pid_p + pid_i + pid_d;

  if (PID < -1000)
  {
    PID = -1000;
  }
  if (PID > 1000)
  {
    PID = 1000;
  }

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;


  /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/
  //Right
  if (pwmRight < 1000)
  {
    pwmRight = 1000;
  }
  if (pwmRight > 2000)
  {
    pwmRight = 2000;
  }
  //Left
  if (pwmLeft < 1000)
  {
    pwmLeft = 1000;
  }
  if (pwmLeft > 2000)
  {
    pwmLeft = 2000;
  }

  Serial.print(pwmLeft);
  Serial.print("  ");
  Serial.println(pwmRight);
  
  /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
  
    
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
  previous_error = error; //Remember to store the previous error.
  delay(40);
}//end of loop void
