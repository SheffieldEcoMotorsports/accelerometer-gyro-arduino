#include <Wire.h>
//Comment
/* 
 *  ROLL=Y, PITCH=Z, YAW=X 
 *  To use MPU6050 in another orientation correct x/y/z usage in calcAngle and callibrate functions
 *  HEAVE=X, SURGE=Y, SWAY=Z
*/

#define CAL_REP 2000 //number of readings to take for callibration
#define A_K 4096  //scaling constant for accelerometer
#define LOOP_PERIOD 250 //period of loop() in micro seconds
//#define G_K 0.000061068  //scaling constant for gyro inc 4ms time delay (4e-3/65.5)
//#define G_K_rad 0.0000010658  //gyro scaling constant (G_K) including conversion to radians see calcAngle()
//#define G_K 0.000038167 //2.5ms ^^^^^^^^
//#define G_K_rad 0.00000066616 //2.5ms ^^^^^^^^
#define G_K 0.00004 //rough guess to "trim" & improve accuracy ^^^^^^^^
#define G_K_rad 0.00000069813 //^^^^^^^^
#define G_PROP 0.9996 //proportion of gyro data in complementary filter
#define A_PROP 0.0004 //proportion of acc data in comp. filter. NB: A_PROP + G_PROP = 1

long gx, gy, gz;  //raw gyro data
double ax, ay, az;  //raw acc data
long gx_cal, gy_cal, gz_cal, ax_cal, ay_cal, az_cal;  //calibration variables
float R, P; //roll/pitch angles calculated from gyro integration and then corrected by accelerometer
float aR, aP; //roll/pitch angles calcualted from accelerometer
unsigned long old_t = 0;  //used to delay each loop to 4ms
float S, W; //surge/sWay after correction for angle/gravity

void calibrate(){ //get gyro offsets, and an average acc value to set initial roll/pitch values
  int n;
  long gx_sum=0, gy_sum=0, gz_sum=0, ax_sum=0, ay_sum=0, az_sum=0;
  for (n=0;n<CAL_REP;n++){
      getData(0); //passing zero returns raw data
      gx_sum+=gx;
      gy_sum+=gy;
      gz_sum+=gz;
      ax_sum+=ax;
      ay_sum+=ay;
      az_sum+=az;
  }
  gx_cal=gx_sum/CAL_REP;
  gy_cal=gy_sum/CAL_REP;
  gz_cal=gz_sum/CAL_REP;
  ax_cal=ax_sum/CAL_REP;
  ay_cal=ay_sum/CAL_REP;
  az_cal=az_sum/CAL_REP;
}

void getData(char calibrated){  //if calibrated=0 then raw data returned, if =1 the calibration offsets are subtracted and calibrated data returned
  Wire.beginTransmission(0x68);  //start coms with MPU
  Wire.write(0x3B); //address of register to be read next...
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);  //request 14 bytes from gyro
  while(Wire.available() < 14){} //wait untill all bytes recieved to input buffer
  //data split into two bytes, they are combined below...
  ax = Wire.read()<<8|Wire.read();                           
  ay = Wire.read()<<8|Wire.read();                           
  az = Wire.read()<<8|Wire.read();                                
  Wire.read();Wire.read();   //ignore temperature data                        
  gx = Wire.read()<<8|Wire.read();                                 
  gy = Wire.read()<<8|Wire.read();                               
  gz = Wire.read()<<8|Wire.read(); 
  if (calibrated){  //subtract gyro offsets if required
    gx-=gx_cal;
    gy-=gy_cal;
    gz-=gz_cal; 
  }
}

// NB: ROLL=Y, PITCH=Z, YAW=X
// NB: HEAVE=X, SURGE=Y, SWAY=Z
void calcAngle(){
  float aTotal;
  //bellow integrates gyro values to find absolute position
  R+=gy*G_K;  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  P+=gz*G_K;
  //bellow corrects roll/pitch from gimbal lock using yaw value 
  R+=P*sin(gx*G_K_rad);  //_rad is same as G_K*pi/180 (convert to rads)
  P-=R*sin(gx*G_K_rad); 
  aTotal = sqrt((ax*ax)+(ay*ay)+(az*az));  //Calculate the total accelerometer vector
  //bellow calculates angle from acc data
  aR=asin((float)az/aTotal)*57.296; //convert to degrees 
  aP=asin((float)ay/aTotal)*-57.296;
  R=R*G_PROP+aR*A_PROP;  //complementary filter for roll
  P=P*G_PROP+aP*A_PROP;  //complementary filter for pitch

  //bellow corrects surge/sway subtracting gravity
  S=(float)ay+(float)aTotal*sin(P*0.01745);
  W=(float)az-(float)aTotal*sin(R*0.01745);
  S/=A_K; //convert to "g"
  W/=A_K;  
}

void setup() {
  float aTotal;

  Serial.println("Comencing setup...");
  
  Wire.begin(); //start I2C as master

  //configure gyro
  Wire.beginTransmission(0x68);  //start coms with MPU
  Wire.write(0x6B); //address of register to be written next...
  Wire.write(0); //initiate MPU
  Wire.endTransmission();

  //configure gyro (500deg/s)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); 
  Wire.write(B00001000); //500deg/s
  Wire.endTransmission();  

  //configure accelerometer (+/- 4g)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); 
  Wire.write(B00010000);
  Wire.endTransmission();  

  Serial.begin(115200);

  calibrate();  //get gyro offset values
  
  aTotal = sqrt((ax*ax)+(ay*ay)+(az*az));  //Calculate the total accelerometer vector then set initial pitch/roll values based on acc data
  R=asin((float)az/aTotal)*57.296;////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  P=asin((float)ay/aTotal)*-57.296;
  
  Serial.println("Setup Complete!");
}

/*
void loop(){  //read register debug
  Wire.beginTransmission(0x68);  //start coms with MPU
  Wire.write(25); //address of register to be read next...
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);  //request 14 bytes from gyro
  while(Wire.available() < 1){} //wait untill all bytes recieved to input buffer
  //data split into two bytes, they are combined below...
  Serial.println(Wire.read());                           
  while(true){}
}
*/

void loop(){
  int n, dt;
  for (n=0;n<50;n++){
    getData(1);
    calcAngle();
    do {  
      dt = micros()-old_t; //delay routine to ensure each loop is 2.5 ms long.
    } while (dt<LOOP_PERIOD);
    
//    Serial.println(micros()-old_t); /* Comment out do-while loop above and uncomment this line to print loop period */

    old_t=micros();  
  }
  Serial.print("Roll = "); Serial.print(R);
  Serial.print("    Pitch = "); Serial.print(P);
  Serial.print("    Surge = "); Serial.print(S);
  Serial.print("    Sway = "); Serial.println(W);
}

