#include <Wire.h>
#include "extra.h"
int ACC_Data0, ACC_Data1, ACC_Data2, ACC_Data3, ACC_Data4, ACC_Data5;

int MAG_Data0, MAG_Data1, MAG_Data2, MAG_Data3, MAG_Data4, MAG_Data5;

int Ax, Ay, Az, Mx, My, Mz;

float Xm_print, Ym_print, Zm_print;
float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, Norm_m;

float Xa_print, Ya_print, Za_print;
float Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Norm_a;

const float alpha = 0.15;
float fXa = 0;
float fYa = 0;
float fZa = 0;
float fXm = 0;
float fYm = 0;
float fZm = 0;
float pitch, pitch_print, roll, roll_print, Heading;
float fXm_comp, fYm_comp;

float fh = 0, fp = 0, fr = 0;
float rH = 0, rR = 0, rP = 0;

#define SIGN(x) (x>0 ? 1: -1)
void setupIMU(){
  Wire.begin();
  Serial.begin(9600);
  
  Wire.beginTransmission(0x19); // Set accel 
  Wire.write(0x20);             // CTRL_REG1_A register
  Wire.write(0x47);             // 50 Hz, normal power, all 3 axis enabled
  Wire.endTransmission();

  Wire.beginTransmission(0x19); // Set accel
  Wire.write(0x23);             // CTRL_REG4_A register
  Wire.write(0x08);             // continuous update, littleendian, 2g, high resolution, 4-wire spi
  Wire.endTransmission();

  Wire.beginTransmission(0x1E); // set Mag
  Wire.write(0x00);             // CRA_REG_M register
  Wire.write(0x14);             // 30 Hz min data output rate
  Wire.endTransmission();

  Wire.beginTransmission(0x1E); // Set Mag
  Wire.write(0x02);             // MR_REG_M
  Wire.write(0x00);             // continuous conversion mode
  Wire.endTransmission();
}
void IMU(float IMUv[], int size){
  int num = 10;
  float avgr = 0, avgh = 0, avgp = 0;
  float avgR[num] = {0};
  float avgH[num] = {0};
  float avgP[num] = {0};
  float aholH = 0;
  float aholP = 0;
  float aholR = 0;

  for (int i = 0; i< num; i++){
    avgr = avgr - avgR[i];
    avgh = avgh - avgH[i];
    avgp = avgp - avgP[i];

    Wire.beginTransmission(0x19);
    Wire.write(0x28);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);
    ACC_Data0 = Wire.read();
    
    Wire.beginTransmission(0x19);
    Wire.write(0x29);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);
    ACC_Data1 = Wire.read();

    Wire.beginTransmission(0x19);
    Wire.write(0x2A);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);
    ACC_Data2 = Wire.read();
    
    Wire.beginTransmission(0x19);
    Wire.write(0x2B);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);
    ACC_Data3 = Wire.read();

    Wire.beginTransmission(0x19);
    Wire.write(0x2C);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);  
    ACC_Data4 = Wire.read();
    
    Wire.beginTransmission(0x19);
    Wire.write(0x2D);
    Wire.endTransmission();
    Wire.requestFrom(0x19, (byte)1);
    ACC_Data5 = Wire.read();

    Ax = (int16_t)(ACC_Data1 << 8) | ACC_Data0;
    Ay = (int16_t)(ACC_Data3 << 8) | ACC_Data2;
    Az = (int16_t)(ACC_Data5 << 8) | ACC_Data4;

    Xa_off = Ax/16.0 + 14.510699; // add/subtract bias calculated by Magneto 1.2. Search the web and you will
    Ya_off = Ay/16.0 - 17.648453; // find this Windows application.  It works very well to find calibrations
    Za_off = Az/16.0 -  6.134981;
    Xa_cal =  1.006480*Xa_off - 0.012172*Ya_off + 0.002273*Za_off; // apply scale factors calculated by Magneto1.2
    Ya_cal = -0.012172*Xa_off + 0.963586*Ya_off - 0.006436*Za_off;
    Za_cal =  0.002273*Xa_off - 0.006436*Ya_off + 0.965482*Za_off;
    Norm_a = sqrt(Xa_cal * Xa_cal + Ya_cal * Ya_cal + Za_cal * Za_cal); //original code did not appear to normalize, and this seems to help
    Xa_cal = Xa_cal / Norm_a;
    Ya_cal = Ya_cal / Norm_a;
    Za_cal = Za_cal / Norm_a;

    Ya_cal = -1.0 * Ya_cal;  // This sign inversion is needed because the chip has +Z up, while algorithms assume +Z down
    Za_cal = -1.0 * Za_cal;  // This sign inversion is needed for the same reason and to preserve right hand rotation system

  // Low-Pass filter accelerometer
    fXa = Xa_cal * alpha + (fXa * (1.0 - alpha));
    fYa = Ya_cal * alpha + (fYa * (1.0 - alpha));
    fZa = Za_cal * alpha + (fZa * (1.0 - alpha));

    Wire.beginTransmission(0x1E);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(0x1E, (byte)6);
    MAG_Data0 = Wire.read();
    MAG_Data1 = Wire.read();
    MAG_Data2 = Wire.read();
    MAG_Data3 = Wire.read();
    MAG_Data4 = Wire.read();
    MAG_Data5 = Wire.read();

    Mx = (int16_t)(MAG_Data0 << 8) | MAG_Data1;
    Mz = (int16_t)(MAG_Data2 << 8) | MAG_Data3;
    My = (int16_t)(MAG_Data4 << 8) | MAG_Data5;

    Xm_off = Mx*(100000.0/1100.0) -   617.106577; // Gain X [LSB/Gauss] for selected sensor input field range (1.3 in these case)
    Ym_off = My*(100000.0/1100.0) -  3724.617984; // Gain Y [LSB/Gauss] for selected sensor input field range
    Zm_off = Mz*(100000.0/980.0 ) - 16432.772031;  // Gain Z [LSB/Gauss] for selected sensor input field range
    Xm_cal =  0.982945*Xm_off + 0.012083*Ym_off + 0.014055*Zm_off; // same calibration program used for mag as accel.
    Ym_cal =  0.012083*Xm_off + 0.964757*Ym_off - 0.001436*Zm_off;
    Zm_cal =  0.014055*Xm_off - 0.001436*Ym_off + 0.952889*Zm_off;
    Norm_m = sqrt(Xm_cal * Xm_cal + Ym_cal * Ym_cal + Zm_cal * Zm_cal); // original code did not appear to normalize  This seems to help
    Xm_cal = Xm_cal / Norm_m;
    Ym_cal = Ym_cal / Norm_m;
    Zm_cal = Zm_cal / Norm_m;
    
    Ym_cal = -1.0 * Ym_cal;  // This sign inversion is needed because the chip has +Z up, while algorithms assume +Z down
    Zm_cal = -1.0 * Zm_cal;  // This sign inversion is needed for the same reason and to preserve right hand rotation system

  // Low-Pass filter magnetometer
    fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
    fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
    fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));


  // Pitch and roll
    pitch = asin(fXa); 
    roll = -asin(fYa);
    pitch_print = pitch*180.0/M_PI;
    roll_print = roll*180.0/M_PI;

  // Tilt compensated magnetic sensor measurements
    fXm_comp = fXm*cos(pitch) + fZm*sin(pitch); 
    fYm_comp = fYm*cos(roll) - fZm*sin(roll);

    Heading = (atan2(-fYm_comp,fXm_comp)*180.0)/M_PI; 

    if (Heading < 0)
    Heading += 360;

    avgH[i] = Heading;
    avgP[i] = pitch_print - fp;
    avgR[i] = roll_print - fr;

    avgr = avgr + avgR[i];
    avgh = avgh + avgH[i];
    avgp = avgp + avgP[i];
    delay(1);
  }
  aholR = avgr / num;
  aholP = avgp / num;
  aholH = avgh / num;
  //Serial.print(fr); Serial.print(" "); Serial.print(fh); Serial.print(" "); Serial.println(fp);


  IMUv[0] = aholP;
  IMUv[1] = aholR;
  IMUv[2] = aholH;

  // Serial.print("Pitch (X): "); Serial.print(IMUv[0]); Serial.print("  ");
  // Serial.print("Roll (Y): "); Serial.print(IMUv[1]); Serial.print("  ");
  // Serial.print("Heading: "); Serial.println(IMUv[2]);
}

void calibrate(){
  float IMUo[3];
  int num = 50;
  float avgr = 0, avgh = 0, avgp = 0;
  float avgR[num] = {0};
  float avgH[num] = {0};
  float avgP[num] = {0};
  float aholH = 0;
  float aholP = 0;
  float aholR = 0;
  for (int i = 0; i< num; i++){
    avgr = avgr - avgR[i];
    avgh = avgh - avgH[i];
    avgp = avgp - avgP[i];
    IMU(IMUo, 3);
    avgH[i] = IMUo[2];
    avgP[i] = IMUo[0];
    avgR[i] = IMUo[1];
    avgr = avgr + avgR[i];
    avgh = avgh + avgH[i];
    avgp = avgp + avgP[i];
    // Serial.print(IMUo[1]); Serial.print(" "); Serial.print(IMUo[2]); Serial.print(" "); Serial.println(IMUo[0]);
    delay(1);
  }
  fr = avgr / num;
  fp = avgp / num;
  fh = avgh / num;
  Serial.print(fr); Serial.print(" "); Serial.print(fh); Serial.print(" "); Serial.println(fp);
  IMU(IMUo, 3);
  rR = IMUo[1]; rP = IMUo[0]; rH = IMUo[2];
}

int cap(int x){
  if (x> 180){
    return 180;
  }
  if (x<0){
    return 0;
  }
  else{
    return x;
  }
}

void updateMotors(int m[], float IMUv[], uint8_t thr){
  int mini = 180;
  for (int i = 0; i<4; i++){
    if (mini>m[i]){
      mini = m[i];
    }
  }
  // int mr = IMUv[1] > 0 ? (mini - map(mini, abs(IMUv[1]), abs(fr), 0, mini)) : -(mini - map(mini, abs(IMUv[1]), abs(fr), 0, mini));
  // int mp = IMUv[0] > 0 ? -(mini - map(abs(IMUv[0]),35, fp, 0, mini)) : +(mini - map(abs(IMUv[0]),35, fp, 0, mini));
  int mr = SIGN(IMUv[1]) * map(mr, (abs(IMUv[1]) + rR), 75, 0, mini);
  int mp = SIGN(IMUv[0]) * map(mp, (abs(IMUv[0]) + rP), 75, 0, mini);
  m[0] += mr;
  m[1] -= mr;
  m[2] += mr;
  m[3] -= mr;

  m[0] += mp;
  m[1] += mp;
  m[2] -= mp;
  m[3] -= mp;

  m[0] = cap(m[0]);
  m[1] = cap(m[1]);
  m[2] = cap(m[2]);
  m[3] = cap(m[3]);
  //Serial.print(IMUv[1]); Serial.print(" "); Serial.print(IMUv[2]); Serial.print(" "); Serial.println(IMUv[0]);
  //Serial.print((IMUv[1])); Serial.print(":"); Serial.println(mr); Serial.print((IMUv[0])); Serial.print(":");Serial.println(mp);
  Serial.print(m[0]); Serial.print(":"); Serial.print(m[1]); Serial.print(":");Serial.print(m[2]); Serial.print(":"); Serial.println(m[3]);
}