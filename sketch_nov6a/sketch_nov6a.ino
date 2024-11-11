#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Simple_AHRS.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified  accel(30301);
Adafruit_LSM303DLH_Mag_Unified mag(30302);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

const int numReadings = 10;
int readingsH[numReadings];  // the readings from the analog input
int readingsR[numReadings];
int readingsP[numReadings];
int readIndex = 0;          // the index of the current reading
int totalr = 0;              // the running total
int totalp = 0;
int totalh = 0;
int averager = 0;            // the average
int averagep = 0;
int averageh = 0;


//m1 m3
//m2 m4



void setup()
{
  
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the sensors.
  accel.begin();
  mag.begin();
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsH[thisReading] = 0;
    readingsP[thisReading] = 0;
    readingsR[thisReading] = 0;
  }
  
}
int c = 0;
int holH = 0;
int holP = 0;
int holR = 0;
int mm1 = 0; int mm2 = 0; int mm3 = 0; int mm4 = 0;
int m1 = 90, m2 = 90, m3 = 90, m4 = 90;  
int fh, fp, fr;

void updateVal(sensors_vec_t orientation){
    holH = orientation.heading;
    holP = orientation.pitch;
    holR = orientation.roll;
    
    totalr = totalr - readingsR[readIndex];
    totalp = totalp - readingsP[readIndex];
    totalh = totalh - readingsH[readIndex];
    readingsH[readIndex] = holH;
    readingsP[readIndex] = holP;
    readingsR[readIndex] = holR;
    totalr = totalr + readingsR[readIndex];
    totalp = totalp + readingsP[readIndex];
    totalh = totalh + readingsH[readIndex];
}
void calibrate(void){
  int num = 100;
  int avgr = 0, avgh = 0, avgp = 0;
  int avgR[num] = {0};
  int avgH[num] = {0};
  int avgP[num] = {0};
  int aholH = 0;
  int aholP = 0;
  int aholR = 0;
  sensors_vec_t   orientation;
  for (int i = 0; i< 100; i++){
    avgr = avgr - avgR[i];
    avgh = avgh - avgH[i];
    avgp = avgp - avgP[i];
    sensors_vec_t   orientation;
    if(ahrs.getOrientation(&orientation)){
      avgH[i] = orientation.heading;
      avgP[i] = orientation.pitch;
      avgR[i] = orientation.roll;
    }
    else{
      i -= 1;
      continue;
    }
    avgr = avgr + avgR[i];
    avgh = avgh + avgH[i];
    avgp = avgp + avgP[i];
  }
  fr = avgr / 100;
  fp = avgp / 100;
  fh = avgh / 100;
  Serial.print(fr); Serial.print(" "); Serial.print(fh); Serial.print(" "); Serial.println(fp);
  delay(10);
}

void loop(void)
{
  m1 = 90;
  m2 = 90;
  m3 = 90; 
  m4 = 90;
  mm1 = 0; 
  mm2 = 0; 
  mm3 = 0;
  mm4 = 0;

  sensors_vec_t   orientation;
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    if (c == 0){
      calibrate();
      updateVal(orientation);
      Serial.println(readingsR[readIndex]);
    }
    else{
      updateVal(orientation);
      // int OOH = (abs(holH - totalh) > 50) ? 1:0;
      // int OOR = (abs(holR - totalr) > 100) ? 1:0;
      // int OOP = (abs(holP - totalp) > 50) ? 1:0;
      // // if (OOH || OOR || OOH){
      // //     Serial.print("OOR: "); Serial.println(totalr);Serial.print("OOR2: "); Serial.println(totalp);Serial.print("OOR3: "); Serial.println(totalh);
      // //     //Serial.print("OOR: "); Serial.println(holR - readingsR[readIndex]);Serial.print("OOR2: "); Serial.println(OOH);Serial.print("OOR3: "); Serial.println(OOP);
      // // }
        readIndex = readIndex + 1;
        if (readIndex >= numReadings) {
          readIndex = 0;
        }
        averager = totalr / numReadings;
        averagep = totalp / numReadings;
        averageh = totalh / numReadings;
        //Serial.print("Roll: "),Serial.println(averager);//Serial.print(" ");Serial.print("Pitch: "),Serial.print(averagep);Serial.print(" ");Serial.print("Heading: "),Serial.println(averageh);
        mm1 = averager > 0 ? (m1 - map(abs(averager), 0, abs(fr), 0, m1)) : -(m1 - map(abs(averager), 0, abs(fr), 0, m1));
        mm2 = averagep > 0 ? -(m1 - map(abs(averagep),35, fp, 0, m1)) : +(m1 - map(abs(averagep),35, fp, 0, m1));
        Serial.println(averager); Serial.println(mm1);Serial.println(averagep);Serial.println(mm2);
        m1 += mm1;
        m2 += mm1;
        m3 -= mm1;
        m4 -= mm1;

        m1 += mm2;
        m3 -= mm2;
        m2 += mm2;
        m4 -= mm2;
        Serial.print("M1: "); Serial.print(m1); Serial.print(" M2: "); Serial.print(m2); Serial.print(" M3: "); Serial.print(m3); Serial.print(" M4: "); Serial.println(m4); 
     }
    c=1;
  }
  
  delay(10);
}
