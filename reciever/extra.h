#ifndef EXTRA_H
#define EXTRA_H

extern void IMU(float IMUv[], int size);
extern void calibrate();
extern void updateMotors(int m[], float IMUv[], uint8_t thr);
#endif