#ifndef IMU_CAL_H
#define IMU_CAL_H

// Bench calibration with the drone still on a level surface (motors never armed): every 5 s,
// print the IMU's average roll/pitch over USB. Copy them into IMU_ROLL_OFFSET_DEG /
// IMU_PITCH_OFFSET_DEG in attitude.c. Call after flight_controller_init(). Never returns.
__attribute__((noreturn)) void imu_cal_run(void);

#endif // IMU_CAL_H
