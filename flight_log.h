#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include "attitude.h"

// Append one sample to the RAM buffer. Ignored once the buffer is full.
void flight_log_record(float throttle, const attitude_t *att, float u_roll, float u_pitch, float u_yaw, float vz);

// Print the recorded samples as CSV over stdio.
void flight_log_dump(void);

#endif // FLIGHT_LOG_H
