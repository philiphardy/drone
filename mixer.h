#ifndef MIXER_H
#define MIXER_H

// Lowest command while armed so motors never stall.
#define MIXER_IDLE_THROTTLE_PERCENT 5.0f

// + configuration mixer. throttle is the collective command (percent);
// u_roll/u_pitch/u_yaw are PID corrections in throttle percent. Negative
// u_roll/u_pitch means "tilted positive, push back". Each motor output is
// clamped to [MIXER_IDLE_THROTTLE_PERCENT, 100].
void mixer_output(float throttle, float u_roll, float u_pitch, float u_yaw);

#endif // MIXER_H
