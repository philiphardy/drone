#ifndef FAILSAFE_H
#define FAILSAFE_H

// Stop all motors, report the reason, and halt forever.
__attribute__((noreturn)) void failsafe(const char *reason);

#endif // FAILSAFE_H
