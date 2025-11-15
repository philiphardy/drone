#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#include <stdint.h>
#include <stdio.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_3d;

inline void printv(char * n, vector_3d * v) {
    printf("%s: (%d, %d, %d)\n", n, v->x, v->y, v->z);
}

#endif
