/*
 * float16Tool.h
 *
 *  Created on: Oct 13, 2023
 *      Author: liu willy
 */

#ifndef FLOAT16_TOOL_H
#define FLOAT16_TOOL_H

#include <stdint.h>

typedef union f32
{
    uint32_t u32;
    uint8_t u8[4];
    float f;
} f32_t;

typedef union f16{
    uint16_t u16;
    uint8_t u8[2];
} f16_t;

void f32_to_f16(f32_t *fIn, f16_t *u);

float f16_to_f32(f16_t *u, f32_t *fOut);

#endif
