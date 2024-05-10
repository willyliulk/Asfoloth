/*
 * float16Tool.c
 *
 *  Created on: Oct 24, 2023
 *      Author: liu willy
 */

#include "IMU_rel/float16Tool.h"

void f32_to_f16(f32_t *fIn, f16_t *u){
    f32_t f32inf = { 255UL << 23 };
    f32_t f16inf = { 31UL << 23 };
    f32_t magic = { 15UL << 23 };
    const uint32_t sign_mask = 0x80000000U;
    const uint32_t round_mask = ~0xFFFU;

    f32_t in;
    in.f = fIn->f;
    uint32_t sign = in.u32 & sign_mask;
    in.u32 ^= sign;

    uint16_t out = 0;

    if (in.u32 >= f32inf.u32)
    {
        out = (in.u32 > f32inf.u32) ? (uint16_t)0x7FFFU : (uint16_t)0x7C00U;
    }
    else
    {
        in.u32 &= round_mask;
        in.f *= magic.f;
        in.u32 -= round_mask;
        if (in.u32 > f16inf.u32)
        {
            in.u32 = f16inf.u32;
        }
        out = (uint16_t)(in.u32 >> 13);
    }

    out |= (uint16_t)(sign >> 16);
    u->u16 = out;
}

float f16_to_f32(f16_t *u, f32_t *fOut){
    uint16_t value = u->u16;
    f32_t magic = { (254UL - 15UL) << 23 };
    f32_t was_inf_nan = { (127UL + 16UL) << 23 };
    f32_t out;

    fOut->u32 = (value & 0x7FFFU) << 13;
    fOut->f *= magic.f;
    if (fOut->f >= was_inf_nan.f)
    {
        fOut->u32 |= 255UL << 23;
    }
    fOut->u32 |= (value & 0x8000UL) << 16;
}
