/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 BambooMaster
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "i2s_uac.h"
#include "pico/stdlib.h"

#include "i2s_core.h"

static int32_t mul_l, mul_r;

// -100dB ~ 0dB (1dB step)
static const int32_t db_to_vol[101] = {
	0x20000000,     0x1c8520af,     0x196b230b,     0x16a77dea,     0x1430cd74,     0x11feb33c,     0x1009b9cf,     0xe4b3b63,      0xcbd4b3f,      0xb5aa19b,
    0xa1e89b1,      0x904d1bd,      0x809bcc3,      0x729f5d9,      0x66284d5,      0x5b0c438,      0x5125831,      0x4852697,      0x4074fcb,      0x3972853,
    0x3333333,      0x2da1cde,      0x28ab6b4,      0x243f2fd,      0x204e158,      0x1ccab86,      0x19a9294,      0x16dec56,      0x146211f,      0x122a9c2,
    0x1030dc4,      0xe6e1c6,       0xcdc613,       0xb76562,       0xa373ae,       0x91ad38,       0x81d59e,       0x73b70f,       0x672194,       0x5bea6e,
    0x51eb85,       0x4902e3,       0x411245,       0x39feb2,       0x33b022,       0x2e1127,       0x290ea8,       0x2497a2,       0x209ce9,       0x1d10f9,
    0x19e7c6,       0x171693,       0x1493ce,       0x1256f0,       0x10585e,       0xe9152,        0xcfbc3,        0xb924e,        0xa5028,        0x9310b,
    0x83126,        0x74d16,        0x681d3,        0x5ccab,        0x52b36,        0x49b50,        0x41b10,        0x3a8c3,        0x342e4,        0x2e818,
    0x2972d,        0x24f0e,        0x20ec7,        0x1d57e,        0x1a26f,        0x174ee,        0x14c60,        0x1283b,        0x10804,        0xeb4d,
    0xd1b7,         0xbae8,         0xa695,         0x9477,         0x8452,         0x75ee,         0x691b,         0x5dad,         0x537d,         0x4a68,
    0x4251,         0x3b1b,         0x34ad,         0x2ef3,         0x29d7,         0x254b,         0x213c,         0x1d9f,         0x1a66,         0x1787,
    0x14f8,
};

int i2s_unpack_uacdata(uint8_t* in, int sample, uint8_t resolution, int32_t *buf_l, int32_t *buf_r){
    if (resolution == 16){
        int16_t *d = (int16_t*)in;
        sample /= 2;
        for (int i = 0; i < sample / 2; i++){
            buf_l[i] = *d++ << 16;
            buf_r[i] = *d++ << 16;
        }
    }
    else if (resolution == 24){
        uint8_t *d = in;
        int32_t e;
        sample /= 3;
        for (int i = 0; i < sample / 2; i++){
            e = 0;
            e |= *d++ << 8;
            e |= *d++ << 16;
            e |= *d++ << 24;
            buf_l[i] = e;
            e = 0;
            e |= *d++ << 8;
            e |= *d++ << 16;
            e |= *d++ << 24;
            buf_r[i] = e;
        }
    }
    else if (resolution == 32){
        int32_t *d = (int32_t*)in;
        sample /= 4;
        for (int i = 0; i < sample / 2; i++){
            buf_l[i] = *d++;
            buf_r[i] = *d++;
        }
    }

    return sample / 2;
}

void i2s_volume_change(int16_t v, int8_t ch){
    v = -v >> 8;
    if (v > 100) v = 100;
    else if (v < 0) v = 0;

    if (ch == 0){
        mul_l = db_to_vol[v];
        mul_r = db_to_vol[v];
    }
    else if (ch == 1){
        mul_l = db_to_vol[v];
    }
    else if (ch == 2){
        mul_r = db_to_vol[v];
    }
}

void i2s_volume(int32_t *buf_l, int32_t *buf_r, int length){
    for (int i = 0; i < length; i++){
        buf_l[i] = (int32_t)(((int64_t)buf_l[i] * mul_l) >> 29u);
        buf_r[i] = (int32_t)(((int64_t)buf_r[i] * mul_r) >> 29u);
    }
}