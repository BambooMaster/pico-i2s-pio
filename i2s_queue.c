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

#include "i2s_queue.h"
#include <stdatomic.h>
#include "pico/stdlib.h"

#include "i2s_core.h"
#include "i2s_queue.h"

static atomic_int queue_write = 0;
static atomic_int queue_read = 0;
static volatile int32_t queue_l[I2S_QUEUE_MAX];
static volatile int32_t queue_r[I2S_QUEUE_MAX];

bool i2s_enqueue(int32_t *buf_l, int32_t *buf_r, int length){
    if ((I2S_QUEUE_MAX - 1 - i2s_get_queue_length()) < length) return false;

    int w = atomic_load(&queue_write);

    int chunk1, chunk2;
    if (w + length >= I2S_QUEUE_MAX){
        chunk1 = I2S_QUEUE_MAX - w;
        chunk2 = length - chunk1;
    }
    else{
        chunk1 = length;
        chunk2 = 0;
    }

    for (int i = 0; i < chunk1; i++){
        queue_l[w + i] = buf_l[i];
        queue_r[w + i] = buf_r[i];
    }
    for (int i = 0; i < chunk2; i++){
        queue_l[i] = buf_l[chunk1 + i];
        queue_r[i] = buf_r[chunk1 + i];
    }

    w += length;
    if (w >= I2S_QUEUE_MAX) w -= I2S_QUEUE_MAX;
    atomic_thread_fence(memory_order_release);
    atomic_store(&queue_write, w);
    return true;
}

int i2s_dequeue(int32_t *buf_l, int32_t *buf_r, int length){
    int read_length = i2s_get_queue_length();
    if (read_length <= 0) return 0;

    if (read_length > length) read_length = length;
    int r = atomic_load(&queue_read);

    int chunk1, chunk2;
    if (r + read_length >= I2S_QUEUE_MAX){
        chunk1 = I2S_QUEUE_MAX - r;
        chunk2 = read_length - chunk1;
    }
    else{
        chunk1 = read_length;
        chunk2 = 0;
    }

    for (int i = 0; i < chunk1; i++){
        buf_l[i] = queue_l[r + i];
        buf_r[i] = queue_r[r + i];
    }
    for (int i = 0; i < chunk2; i++){
        buf_l[chunk1 + i] = queue_l[i];
        buf_r[chunk1 + i] = queue_r[i];
    }

    r += read_length;
    if (r >= I2S_QUEUE_MAX) r -= I2S_QUEUE_MAX;
    atomic_thread_fence(memory_order_release);
    atomic_store(&queue_read, r);
    return read_length;
}

int i2s_get_queue_length(void){
    int w = atomic_load(&queue_write);
    int r = atomic_load(&queue_read);

    if (w >= r) return w - r;
    return I2S_QUEUE_MAX - r + w;
}
