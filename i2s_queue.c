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
#include <string.h>
#include "pico/stdlib.h"

#include "i2s_core.h"

static atomic_int queue_write = 0;
static atomic_int queue_read = 0;
static int32_t queue_l[I2S_QUEUE_MAX];
static int32_t queue_r[I2S_QUEUE_MAX];

bool i2s_enqueue(const int32_t *buf_l, const int32_t *buf_r, int length){
    if (length <= 0 || length > I2S_QUEUE_MAX - 1) return false;

    int queue_free_length;
    int w = atomic_load_explicit(&queue_write, memory_order_relaxed);
    int r = atomic_load_explicit(&queue_read,  memory_order_acquire);
    if (w >= r){
        queue_free_length = I2S_QUEUE_MAX - 1 - w + r;
    }
    else{
        queue_free_length = r - w - 1;
    }
    if (queue_free_length < length) return false;

    int chunk1, chunk2;
    if (w + length >= I2S_QUEUE_MAX){
        chunk1 = I2S_QUEUE_MAX - w;
        chunk2 = length - chunk1;
    }
    else{
        chunk1 = length;
        chunk2 = 0;
    }

    memcpy((void*)&queue_l[w], buf_l, chunk1 * sizeof(int32_t));
    memcpy((void*)&queue_r[w], buf_r, chunk1 * sizeof(int32_t));
    if (chunk2 > 0){
        memcpy((void*)&queue_l[0], buf_l + chunk1, chunk2 * sizeof(int32_t));
        memcpy((void*)&queue_r[0], buf_r + chunk1, chunk2 * sizeof(int32_t));
    }

    w += length;
    if (w >= I2S_QUEUE_MAX) w -= I2S_QUEUE_MAX;
    atomic_store_explicit(&queue_write, w, memory_order_release);
    return true;
}

int i2s_dequeue(int32_t *buf_l, int32_t *buf_r, int length){
    if (length <= 0) return 0;

    int read_length;
    int r = atomic_load_explicit(&queue_read,  memory_order_relaxed);
    int w = atomic_load_explicit(&queue_write, memory_order_acquire);
    if (w >= r){
        read_length = w - r;
    }
    else{
        read_length = I2S_QUEUE_MAX - r + w;
    }
    if (read_length <= 0) return 0;

    if (read_length > length) read_length = length;

    int chunk1, chunk2;
    if (r + read_length >= I2S_QUEUE_MAX){
        chunk1 = I2S_QUEUE_MAX - r;
        chunk2 = read_length - chunk1;
    }
    else{
        chunk1 = read_length;
        chunk2 = 0;
    }

    memcpy(buf_l, (void*)&queue_l[r], chunk1 * sizeof(int32_t));
    memcpy(buf_r, (void*)&queue_r[r], chunk1 * sizeof(int32_t));
    if (chunk2 > 0){
        memcpy(buf_l + chunk1, (void*)&queue_l[0], chunk2 * sizeof(int32_t));
        memcpy(buf_r + chunk1, (void*)&queue_r[0], chunk2 * sizeof(int32_t));
    }

    r += read_length;
    if (r >= I2S_QUEUE_MAX) r -= I2S_QUEUE_MAX;
    atomic_store_explicit(&queue_read, r, memory_order_release);
    return read_length;
}

int i2s_get_queue_length(void){
    int w = atomic_load_explicit(&queue_write, memory_order_acquire);
    int r = atomic_load_explicit(&queue_read,  memory_order_acquire);

    if (w >= r) return w - r;
    return I2S_QUEUE_MAX - r + w;
}
