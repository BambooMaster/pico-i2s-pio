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

#ifndef I2S_QUEUE_H
#define I2S_QUEUE_H
#include "i2s.h"

#ifndef I2S_QUEUE_LEN
#define I2S_QUEUE_LEN     10
#endif
#define I2S_QUEUE_MAX       (I2S_MAX_FREQ_KHZ * I2S_QUEUE_LEN)
#define I2S_DEQUEUE_LEN     48

/**
 * @brief 送信キューへのデータ追加
 * 
 * @param buf_l Lchデータバッファ
 * @param buf_r Rchデータバッファ
 * @param length データ長 (サンプル数)
 * @return true 成功
 * @return false 失敗 (キューが満杯)
 */
bool i2s_enqueue(int32_t *buf_l, int32_t *buf_r, int length);

/**
 * @brief 送信キューからのデータ取り出し
 * 
 * @param buf_l Lch出力バッファ
 * @param buf_r Rch出力バッファ
 * @param length 要求データ長 (サンプル数)
 * @return 実際に取り出したデータ長
 */
int i2s_dequeue(int32_t *buf_l, int32_t *buf_r, int length);

/**
 * @brief 送信キュー内のデータ残量を取得
 * 
 * @return キュー内のデータ数 (サンプル数)
 */
int i2s_get_queue_length(void);

#endif
