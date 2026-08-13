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

#ifndef I2S_UAC_H
#define I2S_UAC_H
#include "pico/stdlib.h"

/**
 * @brief USBオーディオデータ(8bitパック)を32bit整数へ変換
 * 
 * @param in 入力データポインタ (USBパケット)
 * @param sample 入力データ長 (バイト数)
 * @param resolution ビット深度 (16, 24, 32)
 * @param buf_l Lch出力バッファ
 * @param buf_r Rch出力バッファ
 * @return 変換後のデータ長 (サンプル数)
 */
int i2s_unpack_uacdata(uint8_t* in, int sample, uint8_t resolution, int32_t *buf_l, int32_t *buf_r);

/**
 * @brief 出力音量の設定
 * 
 * @param v 音量値 (単位: 1/256 dB)
 * @param ch 対象チャンネル (0:Master, 1:L, 2:R)
 */
void i2s_volume_change(int16_t v, int8_t ch);

/**
 * @brief 音量調整処理の適用
 * 
 * @param buf_l Lchデータバッファ (In/Out)
 * @param buf_r Rchデータバッファ (In/Out)
 * @param length データ長 (サンプル数)
 */
void i2s_volume(int32_t *buf_l, int32_t *buf_r, int length);

#endif
