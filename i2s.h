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

#ifndef I2S_H
#define I2S_H
#include "hardware/pio.h"

#define I2S_DEQUEUE_LEN     48
#define I2S_QUEUE_MAX       (I2S_DEQUEUE_LEN * 8)


typedef enum {
    MODE_I2S,
    MODE_PT8211,
    MODE_EXDF,
    MODE_I2S_DUAL,
    MODE_PT8211_DUAL,
    MODE_I2S_SLAVE
} I2S_MODE;

typedef enum {
    CLOCK_MODE_DEFAULT,
    CLOCK_MODE_LOW_JITTER,
    CLOCK_MODE_EXTERNAL
} CLOCK_MODE;

/**
 * @brief i2sの出力ピンを設定する
 * 
 * @param data_pin data出力ピン
 * @param clock_pin_base LRCLK出力ピン
 * @param mclk_pin_pin LRCLK出力ピン
 * @note BCLK=clock_pin_base+1
 * @note MODE_EXDFの場合、DOUTL = data_pin, DOUTR = data_pin + 1, WCK=clock_pin_base, BCK=clock_pin_base+1 MCLK=clock_pin_base+2
 */
void i2s_mclk_set_pin(uint data_pin, uint clock_pin_base, uint mclk_pin);

/**
 * @brief i2sの設定を行う
 * 
 * @param pio i2sに使用するpio pio0 or pio1
 * @param clock_mode クロックモードの選択 (CLOCK_MODE_DEFAULT, CLOCK_MODE_LOW_JITTER, CLOCK_MODE_EXTERNAL)
 * @param mode 出力するフォーマットを選択する (MODE_I2S, MODE_PT8211, MODE_EXDF, MODE_I2S_DUAL, MODE_PT8211_DUAL)
 * @note lowジッタモードを使用する場合はuart,i2s,spi設定よりも先に呼び出す
 * @note MODE_PT8211はBCLK32fsのlsbj16,MCLKなし
 */
void i2s_mclk_set_config(PIO pio, CLOCK_MODE clock_mode, I2S_MODE mode);

/**
 * @brief i2sで使用するdmaのチャンネルを取得する
 * 
 * @return dma ch
 */
int i2s_get_dma_ch(void);

/**
 * @brief i2sのモードを取得する
 * 
 * @return i2s mode
 */
I2S_MODE i2s_get_i2s_mode(void);

/**
 * @brief i2sの初期化を行う
 * 
 * @param audio_clock サンプリング周波数
 * @note 呼び出してすぐにi2sの出力が開始される
 */
void i2s_mclk_init(uint32_t audio_clock);

/**
 * @brief i2sの周波数を変更する
 * 
 * @param audio_clock サンプリング周波数
 */
void i2s_mclk_change_clock(uint32_t audio_clock);

/**
 * @brief i2sのバッファに格納する
 * 
 * @param buf_l 格納するLchデータのポインタ
 * @param buf_r 格納するRchデータのポインタ
 * @param length 格納するデータの長さ
 * @return true 成功
 * @return false 失敗(バッファが一杯)
 */
bool i2s_enqueue(int32_t *buf_l, int32_t *buf_r, int length);

/**
 * @brief i2sのバッファからデータを取り出す
 * 
 * @param buf_l 取り出したLchデータの格納先のポインタ
 * @param buf_r 取り出したLchデータの格納先のポインタ
 * @param length 取り出すデータの長さ
 * @return 取り出しに成功したデータの長さ
 */
int i2s_dequeue(int32_t *buf_l, int32_t *buf_r, int length);

/**
 * @brief i2sのバッファの格納量を取得する
 * 
 * @return バッファの長さ
 */
int i2s_get_queue_length(void);

/**
 * @brief usb audioから送られてくる8ビットパックデータをint32_tに変換する
 * 
 * @param in 8ビットパックデータのポインタ
 * @param sample 8ビットパックデータの長さ
 * @param resolution 
 * @param buf_l 変換したLchデータの格納先ののポインタ
 * @param buf_r 変換したRchデータの格納先ののポインタ
 * @return 変換したL/Rchデータの長さ
 */
int i2s_unpack_uacdata(uint8_t* in, int sample, uint8_t resolution, int32_t *lch_buf, int32_t *rch_buf);

/**
 * @brief i2sの音量を変更する
 * 
 * @param v 音量 (n[dB] << 128)
 * @param ch チャンネル 0:L&R 1:L 2:R
 */
void i2s_volume_change(int16_t v, int8_t ch);

/**
 * @brief 音量処理
 * 
 * @param buf_l 音量処理するLchデータのポインタ
 * @param buf_r 音量処理するRchデータのポインタ
 * @param length 音量処理するデータの長さ
 */
void i2s_volume(int32_t *buf_l, int32_t *buf_r, int length);

/**
 * @brief i2s pioの形式にデータを変換する
 * 
 * @param buf_l 変換するLchデータのポインタ
 * @param buf_r 変換するRchデータのポインタ
 * @param length 変換するデータの長さ
 * @param buf_tx 変換したデータを格納するポインタ
 * @return 変換後のデータの長さ
 */
int i2s_format_piodata(int32_t *lch_buf, int32_t *rch_buf, int length, uint32_t *buf_tx);

/**
 * @brief EXDFのLRのビットを交互に並び替える操作の高速化関数
 * 
 * @param x 入力
 */
static __force_inline uint32_t part1by1_16(uint16_t x){
    uint32_t res = x;
    res = (res | (res << 8))  & 0x00FF00FF;
    res = (res | (res << 4))  & 0x0F0F0F0F;
    res = (res | (res << 2))  & 0x33333333;
    res = (res | (res << 1))  & 0x55555555;
    return res;
}

#endif