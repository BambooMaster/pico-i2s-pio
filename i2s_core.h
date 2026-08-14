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

#ifndef I2S_CORE_H
#define I2S_CORE_H
#include "hardware/pio.h"

#ifndef I2S_MAX_FREQ_KHZ
#define I2S_MAX_FREQ_KHZ    384
#endif

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
 * @brief I2S出力ピンの設定
 * 
 * @param data_pin データ出力ピン
 * @param clock_pin_base LRCLK/WCKピン (BCLKはclock_pin_base+1)
 * @param mclk_pin MCLKピン
 * @note BCLK = clock_pin_base + 1
 * @note MODE_EXDFの場合: DOUTL=data_pin, DOUTR=data_pin+1, WCK=clock_pin_base, BCK=clock_pin_base+1, MCLK=clock_pin_base+2
 */
void i2s_set_pin(uint data_pin, uint clock_pin_base, uint mclk_pin);

static inline void i2s_mclk_set_pin(uint data_pin, uint clock_pin_base, uint mclk_pin){
    i2s_set_pin(data_pin, clock_pin_base,  mclk_pin);
}
/**
 * @brief I2Sドライバの設定
 * 
 * @param pio 使用するPIOインスタンス (pio0 または pio1)
 * @param clock_mode クロック生成モード (DEFAULT, LOW_JITTER, EXTERNAL)
 * @param mode 出力フォーマット (I2S, PT8211, EXDF, DUAL等)
 * @note Lowジッタモードを使用する場合は、UART/I2C/SPI設定よりも先に呼び出す必要があります。
 * @note MODE_PT8211はBCLK=32fs (LSB Justified 16bit), MCLKなしとなります。
 */
void i2s_set_config(PIO pio, CLOCK_MODE clock_mode, I2S_MODE mode);

static inline void i2s_mclk_set_config(PIO pio, CLOCK_MODE clock_mode, I2S_MODE mode){
    i2s_set_config(pio, clock_mode, mode);
}

/**
 * @brief 現在のI2Sモードを取得
 * 
 * @return 現在のI2Sモード
 */
I2S_MODE i2s_get_i2s_mode(void);

/**
 * @brief I2Sドライバの初期化と開始
 * 
 * @param sample_rate_hz サンプリングレート (Hz)
 * @note 呼び出し直後からI2S出力が開始されます。
 */
void i2s_init(uint32_t sample_rate_hz);

static inline void i2s_mclk_init(uint32_t sample_rate_hz){
    i2s_init(sample_rate_hz);
}


/**
 * @brief サンプリングレートの変更
 * 
 * @param sample_rate_hz 新しいサンプリングレート (Hz)
 */
void i2s_change_clock(uint32_t sample_rate_hz);

static inline void i2s_mclk_change_clock(uint32_t sample_rate_hz){
    i2s_change_clock(sample_rate_hz);
}

/**
 * @brief 現在のi2sサンプリングレート取得
 * 
 * @return i2sサンプリングレート
 */
uint32_t i2s_get_sample_rate_hz(void);

static inline uint32_t i2s_get_freq(void){
    i2s_get_sample_rate_hz();
}

/**
 * @brief DMA転送の開始 (ブロッキング待機含む)
 * 
 * @param tx_buf_a 送信バッファA
 * @param tx_buf_b 送信バッファB
 * @param tx_length 転送データ長
 * @note Dual/EXDFモード時は、送信バッファBのデータが data_pin+1 に出力されます。
 */
void i2s_dma_transfer_blocking(int32_t *tx_buf_a, int32_t *tx_buf_b, int tx_length);

/**
 * @brief L/RデータをPIO送信形式に変換
 * 
 * @param buf_l Lch入力データ
 * @param buf_r Rch入力データ
 * @param length データ長 (サンプル数)
 * @param tx_buf_a 送信バッファA
 * @param tx_buf_b 送信バッファB
 * @return 生成された送信データの長さ
 * @note Dual/EXDFモード時は、送信バッファBのデータが data_pin+1 に出力されます。
 */
int i2s_format_piodata(int32_t *buf_l, int32_t *buf_r, int length, uint32_t *tx_buf_a, uint32_t *tx_buf_b);

#endif
