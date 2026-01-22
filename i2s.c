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

#include <stdatomic.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include "i2s.pio.h"
#include "i2s.h"

static uint i2s_dout_pin        = 18;
static uint i2s_clk_pin_base    = 20;
static uint i2s_mclk_pin        = 22;
static PIO  i2s_pio             = pio0;
static uint i2s_sm, i2s_mclk_sm;
static int i2s_dma_chan;
static CLOCK_MODE i2s_clock_mode = CLOCK_MODE_DEFAULT;
static I2S_MODE i2s_mode        = MODE_I2S;

static atomic_int queue_write = 0;
static atomic_int queue_read = 0;
static volatile int32_t queue_l[I2S_QUEUE_MAX];
static volatile int32_t queue_r[I2S_QUEUE_MAX];

static int32_t mul_l, mul_r;

//-100dB ~ 0dB (1dB step)
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

/**
 * @brief システムクロックを180.75MHzに設定する
 * 
 * @note 44.1kHz系 180.75 / 8 = 22.59375MHz
 */
static void set_sys_clock_180750khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 2, 1446 * MHZ, 4, 2);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 180750 * KHZ);
}

/**
 * @brief システムクロックを196.5MHzに設定する
 * 
 * @note 48.0kHz系 196.5 / 8 = 24.5625MHz
 */
static void set_sys_clock_196500khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 1, 1572 * MHZ, 4, 2);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 196500 * KHZ);
}

/**
 * @brief システムクロックをgpin0に設定する
 * 
 * @note gpin0 = 45.1584MHz
 */
static void set_sys_clock_gpin0(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    clock_configure_gpin(clk_sys, 20, 45158400, 45158400);
}

/**
 * @brief システムクロックをgpin1に設定する
 * 
 * @note gpin1 = 49.152MHz
 */
static void set_sys_clock_gpin1(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    clock_configure_gpin(clk_sys, 22, 49152 * KHZ, 49152 * KHZ);
}

void i2s_mclk_set_pin(uint data_pin, uint clock_pin_base, uint mclk_pin){
    i2s_dout_pin = data_pin;
    i2s_clk_pin_base = clock_pin_base;
    i2s_mclk_pin = mclk_pin;
}

//ロージッターモードを使うときはuart,i2c,spi設定よりも先に呼び出す
void i2s_mclk_set_config(PIO pio, CLOCK_MODE clock_mode, I2S_MODE mode){
    i2s_pio = pio;
    i2s_sm = pio_claim_unused_sm(pio, true);
    i2s_mclk_sm = pio_claim_unused_sm(pio, true);
    i2s_clock_mode = clock_mode;
    i2s_mode = mode;

    //あらかじめclk_periをclk_sysから分離する
    if (i2s_clock_mode == CLOCK_MODE_LOW_JITTER){
        vreg_set_voltage(VREG_VOLTAGE_1_15);
    }
    if (i2s_clock_mode != CLOCK_MODE_DEFAULT){
        clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    }
}

int i2s_get_dma_ch(void){
    return i2s_dma_chan;
}

I2S_MODE i2s_get_i2s_mode(void){
    return i2s_mode;
}

void i2s_init(void){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset, offset_mclk;
    uint pin_mask;

    //i2s pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, i2s_mclk_pin);

    //mclk init
    pio_sm_set_consecutive_pindirs(pio, i2s_mclk_sm, i2s_mclk_pin, 1, true);
    offset_mclk = pio_add_program(pio, &i2s_mclk_program);
    sm_config_mclk = i2s_mclk_program_get_default_config(offset_mclk);
    sm_config_set_set_pins(&sm_config_mclk, i2s_mclk_pin, 1);
    pio_sm_init(pio, i2s_mclk_sm, offset_mclk, &sm_config_mclk);
    pio_sm_set_enabled(pio, i2s_mclk_sm, true);

    //i2s data init
    offset = pio_add_program(pio, &i2s_data_program);
    sm_config = i2s_data_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, i2s_sm, offset, &sm_config);
    pin_mask = (1u << data_pin) | (3u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, i2s_sm, pin_mask, pin_mask);
    pio_sm_exec(pio, i2s_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_sm, 0);
    pio_sm_clear_fifos(pio, i2s_sm);
    pio_sm_set_enabled(pio, i2s_sm, true);
}

void pt8211_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    //pt8211 pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    //pt8211 data init
    offset = pio_add_program(pio, &i2s_pt8211_program);
    sm_config = i2s_pt8211_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &sm_config);
    pin_mask = (1u << data_pin) | (3u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
}

void exdf_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    //exdf pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, clock_pin_base + 2);

    //exdf data init
    offset = pio_add_program(pio, &i2s_exdf_program);
    sm_config = i2s_exdf_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 2);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &sm_config);
    pin_mask = (3u << data_pin) | (7u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
}

void i2s_dual_init(void){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset, offset_mclk;
    uint pin_mask;

    //i2s dual pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, i2s_mclk_pin);

    pio_sm_set_consecutive_pindirs(pio, i2s_mclk_sm, i2s_mclk_pin, 1, true);
    offset_mclk = pio_add_program(pio, &i2s_mclk_program);
    sm_config_mclk = i2s_mclk_program_get_default_config(offset_mclk);
    sm_config_set_set_pins(&sm_config_mclk, i2s_mclk_pin, 1);
    pio_sm_init(pio, i2s_mclk_sm, offset_mclk, &sm_config_mclk);
    pio_sm_set_enabled(pio, i2s_mclk_sm, true);

    offset = pio_add_program(pio, &i2s_data_dual_program);
    sm_config = i2s_data_dual_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 2);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, i2s_sm, offset, &sm_config);
    pin_mask = (3u << data_pin) | (3u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, i2s_sm, pin_mask, pin_mask);
    pio_sm_exec(pio, i2s_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_sm, 0);
    pio_sm_clear_fifos(pio, i2s_sm);
    pio_sm_set_enabled(pio, i2s_sm, true);
}

void pt8211_dual_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    //pt8211 dual pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    //pt8211 dual data init
    offset = pio_add_program(pio, &i2s_pt8211_dual_program);
    sm_config = i2s_pt8211_dual_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 2);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &sm_config);
    pin_mask = (3u << data_pin) | (3u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
}

void i2s_slave_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;
    
    //i2s slave pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    //i2s svalse data init
    pio_sm_set_consecutive_pindirs(pio, sm, data_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, clock_pin_base, 2, false);
    
    offset = pio_add_program(pio, &i2s_slave_program);
    sm_config = i2s_slave_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_in_pin_base(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, sm, offset, &sm_config);
    pio_sm_set_enabled(pio, sm, true);
}

void i2s_mclk_init(uint32_t audio_clock){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;

    switch (i2s_mode){
        case MODE_I2S:
            i2s_init();
            break;
        case MODE_PT8211:
            pt8211_init();
            break;
        case MODE_EXDF:
            exdf_init();
            break;
        case MODE_I2S_DUAL:
            i2s_dual_init();
            break;
        case MODE_PT8211_DUAL:
            pt8211_dual_init();
            break;
        case MODE_I2S_SLAVE:
            i2s_slave_init();
            break;
    }
    i2s_mclk_change_clock(audio_clock);

    //dma init
    i2s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config conf = dma_channel_get_default_config(i2s_dma_chan);
    
    channel_config_set_read_increment(&conf, true);
    channel_config_set_write_increment(&conf, false);
    channel_config_set_transfer_data_size(&conf, DMA_SIZE_32);
    channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));
    
    dma_channel_configure(
        i2s_dma_chan,
        &conf,
        &i2s_pio->txf[i2s_sm],
        NULL,
        0,
        false
    );
}

void i2s_mclk_change_clock(uint32_t audio_clock){
    //周波数変更
    if (i2s_mode == MODE_I2S_SLAVE){
        int div;
        if (audio_clock % 48000 == 0){
            div = 384000 / audio_clock;
            //ここで外部のクロック変更
            //picoのGPIOクロック出力だとクロック間の同期ができない
        }
        else{
            div = 352800 / audio_clock;
            //ここで外部のクロック変更
            //picoのGPIOクロック出力だとクロック間の同期ができない
        }
    }
    else if (i2s_clock_mode == CLOCK_MODE_DEFAULT){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 128);
        pio_sm_set_clkdiv(i2s_pio, i2s_sm, div);

        //mclk
        if (i2s_mode == MODE_I2S || i2s_mode == MODE_I2S_DUAL){
            if (audio_clock % 48000 == 0){
                div = (float)clock_get_hz(clk_sys) / (49.152f * (float)MHZ);
                pio_sm_set_clkdiv(i2s_pio, i2s_mclk_sm, div);
            }
            else{
                div = (float)clock_get_hz(clk_sys) / (45.1584f * (float)MHZ);
                pio_sm_set_clkdiv(i2s_pio, i2s_mclk_sm, div);
            }
        }
    }
    else{
        //mclk出力
        if (i2s_mode == MODE_I2S || i2s_mode == MODE_I2S_DUAL){
            switch (i2s_clock_mode){
                case CLOCK_MODE_LOW_JITTER:
                    pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_mclk_sm, 4, 0);
                    break;
                case CLOCK_MODE_EXTERNAL:
                    pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_mclk_sm, 1, 0);
                    break;
            }
        }

        //pio周波数変更
        uint dev;
        if (audio_clock % 48000 == 0){
            switch (i2s_clock_mode){
                case CLOCK_MODE_LOW_JITTER:
                    set_sys_clock_196500khz();
                    dev = 8 * 192000 / audio_clock;
                    break;
                case CLOCK_MODE_EXTERNAL:
                    set_sys_clock_gpin1();
                    dev = 2 * 192000 / audio_clock;
                    break;
            }
        }
        else {
            switch (i2s_clock_mode){
                case CLOCK_MODE_LOW_JITTER:
                    set_sys_clock_180750khz();
                    dev = 8 * 176400 / audio_clock;
                    break;
                case CLOCK_MODE_EXTERNAL:
                    set_sys_clock_gpin0();
                    dev = 2 * 176400 / audio_clock;
                    break;
            }
        }
        pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
    }
}

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

int i2s_format_piodata(int32_t *buf_l, int32_t *buf_r, int length, uint32_t *buf_tx){
    if (i2s_mode == MODE_EXDF){
        //並び替え
        for (int i = 0, j = 0; i < length; i++) {
            uint32_t left_upper = part1by1_16(buf_l[i] & 0xFFFF);
            uint32_t left_lower = part1by1_16(buf_l[i] >> 16);
            uint32_t right_upper = part1by1_16(buf_r[i] & 0xFFFF);
            uint32_t right_lower = part1by1_16(buf_r[i] >> 16);

            buf_tx[j++] = (left_upper << 1) | right_upper;
            buf_tx[j++] = (left_lower << 1) | right_lower;
        }
    }
    else if (i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_I2S_DUAL){
        //並び替え
        for (int i = 0, j = 0; i < length; i++) {
            uint32_t left_upper = part1by1_16(buf_l[i] & 0xFFFF);
            uint32_t left_lower = part1by1_16(buf_l[i] >> 16);
            uint32_t right_upper = part1by1_16(buf_r[i] & 0xFFFF);
            uint32_t right_lower = part1by1_16(buf_r[i] >> 16);

            buf_tx[j++] = (left_upper << 1) | right_upper;
            buf_tx[j++] = (left_lower << 1) | right_lower;

            //反転
            int32_t d_r, d_l;
            if (buf_l[i] == INT32_MIN){
                d_l = INT32_MAX;
            }
            else{
                d_l = -buf_l[i];
            }
            if (buf_r[i] == INT32_MIN){
                d_r = INT32_MAX;
            }
            else{
                d_r = -buf_r[i];
            }

            left_upper = part1by1_16(d_l & 0xFFFF);
            left_lower = part1by1_16(d_l >> 16);
            right_upper = part1by1_16(d_r & 0xFFFF);
            right_lower = part1by1_16(d_r >> 16);

            buf_tx[j++] = (left_upper << 1) | right_upper;
            buf_tx[j++] = (left_lower << 1) | right_lower;
        }
        length *= 2;
    }
    else {
        for (int i = 0, j = 0; i < length; i++){
            buf_tx[j++] = buf_l[i];
            buf_tx[j++] = buf_r[i];
        }
    }

    return length * 2;
}
