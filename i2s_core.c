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
static uint i2s_sm, i2s_mclk_sm, i2s_dual_sm, i2s_sm_mask;
static int i2s_dma_chan_a, i2s_dma_chan_b;
static CLOCK_MODE i2s_clock_mode = CLOCK_MODE_DEFAULT;
static I2S_MODE i2s_mode        = MODE_I2S;

static atomic_uint i2s_freq = 44100;

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

void i2s_set_pin(uint data_pin, uint clock_pin_base, uint mclk_pin){
    i2s_dout_pin = data_pin;
    i2s_clk_pin_base = clock_pin_base;
    i2s_mclk_pin = mclk_pin;
}

// ロージッターモードを使うときはuart,i2c,spi設定よりも先に呼び出す
void i2s_set_config(PIO pio, CLOCK_MODE clock_mode, I2S_MODE mode){
    i2s_pio = pio;
    i2s_sm = pio_claim_unused_sm(pio, true);
    i2s_mclk_sm = pio_claim_unused_sm(pio, true);
    i2s_dual_sm = pio_claim_unused_sm(pio, true);
    i2s_clock_mode = clock_mode;
    i2s_mode = mode;

    // あらかじめclk_periをclk_sysから分離する
    if (i2s_clock_mode == CLOCK_MODE_LOW_JITTER){
        vreg_set_voltage(VREG_VOLTAGE_1_15);
    }
    if (i2s_clock_mode != CLOCK_MODE_DEFAULT){
        clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    }
}

I2S_MODE i2s_get_i2s_mode(void){
    return i2s_mode;
}

void i2s_pio_init(void){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset, offset_mclk;
    uint pin_mask;

    // i2s pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, i2s_mclk_pin);

    // mclk init
    pio_sm_set_consecutive_pindirs(pio, i2s_mclk_sm, i2s_mclk_pin, 1, true);
    offset_mclk = pio_add_program(pio, &i2s_mclk_program);
    sm_config_mclk = i2s_mclk_program_get_default_config(offset_mclk);
    sm_config_set_set_pins(&sm_config_mclk, i2s_mclk_pin, 1);
    pio_sm_init(pio, i2s_mclk_sm, offset_mclk, &sm_config_mclk);
    pio_sm_set_enabled(pio, i2s_mclk_sm, true);

    // i2s data init
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

void pt8211_pio_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    // pt8211 pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    // pt8211 data init
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

void exdf_pio_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    // exdf pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, clock_pin_base + 2);

    // exdf_a init
    offset = pio_add_program(pio, &i2s_exdf_a_program);
    sm_config = i2s_exdf_a_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_sm, offset, &sm_config);
    pin_mask = (1u << data_pin) | (7u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, i2s_sm, pin_mask, pin_mask);
    pio_sm_exec(pio, i2s_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_sm, 0);
    pio_sm_clear_fifos(pio, i2s_sm);

    // exdf_b init
    pio_sm_set_consecutive_pindirs(pio, i2s_dual_sm, data_pin + 1, 1, true);
    offset = pio_add_program(pio, &i2s_exdf_b_program);
    sm_config = i2s_exdf_b_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin + 1, 1);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_dual_sm, offset, &sm_config);
    pio_sm_exec(pio, i2s_dual_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_dual_sm, 0);
    pio_sm_clear_fifos(pio, i2s_dual_sm);

    i2s_sm_mask = (1u << i2s_sm) | (1u << i2s_dual_sm);
    pio_enable_sm_mask_in_sync(pio, i2s_sm_mask);
}

void i2s_dual_pio_init(void){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset, offset_mclk;
    uint pin_mask;

    // i2s dual pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    pio_gpio_init(pio, i2s_mclk_pin);

    // mclk init
    pio_sm_set_consecutive_pindirs(pio, i2s_mclk_sm, i2s_mclk_pin, 1, true);
    offset_mclk = pio_add_program(pio, &i2s_mclk_program);
    sm_config_mclk = i2s_mclk_program_get_default_config(offset_mclk);
    sm_config_set_set_pins(&sm_config_mclk, i2s_mclk_pin, 1);
    pio_sm_init(pio, i2s_mclk_sm, offset_mclk, &sm_config_mclk);
    pio_sm_set_enabled(pio, i2s_mclk_sm, true);

    // i2s data init
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

    // i2s dual init
    pio_sm_set_consecutive_pindirs(pio, i2s_dual_sm, data_pin + 1, 1, true);
    offset = pio_add_program(pio, &i2s_data_dual_program);
    sm_config = i2s_data_dual_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin + 1, 1);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_dual_sm, offset, &sm_config);
    pio_sm_exec(pio, i2s_dual_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_dual_sm, 0);
    pio_sm_clear_fifos(pio, i2s_dual_sm);

    i2s_sm_mask = (1u << i2s_sm) | (1u << i2s_dual_sm);
    pio_enable_sm_mask_in_sync(pio, i2s_sm_mask);
}

void pt8211_dual_pio_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;

    // pt8211 dual pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, data_pin + 1);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    // pt8211 data init
    offset = pio_add_program(pio, &i2s_pt8211_program);
    sm_config = i2s_pt8211_program_get_default_config(offset);
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

    // pt8211 dual init
    pio_sm_set_consecutive_pindirs(pio, i2s_dual_sm, data_pin + 1, 1, true);
    offset = pio_add_program(pio, &i2s_pt8211_dual_program);
    sm_config = i2s_pt8211_dual_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, data_pin + 1, 1);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_dual_sm, offset, &sm_config);
    pio_sm_exec(pio, i2s_dual_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_dual_sm, 0);
    pio_sm_clear_fifos(pio, i2s_dual_sm);

    i2s_sm_mask = (1u << i2s_sm) | (1u << i2s_dual_sm);
    pio_enable_sm_mask_in_sync(pio, i2s_sm_mask);
}

void i2s_slave_pio_init(void){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;
    uint pin_mask;
    
    // i2s slave pin init
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);

    // i2s slave data init
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

void i2s_init(uint32_t audio_clock){
    pio_sm_config sm_config, sm_config_mclk;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;

    switch (i2s_mode){
        case MODE_I2S:
            i2s_pio_init();
            break;
        case MODE_PT8211:
            pt8211_pio_init();
            break;
        case MODE_EXDF:
            exdf_pio_init();
            break;
        case MODE_I2S_DUAL:
            i2s_dual_pio_init();
            break;
        case MODE_PT8211_DUAL:
            pt8211_dual_pio_init();
            break;
        case MODE_I2S_SLAVE:
            i2s_slave_pio_init();
            break;
    }
    i2s_change_clock(audio_clock);

    // dma init
    i2s_dma_chan_a = dma_claim_unused_channel(true);
    dma_channel_config conf = dma_channel_get_default_config(i2s_dma_chan_a);
    
    channel_config_set_read_increment(&conf, true);
    channel_config_set_write_increment(&conf, false);
    channel_config_set_transfer_data_size(&conf, DMA_SIZE_32);
    channel_config_set_dreq(&conf, pio_get_dreq(pio, i2s_sm, true));
    
    dma_channel_configure(
        i2s_dma_chan_a,
        &conf,
        &i2s_pio->txf[i2s_sm],
        NULL,
        0,
        false
    );

    if (i2s_mode == MODE_I2S_DUAL || i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_EXDF){
        i2s_dma_chan_b = dma_claim_unused_channel(true);
        conf = dma_channel_get_default_config(i2s_dma_chan_b);
        
        channel_config_set_read_increment(&conf, true);
        channel_config_set_write_increment(&conf, false);
        channel_config_set_transfer_data_size(&conf, DMA_SIZE_32);
        channel_config_set_dreq(&conf, pio_get_dreq(pio, i2s_dual_sm, true));
        
        dma_channel_configure(
            i2s_dma_chan_b,
            &conf,
            &i2s_pio->txf[i2s_dual_sm],
            NULL,
            0,
            false
        );
    }
}

void i2s_change_clock(uint32_t audio_clock){
    // 周波数変更
    atomic_store(&i2s_freq, audio_clock);
    
    if (i2s_mode == MODE_I2S_SLAVE){
        if (audio_clock % 48000 == 0){
            // ここで外部のクロック変更
            // picoのGPIOクロック出力だとクロック間の同期ができない
        }
        else{
            // ここで外部のクロック変更
            // picoのGPIOクロック出力だとクロック間の同期ができない
        }
    }
    else if (i2s_clock_mode == CLOCK_MODE_DEFAULT){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 128);

        if (i2s_mode == MODE_I2S_DUAL || i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_EXDF){
            pio_set_sm_mask_enabled(i2s_pio, i2s_sm_mask, false);
            pio_sm_set_clkdiv(i2s_pio, i2s_sm, div);
            pio_sm_set_clkdiv(i2s_pio, i2s_dual_sm, div);
            pio_enable_sm_mask_in_sync(i2s_pio, i2s_sm_mask);
        }
        else{
            pio_sm_set_clkdiv(i2s_pio, i2s_sm, div);
        }

        // mclk
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
        // mclk出力
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

        // pio周波数変更
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

        if (i2s_mode == MODE_I2S_DUAL || i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_EXDF){
            pio_set_sm_mask_enabled(i2s_pio, i2s_sm_mask, false);
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_dual_sm, dev, 0);
            pio_enable_sm_mask_in_sync(i2s_pio, i2s_sm_mask);
        }
        else{
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
        }
    }
}

uint32_t i2s_get_freq(void){
    return atomic_load(&i2s_freq);
}

void i2s_dma_transfer_blocking(int32_t *tx_buf_a, int32_t *tx_buf_b, int tx_length){
    if (i2s_mode == MODE_I2S_DUAL || i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_EXDF){
        uint32_t mask = (1u << i2s_dma_chan_a) | (1u << i2s_dma_chan_b);
        while (dma_channel_is_busy(i2s_dma_chan_a) || dma_channel_is_busy(i2s_dma_chan_b)) tight_loop_contents();

        dma_channel_set_transfer_count(i2s_dma_chan_a, tx_length, false);
        dma_channel_set_read_addr(i2s_dma_chan_a, tx_buf_a, false);
        dma_channel_set_transfer_count(i2s_dma_chan_b, tx_length, false);
        dma_channel_set_read_addr(i2s_dma_chan_b, tx_buf_b, false);

        dma_start_channel_mask(mask);
    }
    else{
        dma_channel_wait_for_finish_blocking(i2s_dma_chan_a);
        dma_channel_transfer_from_buffer_now(i2s_dma_chan_a, tx_buf_a, tx_length);
    }
}

int i2s_format_piodata(int32_t *buf_l, int32_t *buf_r, int length, uint32_t *tx_buf_a, uint32_t *tx_buf_b){
    I2S_MODE i2s_mode = i2s_get_i2s_mode();
    if (i2s_mode == MODE_EXDF){
        for (int i = 0; i < length; i++){
            tx_buf_a[i] = buf_l[i];
            tx_buf_b[i] = buf_r[i];
        }
    }
    else if (i2s_mode == MODE_PT8211_DUAL || i2s_mode == MODE_I2S_DUAL){
        // 並び替え
        for (int i = 0, j = 0; i < length; i++) {
            // 反転
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

            tx_buf_a[j] = buf_l[i];
            tx_buf_b[j] = buf_r[i];
            j++;
            tx_buf_a[j] = d_l;
            tx_buf_b[j] = d_r;
            j++;
        }
        length *= 2;
    }
    else {
        for (int i = 0, j = 0; i < length; i++){
            tx_buf_a[j++] = buf_l[i];
            tx_buf_a[j++] = buf_r[i];
        }
        length *= 2;
    }

    return length;
}
