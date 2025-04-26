// SPDX-License-Identifier: MIT

/**
 * @file i2s.c
 * @author BambooMaster (https://misskey.hakoniwa-project.com/@BambooMaster)
 * @brief pico-i2s-pio
 * @version 0.3
 * @date 2025-04-06
 * 
 */

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

static spin_lock_t* queue_spin_lock;
static bool clk_48khz;

static uint i2s_dout_pin        = 18;
static uint i2s_clk_pin_base    = 19;
static PIO  i2s_pio             = pio0;
static uint i2s_sm              = 0;

static int i2s_dma_chan         = 0;
static bool i2s_use_core1       = false;
static bool i2s_low_jitter      = false;
static bool i2s_overclock       = false;
static I2S_MODE i2s_mode        = MODE_I2S;

static int8_t i2s_buf_length;
static uint8_t enqueue_pos;
static uint8_t dequeue_pos;

static int32_t i2s_buf[I2S_BUF_DEPTH][I2S_DATA_LEN];
static uint32_t i2s_sample[I2S_BUF_DEPTH];

static int32_t mul_l;
static int32_t mul_r;

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
 * @brief set_playback_stateのデフォルトハンドラ
 * 
 * @param state 再生状態 true:再生開始 false:再生停止
 * @note PICO_DEFAULT_LED_PINで通知
 */
static inline void default_playback_handler(bool state){
    gpio_put(PICO_DEFAULT_LED_PIN, state);
}
static ExternalFunction playback_handler = default_playback_handler;

/**
 * @brief i2s再生状態の切り替わりを通知する
 * 
 * @param state 再生状態 true:再生開始 false:再生停止
 */
static inline void set_playback_state(bool state){
    playback_handler(state);
}

/**
 * @brief システムクロックを271MHzに設定する
 * 
 * @note 44.1kHz系 271 / 12 = 22.583MHz
 */
static void set_sys_clock_271000khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 2, 1626 * MHZ, 6, 1);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 271 * MHZ);
}

/**
 * @brief システムクロックを135.5MHzに設定する
 * 
 * @note 44.1kHz系 135.5 / 6 = 22.583MHz
 */
static void set_sys_clock_135500khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 2, 1626 * MHZ, 6, 1);
    clock_configure_int_divider(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 271 * MHZ, 2);
}

/**
 * @brief システムクロックを295MHzに設定する
 * 
 * @note 48kHz系 295 / 12 = 24.583MHz
 */
static void set_sys_clock_295000khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 2, 1770 * MHZ, 6, 1);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 295 * MHZ);
}

/**
 * @brief システムクロックを147.5MHzに設定する
 * 
 * @note 48kHz系 147.5 / 6 = 24.583MHz
 */
static void set_sys_clock_147500khz(void){
    while (running_on_fpga()) tight_loop_contents();
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    pll_init(pll_sys, 2, 1770 * MHZ, 6, 1);
    clock_configure_int_divider(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 295 * MHZ, 2);
}

/**
 * @brief i2sのバッファからデータを取り出すハンドラ
 * 
 * @note use_core1がfalseのときに呼び出される
 */
static void __isr __time_critical_func(i2s_handler)(){
	static bool mute;
	static int32_t mute_buff[96 * 2] = {0};
	static uint32_t mute_len = sizeof(mute_buff) / sizeof(int32_t);
	
	if (i2s_buf_length == 0){
        mute = true;
        set_playback_state(false);
    }
	else if (i2s_buf_length >= I2S_START_LEVEL && mute == true){
        mute = false;
        set_playback_state(true);
    }

	if (mute == false){
		dma_channel_transfer_from_buffer_now(i2s_dma_chan, i2s_buf[dequeue_pos], i2s_sample[dequeue_pos]);
		dequeue_pos++;
		if (dequeue_pos >= I2S_BUF_DEPTH){
            dequeue_pos = 0;
        }
		i2s_buf_length--;
	}
	else{
		dma_channel_transfer_from_buffer_now(i2s_dma_chan, mute_buff, mute_len);
	}
    
   	dma_hw->ints0 = 1u << i2s_dma_chan;
}

/**
 * @brief core1のメイン関数
 * 
 * @note use_core1がtrueのときに呼び出される
 */
static void defalut_core1_main(void){
    int32_t* buff;
    int dma_sample[2], sample;
    bool mute = false;
    int32_t mute_buff[96 * 2] = {0};
    uint32_t mute_len = sizeof(mute_buff) / sizeof(int32_t);
    int8_t buf_length;
    static int32_t dma_buff[2][I2S_DATA_LEN];
    uint8_t dma_use = 0;

    while (1){
        buf_length = i2s_get_buf_length();

        if (buf_length == 0){
            mute = true;
            set_playback_state(false);
        }
        else if (buf_length >= I2S_START_LEVEL && mute == true){
            mute = false;
            set_playback_state(true);
        }

        if (mute == true){
            buff = mute_buff;
            sample = mute_len;
        }
        else if (i2s_dequeue(&buff, &sample) == false){
            buff = mute_buff;
            sample = mute_len;
        }
        
        //i2sバッファに格納
        for (int i = 0; i < sample; i++){
            dma_buff[dma_use][i] = buff[i];
        }
        dma_sample[dma_use] = sample;

        dma_channel_wait_for_finish_blocking(i2s_dma_chan);
        dma_channel_transfer_from_buffer_now(i2s_dma_chan, dma_buff[dma_use], dma_sample[dma_use]);
        dma_use ^= 1;
    }
}
static Core1MainFunction core1_main_funcion = defalut_core1_main;

void i2s_mclk_set_pin(int data_pin, int clock_pin_base){
    i2s_dout_pin = data_pin;
    i2s_clk_pin_base = clock_pin_base;
}

//ロージッターモードを使うときはuart,i2s,spi設定よりも先に呼び出す
void i2s_mclk_set_config(PIO pio, uint sm, int dma_ch, bool use_core1, bool low_jitter, bool overclock, I2S_MODE mode){
    i2s_pio = pio;
    i2s_sm = sm;
    i2s_dma_chan = dma_ch;
    i2s_use_core1 = use_core1;
    i2s_low_jitter = low_jitter;
    i2s_overclock = overclock;
    i2s_mode = mode;

    //あらかじめclk_periをclk_sysから分離する
    if (i2s_low_jitter == true){
        if (overclock == true){
            vreg_set_voltage(VREG_VOLTAGE_1_20);
        }
        clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, USB_CLK_HZ);
    }
}

void i2s_mclk_init(uint32_t audio_clock){
    pio_sm_config sm_config;
    PIO pio = i2s_pio;
    uint sm = i2s_sm;
    uint data_pin = i2s_dout_pin;
    uint clock_pin_base = i2s_clk_pin_base;
    uint offset;

    //再生状態をGPIO25で通知
    if (playback_handler == default_playback_handler){
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    }

    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    if (i2s_low_jitter == false && i2s_mode == MODE_I2S){
        pio_gpio_init(pio, clock_pin_base + 2);
    }

    if (i2s_mode == MODE_PT8211){
        offset = pio_add_program(pio, &i2s_pt8211_program);
        sm_config = i2s_pt8211_program_get_default_config(offset);
    }
    else if (i2s_low_jitter == false){
        offset = pio_add_program(pio, &i2s_mclk_256_program);
        sm_config = i2s_mclk_256_program_get_default_config(offset);
    }
    else{
        offset = pio_add_program(pio, &i2s_no_mclk_program);
        sm_config = i2s_no_mclk_program_get_default_config(offset);
    }
    
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    queue_spin_lock = spin_lock_init(spin_lock_claim_unused(true));

    i2s_buf_length = 0;
    enqueue_pos = 0;
    dequeue_pos = 0;

    if (i2s_low_jitter == false && i2s_mode == MODE_I2S){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 512);
        sm_config_set_clkdiv(&sm_config, div);
    }
    else if (i2s_low_jitter == false && i2s_mode == MODE_PT8211){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 128);
        sm_config_set_clkdiv(&sm_config, div);
    }
    else{
        //sys_clk変更
        if (audio_clock % 48000 == 0){
            if (i2s_overclock == true){
                set_sys_clock_295000khz();
            }
            else {
                set_sys_clock_147500khz();
            }
            clk_48khz = true;
        }
        else {
            if (i2s_overclock == true){
                set_sys_clock_271000khz();
            }
            else {
                set_sys_clock_135500khz();
            }
            clk_48khz = false;
        }

        //mclk出力
        if (i2s_mode == MODE_I2S){
            clock_gpio_init_int_frac8(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 12, 0);
        }

        //pio周波数変更
        uint dev;
        if (clk_48khz == true){
            dev = 12 * 192000 / audio_clock;
            if (i2s_overclock == false){
                dev /= 2;
            }
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
        }
        else {
            dev = 12 * 192000 / audio_clock;
            if (i2s_overclock == false){
                dev /= 2;
            }
            dev = 12 * 176400 / audio_clock;
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
        }
    }

    pio_sm_init(pio, sm, offset, &sm_config);

    uint pin_mask;
    if (i2s_low_jitter == false && i2s_mode == MODE_I2S){
        pin_mask = (1u << data_pin) | (7u << clock_pin_base);
    }
    else{
        pin_mask = (1u << data_pin) | (3u << clock_pin_base);
    }
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, true);


    //dma init
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

    dma_channel_set_irq0_enabled(i2s_dma_chan, true);
    if (i2s_use_core1 == false){
        irq_set_exclusive_handler(DMA_IRQ_0, i2s_handler);
        irq_set_priority(DMA_IRQ_0, 0);
        irq_set_enabled(DMA_IRQ_0, true);
        i2s_handler();
    }

    //core1スタート
    if (i2s_use_core1 == true){
        multicore_launch_core1(core1_main_funcion);
    }
}

void i2s_mclk_change_clock(uint32_t audio_clock){
    //周波数変更
    if (i2s_low_jitter == false && i2s_mode == MODE_I2S){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 512);
        pio_sm_set_clkdiv(i2s_pio, i2s_sm, div);
    }
    else if (i2s_low_jitter == false && i2s_mode == MODE_PT8211){
        float div;
        div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 128);
        pio_sm_set_clkdiv(i2s_pio, i2s_sm, div);
    }
    else{
        //sys_clk変更
        if (audio_clock % 48000 == 0){
            if (i2s_overclock == true){
                set_sys_clock_295000khz();
            }
            else {
                set_sys_clock_147500khz();
            }
            clk_48khz = true;
        }
        else {
            if (i2s_overclock == true){
                set_sys_clock_271000khz();
            }
            else {
                set_sys_clock_135500khz();
            }
            clk_48khz = false;
        }

        //pio周波数変更
        uint dev;
        if (clk_48khz == true){
            dev = 12 * 192000 / audio_clock;
            if (i2s_overclock == false){
                dev /= 2;
            }
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
        }
        else {
            dev = 12 * 176400 / audio_clock;
            if (i2s_overclock == false){
                dev /= 2;
            }
            pio_sm_set_clkdiv_int_frac(i2s_pio, i2s_sm, dev, 0);
        }
    }
}

//i2sのバッファにusb受信データを積む
bool i2s_enqueue(uint8_t* in, int sample, uint8_t resolution){
    int i, j;
    static int32_t lch_buf[I2S_DATA_LEN / 2];
    static int32_t rch_buf[I2S_DATA_LEN / 2];

	if (i2s_get_buf_length() < I2S_BUF_DEPTH){
        if (resolution == 16){
            int16_t *d = (int16_t*)in;
            sample /= 2;
            for (i = 0; i < sample / 2; i++){
                lch_buf[i] = *d++ << 16;
                rch_buf[i] = *d++ << 16;
            }
        }
        else if (resolution == 24){
            uint8_t *d = in;
            int32_t e;
            sample /= 3;
            for (i = 0; i < sample / 2; i++){
                e = 0;
                e |= *d++ << 8;
                e |= *d++ << 16;
                e |= *d++ << 24;
                lch_buf[i] = e;
                e = 0;
                e |= *d++ << 8;
                e |= *d++ << 16;
                e |= *d++ << 24;
                rch_buf[i] = e;
            }
        }
        else if (resolution == 32){
            int32_t *d = (int32_t*)in;
            sample /= 4;
            for (i = 0; i < sample / 2; i++){
                lch_buf[i] = *d++;
                rch_buf[i] = *d++;
            }
        }

        //音量処理
        for (i = 0; i < sample / 2; i++){
            lch_buf[i] = (int32_t)(((int64_t)lch_buf[i] * mul_l) >> 29u);
            rch_buf[i] = (int32_t)(((int64_t)rch_buf[i] * mul_r) >> 29u);
        }

        //i2sバッファに格納
        j = 0;
        for (i = 0; i < sample / 2; i++){
            i2s_buf[enqueue_pos][j++] = lch_buf[i];
            i2s_buf[enqueue_pos][j++] = rch_buf[i];
        }
        i2s_sample[enqueue_pos] = sample;
		enqueue_pos++;
		if (enqueue_pos >= I2S_BUF_DEPTH){
            enqueue_pos = 0;
        }
        
        uint32_t save = spin_lock_blocking(queue_spin_lock);
        i2s_buf_length++;
        spin_unlock(queue_spin_lock, save);

		return true;
	}
	else return false;
}

bool i2s_dequeue(int32_t** buff, int* sample){
    if (i2s_get_buf_length()){
        *buff = i2s_buf[dequeue_pos];
        *sample = i2s_sample[dequeue_pos];

        dequeue_pos++;
        if (dequeue_pos >= I2S_BUF_DEPTH){
            dequeue_pos = 0;
        }

        uint32_t save = spin_lock_blocking(queue_spin_lock);
        i2s_buf_length--;
        spin_unlock(queue_spin_lock, save);

        return true;
    }
    else return false;
}

int8_t i2s_get_buf_length(void){
    int8_t d;

    uint32_t save = spin_lock_blocking(queue_spin_lock);
	d = i2s_buf_length;
    spin_unlock(queue_spin_lock, save);

    return d;
}

void i2s_volume_change(int16_t v, int8_t ch){
    if (ch == 0){
        mul_l = db_to_vol[-v >> 8];
        mul_r = db_to_vol[-v >> 8];
    }
    else if (ch == 1){
        mul_l = db_to_vol[-v >> 8];
    }
    else if (ch == 2){
        mul_r = db_to_vol[-v >> 8];
    }
}

void set_playback_handler(ExternalFunction func){
    playback_handler = func;
}

void set_core1_main_function(Core1MainFunction func){
    core1_main_funcion = func;
}
