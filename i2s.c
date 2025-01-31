#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "i2s.pio.h"
#include "i2s.h"

#ifdef I2S_USE_CORE1
#define SPINLOCK_ID_AUDIO_QUEUE (16 + 0)
static spin_lock_t* queue_spin_lock;
#endif

static int8_t i2s_buf_length;
static uint8_t enqueue_pos;
static uint8_t dequeue_pos;

static int32_t i2s_buf[I2S_BUF_DEPTH][I2S_DATA_LEN];
static uint32_t i2s_sample[I2S_BUF_DEPTH];

static int32_t mul_l;
static int32_t mul_r;
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

void i2s_mclk_init(uint32_t audio_clock){
    pio_sm_config sm_config;
    PIO pio = PIO_I2S;
    uint sm = PIO_I2S_SM;
    uint data_pin = PIN_I2S_DATA;
    uint clock_pin_base = PIN_I2S_CLK;
    uint func = GPIO_FUNC_PIO0;
    uint offset;
    float div;

    gpio_set_function(data_pin, func);
    gpio_set_function(clock_pin_base, func);
    gpio_set_function(clock_pin_base + 1, func);
    gpio_set_function(clock_pin_base + 2, func);

    offset = pio_add_program(pio, &i2s_mclk_256_program);
    sm_config = i2s_mclk_256_program_get_default_config(offset);
    
    sm_config_set_out_pins(&sm_config, data_pin, 1);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_out_shift(&sm_config, false, false, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    sm_config_set_set_pins(&sm_config, data_pin, 1);

    #ifdef I2S_USE_CORE1
    queue_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_QUEUE);
    #endif
    i2s_buf_length = 0;
    enqueue_pos = 0;
    dequeue_pos = 0;

    div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 512);
    sm_config_set_clkdiv(&sm_config, div);

    pio_sm_init(pio, sm, offset, &sm_config);

    uint pin_mask = (1u << data_pin) | (7u << clock_pin_base);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pins(pio, sm, 1);

    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
}

void i2s_mclk_change_clock(uint32_t audio_clock){
    #ifndef I2S_USE_CORE1
    irq_set_enabled(DMA_IRQ_0, false);
    #endif

    i2s_buf_length = 0;
    enqueue_pos = 0;
    dequeue_pos = 0;

    i2s_mclk_clock_set(audio_clock);
    
    #ifndef I2S_USE_CORE1
    irq_set_enabled(DMA_IRQ_0, true);
    #endif
}

void i2s_mclk_clock_set(uint32_t audio_clock){
    //MCLK * 512
    float div;
    div = (float)clock_get_hz(clk_sys) / (float)(audio_clock * 512);
    pio_sm_set_clkdiv(PIO_I2S, PIO_I2S_SM, div);
}

void i2s_mclk_dma_init(void){
    uint ch = DMA_I2S_CN;
    PIO pio = PIO_I2S;
    uint sm = PIO_I2S_SM;

    dma_channel_config conf = dma_channel_get_default_config(ch);
    
    channel_config_set_read_increment(&conf, true);
    channel_config_set_write_increment(&conf, false);
    channel_config_set_transfer_data_size(&conf, DMA_SIZE_32);
    channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));
    
    dma_channel_configure(
        ch,
        &conf,
        &PIO_I2S->txf[PIO_I2S_SM],
        NULL,
        0,
        false
    );

    dma_channel_set_irq0_enabled(ch, true);
    #ifndef I2S_USE_CORE1
	irq_set_exclusive_handler(DMA_IRQ_0, i2s_handler);
    irq_set_priority(DMA_IRQ_0, 0);
    irq_set_enabled(DMA_IRQ_0, true);
    i2s_handler();
    #endif
}


//i2sのバッファにusb受信データを積む
bool i2s_enqueue(uint8_t* in, int sample, uint8_t resolution){
    int i, j;
    int32_t lch_buf[I2S_DATA_LEN / 2];
    int32_t rch_buf[I2S_DATA_LEN / 2];

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
		if (enqueue_pos >= I2S_BUF_DEPTH) enqueue_pos = 0;
        
        #ifdef I2S_USE_CORE1
        uint32_t save = spin_lock_blocking(queue_spin_lock);
        #else
        irq_set_enabled(DMA_IRQ_0, false);
		#endif
        i2s_buf_length++;
        #ifdef I2S_USE_CORE1
        spin_unlock(queue_spin_lock, save);
        #else
        irq_set_enabled(DMA_IRQ_0, true);
        #endif
		return true;
	}
	else return false;
}

#ifndef I2S_USE_CORE1
void __isr __time_critical_func(i2s_handler)(){
	static bool mute;
	static int32_t mute_buff[96 * 2] = {0};
	static uint32_t mute_len = sizeof(mute_buff) / sizeof(int32_t);
	
	if (i2s_buf_length == 0){
        mute = true;
    }
	else if (i2s_buf_length >= I2S_START_LEVEL && mute == true){
        mute = false;
    }

	if (!mute){
		dma_channel_transfer_from_buffer_now(DMA_I2S_CN, i2s_buf[dequeue_pos], i2s_sample[dequeue_pos]);

		dequeue_pos++;
		if (dequeue_pos >= I2S_BUF_DEPTH) dequeue_pos = 0;
		
		i2s_buf_length--;
	}
	else{
		dma_channel_transfer_from_buffer_now(DMA_I2S_CN, mute_buff, mute_len);
	}
    
   	dma_hw->ints0 = 1u << DMA_I2S_CN;
}
#endif

#ifdef I2S_USE_CORE1
bool i2s_dequeue(int32_t** buff, int* sample){
    if (i2s_get_buf_length()){
        *buff = i2s_buf[dequeue_pos];
        *sample = i2s_sample[dequeue_pos];

        uint32_t save = spin_lock_blocking(queue_spin_lock);
        dequeue_pos++;
        if (dequeue_pos >= I2S_BUF_DEPTH) dequeue_pos = 0;
        i2s_buf_length--;
        spin_unlock(queue_spin_lock, save);

        return true;
    }
    else return false;
}
#endif

int8_t i2s_get_buf_length(void){
    int8_t d;
    #ifdef I2S_USE_CORE1
    uint32_t save = spin_lock_blocking(queue_spin_lock);
    #else
    irq_set_enabled(DMA_IRQ_0, false);
	#endif
	d = i2s_buf_length;
    #ifdef I2S_USE_CORE1
    spin_unlock(queue_spin_lock, save);
    #else
    irq_set_enabled(DMA_IRQ_0, true);
    #endif
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

#ifdef I2S_USE_CORE1
void core1_main(void){
    int i;
    int32_t* buff;
    int dma_sample[2], sample;
    bool mute = false;
    int32_t mute_buff[96 * 2] = {0};
    uint32_t mute_len = sizeof(mute_buff) / sizeof(int32_t);
    int8_t buf_length;
    int32_t dma_buff[I2S_BUF_DEPTH][I2S_DATA_LEN];
    uint8_t dma_use = 0;

    while (1){
        buf_length = i2s_get_buf_length();

        if (buf_length == 0){
            mute = true;
        }
        else if (buf_length >= I2S_START_LEVEL && mute == true){
            mute = false;
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
        for (i = 0; i < sample; i++){
            dma_buff[dma_use][i] = buff[i];
        }
        dma_sample[dma_use] = sample;

        dma_channel_wait_for_finish_blocking(DMA_I2S_CN);
        dma_channel_transfer_from_buffer_now(DMA_I2S_CN, dma_buff[dma_use], dma_sample[dma_use]);
        dma_use ^= 1;
    }
}
#endif
