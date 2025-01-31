#ifndef I2S_H
#define I2S_H

#define I2S_USE_CORE1

#define I2S_BUF_DEPTH   16
#define I2S_START_LEVEL     I2S_BUF_DEPTH / 4
#define I2S_TARGET_LEVEL    I2S_BUF_DEPTH / 2
#define I2S_DATA_LEN    (192 + 1) * 2

#define PIO_I2S         pio0
#define PIO_I2S_SM      0
#define DMA_I2S_CN      0
#define PIN_I2S_DATA    1
#define PIN_I2S_CLK     2

void i2s_mclk_init(uint32_t audio_clock);
void i2s_mclk_change_clock(uint32_t audio_clock);
void i2s_mclk_clock_set(uint32_t audio_clock);
void i2s_mclk_dma_init(void);
bool i2s_enqueue(uint8_t* in, int sample, uint8_t resolution);
void i2s_handler(void);
bool i2s_dequeue(int32_t** buff, int* sample);
int8_t i2s_get_buf_length(void);
void i2s_volume_change(int16_t v, int8_t ch);
void core1_main(void);

#endif