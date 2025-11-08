# pico-i2s-pio
Raspberry Pi Picoのpioを使ってMCLK対応のi2sを出力するライブラリです。RP2040/RP2350のシステムクロックをMCLKの整数倍に設定し、pioのフラクショナル分周を使わないlowジッタモードを搭載しています。また、PCM5102AやPT8211のような差動出力非対応のDACをデュアルモノで動作させる機能を搭載しています。i2sのslaveモードにも対応しました。

## 対応フォーマット
16,24,32bit 44.1kHz～384kHz
### i2s
BCLK: 64fs  
MCLK: 22.5792/24.576MHz  

|name|pin|
|----|---|
|DATA|data_pin|
|LRCLK|clock_pin_base|
|BCLK|clock_pin_base+1|
|MCLK|mclk_pin|

### PT8211
BCLK: 32fs  
MCLK: no  

|name|pin|
|----|---|
|DATA|data_pin|
|LRCLK|clock_pin_base|
|BCLK|clock_pin_base+1|

### AK449X EXDF (試験的機能)
BCK: 32fs  
MCLK: 32fs (BCK)  

|name|pin|
|----|---|
|DOUTL|data_pin|
|DOUTR|data_pin+1|
|WCK|clock_pin_base|
|BCK|clock_pin_base+1|
|MCLK|clock_pin_base+2|

### i2s dual
BCLK: 64fs  
MCLK: 22.5792/24.576MHz  
各DACのLがポジティブ、Rがネガティブとなります。  

|name|pin|
|----|---|
|DATAL|data_pin|
|DATAR|data_pin + 1|
|LRCLK|clock_pin_base|
|BCLK|clock_pin_base+1|
|MCLK|mclk_pin|

### PT8211 dual
BCLK: 32fs  
MCLK: no  
各DACのLがポジティブ、Rがネガティブとなります。  

|name|pin|
|----|---|
|DATAL|data_pin|
|DATAR|data_pin + 1|
|LRCLK|clock_pin_base|
|BCLK|clock_pin_base+1|

### i2s slave (試験的機能)
BCLK: 64fs  
MCLK: no  

|name|pin|
|----|---|
|DATA|data_pin|
|LRCLK|clock_pin_base|
|BCLK|clock_pin_base+1|

## クロックについて
MCLKは22.5792/24.576MHzで固定です。EXDFモードの場合は、BCKと同じクロックが出力されます。  
lowジッタモードでは、フラクショナル分周を使用した場合と比べて周波数誤差が大きくなる可能性があります。  
外部クロック機能を使用する場合は、GPIO20に45.1584MHzを、GPIO22に49.152MHzを入力してください。(試験的機能)  
slaveモードを使用する場合は、LRCLKとBCLKに同期したクロックを入力してください。  

## デフォルト
- 出力フォーマット: i2s
- lowジッタモード: off

|name|pin|
|----|---|
|DATA|GPIO18|
|LRCLK|GPIO20|
|BCLK|GPIO21|
|MCLK|GPIO22|
