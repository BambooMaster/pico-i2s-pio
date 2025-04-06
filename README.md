# これなに
Raspberry Pi Picoのpioを使ってMCLK対応のi2sを出力するライブラリです。sys_pllを変化させpioのフラクショナル分周を使わずにi2sを出力するlowジッタモードを搭載しています。

# 対応フォーマット
16,24,32bit 44.1kHz～384kHz (BCLK=64fs)

# MCLKについて
MCLKはlowジッタモードがオンの場合は22.581/24.571MHzで固定、lowジッタモードオフの場合は256fs可変です。

# i2sのピン
|name|pin|
|----|---|
|DATA|GPIO18|
|LRCLK|GPIO19|
|BCLK|GPIO20|
|MCLK|GPIO21|
