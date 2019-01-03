# temp-humidity-mikrobus
MikroBus sensor board for temperature and humidity.

Tested with:
 - [Microchip Explorer 16/32 Dev Kit](https://www.microchip.com/DevelopmentTools/ProductDetails/DM240001-2) using PIC24FJ1024GB610
 - Raspberry Pi 3 with [Pi 3 click shield](https://www.mikroe.com/pi-3-click-shield)
 
## Build

Raspberry Pi:

Install bcm2835 linux library

`cd raspberry; make`

Microchip Explorer 16/32:

 - Open MPLabX IDE
 - Open project `temp-humid.X`
 - Press Build

## PCB (compatible with mikroBUS™)
![](https://i.imgur.com/3snxZdm.jpg)
![](https://i.imgur.com/8KveNLd.jpg)
