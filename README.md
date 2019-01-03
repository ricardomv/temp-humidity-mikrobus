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

## Components

 - [MCP3201 - 12-Bit A/D Converter with SPI™](https://www.microchip.com/wwwproducts/en/MCP3201)
 - [HIH-5030 - Humidity Sensor](https://sensing.honeywell.com/sensors/humidity-sensors/HIH-5030-5031-series)
 - [AD592 - Current Output – Precision IC Temperature Transducer](https://www.analog.com/en/products/ad592.html)
 - [MCP1501 - Voltage reference](https://www.microchip.com/wwwproducts/en/MCP1501)
 - [1040310 - microSD Memory Card Connector](https://www.molex.com/molex/products/datasheet.jsp?part=active/1040310811_MEMORY_CARD_SOCKET.xml&channel=Products&Lang=en-US)

## PCB (compatible with mikroBUS™)
![](https://i.imgur.com/3snxZdm.jpg)
![](https://i.imgur.com/8KveNLd.jpg)
