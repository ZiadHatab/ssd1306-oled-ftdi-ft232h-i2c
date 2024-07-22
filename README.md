# SSD1306 OLED with FTDI FT232H

Controlling SSD1306 OLED display over I2C with FTDI FT232H USB bridge.

*__NOTE:__* _The GIF images I used are converted and resized from [Telepurte](https://twitter.com/Telepeturtle) videos._

## Code Requirements

You need to install [FTDI driver](https://ftdichip.com/drivers/d2xx-drivers/). The code uses [ftd2xx wrapper](https://github.com/ftd2xx/ftd2xx) for communicating with the FTDI chip. For processing GIF images, you need [pillow](https://github.com/python-pillow/Pillow) and [numpy](https://github.com/numpy/numpy).

```cmd
python -m pip install ftd2xx
python -m pip install pillow
python -m pip install numpy
```

__NOTE:__ I'm using 128x64 display size. You can use other display sizes, e.g., 128x32, but then you need to crop your image to correct size and change some related parameters in the script.

## Circuit Layout

For I2C you need to connect D1 and D2 together (this is done internally through code), which then connects to SCL. D0 is the clock and connects to SDA. The SSD1306 display already uses pull-up resistors. VCC is connected to 5V from the FTDI breadboard.

![](./Images/layout.png)
_Wiring diagram._