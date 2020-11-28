
# Nokia 5110 Display Driver for Arduino

**PCD8544** is a library for the [Arduino](http://arduino.cc/) to interface with LCDs based on the
Philips PCD8544 controller. These displays are commonly found on monochrome mobile phones, such as the
[Nokia 3310](http://en.wikipedia.org/wiki/Nokia_3310) or [5110](http://en.wikipedia.org/wiki/Nokia_5110). 

The library has a small memory footprint and is optimized a lot for fast refresh rates. 

A full description to set up your display can be found [here](https://www.xarg.org/2014/06/how-to-use-a-nokia-5110-graphical-display-with-arduino/).



# Bitmap support

You can easily generate images using our [LCD image generator](https://www.xarg.org/tools/lcd-image-generator/) and put them on the display:

![](https://www.xarg.org/image/nokia5110/nokia5110-cat.jpg)

# Geometry support

You can easily draw geometries like lines, rectangles or circles:

![](https://www.xarg.org/image/nokia5110/nokia5110-pyramid.jpg)

# Installation

Clone this repo directly into your Arduino libraries folder:

```
cd ~/Documents/Arduino/libraries
git clone https://github.com/infusion/PCD8544
```

Alternatively download this repo as a zip file and open it from the `Sketch > Include Library > Add .ZIP Library...` menu inside the Arduino IDE. 

Copyright and licensing
===
Copyright (c) 2014, [Robert Eisele](https://www.xarg.org/)
Dual licensed under the MIT or GPL Version 2 licenses.