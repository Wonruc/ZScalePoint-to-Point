# ZScalePoint-to-Point
Arduino Nano/Uno/Mega L298 based DC Point-To-Point Z Scale Automated Trolley Control

This is a project I started because Showcase Miniatures made a $35 trolley kit that goes over a $20 shorty mechanism.

You can see how I developed and built this project at:
https://www.trainboard.com/highball/index.php?threads/the-arduino-z-scale-trolley-project.151045/

If you would like to build your own you will need:

* Arduino Nano, Uno or Mega. 

* Arduino L298p or L298n motor shield or equivalent like the Deeks Robot shield. That Alice account on the bay sells Nano shilds.

* IR sensors with 3 pins ( I used some from HiLetGO)

* A 10 ohm potentiometer

* A 470 ohm resistor

* Various wires to connect it all. Give strong consideration to a servo wire/jst crimp kit where you can make your own 3 pin wires.

There are also some optional parts you may want to consider.

- A vibration motor can be connected to pin D2 to correct stalls on small layouts. I used one of the 3 pin modules.

- An 128 x 64 OLED display can be added.  I used a white HiLetgo screen. The screens with a color band at the top could be used.
  If you use a OLED display you will need to install the Adafruit GFX library, Adafruit SSD1306 library, and likely Adafruit BusIO library
