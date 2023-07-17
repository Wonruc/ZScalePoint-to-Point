# ZScalePoint-to-Point
Arduino Uno / Nano L298 based DC Point-To-Point Z Scale Automated Trolley Control

This is a project I started because Showcase Miniatures made a $35 trolley kit that goes over a $20 shorty mechanism.

If you would like to build your own you will need:

* Arduino Uno or Nano. 
(A Mega would need slight modification to the PWMFreq() function as pin D3’s PWM is on Timer 3 not Timer2)

* Arduino L298p or L298n motor shield or equivalent like the Deeks Robot shield. That Alice account on the bay sells Nano shilds.

* IR sensors with 3 pins ( I used some from HiLetGO)

* A 10 ohm potentiometer

* A 470 ohm resistor

* Various wires to connect it all. Give strong consideration to a servo wire/jst crimp kit where you can make your own 3 pin wires.

The code currently has alot of commented out code for various uses.  I may split this off soon into several variants that would just work without editing.  
