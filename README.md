# LED Icosahedron
![Icosahedron](http://www.4k2.de/ikosaeder/img/ikosaeder_final.jpg "LED Icosahedron")

This is the Arduino sketch for my first LED Icosahedron.
The LED part is explained on my [homepage](http://www.4k2.de/ikosaeder).

The sources are hosted on
https://github.com/jensh/ArduinoLedIkosaeder/ .

Variants of the icosahedron are in the other branches:
 * [icosaeder with smd LEDs](https://github.com/jensh/ArduinoLedIkosaeder/tree/ikosaeder2)

# Schematic
Connect the 7 LED wires (via 220Ω) with the output pins D8, D9, D2, D3, D4 and D5.
Pin D7 is unused in this sketch.

Using the auto-off powercontrol with DC-DC converter is optional:
![PCB](fritzing/Ikosaeder_bb.png)
![Schematic](fritzing/Ikosaeder_schem.png)
