# Hyduino
## A hardware and software platform for automated monitoring of traditional and hydroponic gardening

The objective of this project, is to bring together a modular system of software and DIY hardware to comprehensively monitor every aspect of a garden according to your individual needs.

## Hardware used

Sensor controllers are some combination of ESP8266, Arduino Uno and Arduino Mega or equivalent. In completed form, a server run on an x86 or arm system then aggregates data and does statistical analysis to give the high level results to the use. The assumed tool will be a Raspberry Pi, but other hardware platforms should work.

This project is a work in progress. As of July 3, 2021 the only item present is code that should be able to run temperature, water, light and pH sensors as well as adjusting these using an Arduino device. The completed project will include modular Arduino sensors for use with an ad hoc sensor setup, as well as schematics for a circuit board to fit into the microcontroller development boards to connect sensors to handle the signal processing. Additionally, schematics for a totally integrated circuit board that contains the signal processing components and microcontroller will be included.

While I am intending to use my knowledge of server development, Arduino development, and chemical instrumental sensor analysis, I am intending to complete this on my own, however contributions on circuit design for signal processing PCBs would be very helpful.
