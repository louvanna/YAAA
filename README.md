# YAAA
Yet Another (HE280) Accelerometer Adapter
This is a simple and cheap adapter that allows the SeeMeCNC HE280 accerometer probe to be used with the Duet controller. It may very well work for other controllers like Smoothie but I have not tested that yet.

The adapter is an Adafruit Trinket 3.3V - THIS IS IMPORTANT, the 5v version will not work with Duet.
I've made it easy to integrate to the existing Rostock MAX's Whip wiring harness. You do need a 4 position and 2 position female mating connector.
Digi-Key WM253-ND (2 pos) and WM2535-ND (4 pos) and at least 3 pins WM2565-ND
Of course, you can always cut and splice the wires too, but the connectors are convenient.
Read the source file header for all the details on how to setup the Arduino IDE, pin connections, etc. And the accompanying diagram shows the connections.
