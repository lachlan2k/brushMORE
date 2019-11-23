# brushMORE

brushMORE is a WIP Firmware for Brushless ESCs to control ordinary brushed dc motors.

Currently it is configured for use only for the Afro Mini 20A, however, it should work with any ESC that runs 
afro_nfet.hex simonk firmware. I intend on making it more configurable for other ESCs in the future.

**As of right now, it is not yet functional and only controls PWM on the LEDs, not the fets.**

# Features
 - Configurable deadband
 - Forward/Neutral/Reverse operation with standard 1000us-2000us PWM input.
 - Uses Timer2 for 8 bit 62.5khz PWM
 - Uses Timer1 ICP for 
 - Detects signal loss and input shorts
 - Configurable braking (NOT YET IMPLEMENTED)
 - Installs just like simonk