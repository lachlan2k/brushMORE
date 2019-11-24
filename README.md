# brushMORE

brushMORE is a WIP Firmware for Brushless ESCs to control ordinary brushed dc motors.

Currently it is configured for use with the Afro Mini 20A, however, it should work with any ESC that runs 
afro_nfet.hex simonk firmware. I intend on making it easy to configure for other ESCs in the future.

**This hasn't been extensively tested so use at your own peril, but it probably should work, maybe**

# Features
 - Configurable deadband
 - Forward/Neutral/Reverse operation with standard 1000us-2000us PWM input.
 - Uses Timer1 ICP for reading PWM input with 16000 steps
 - Uses Timer2 for 8 bit 62.5khz PWM
 - Detects signal loss and input shorts
 - Configurable braking (NOT YET IMPLEMENTED)
 - Installs just like simonk