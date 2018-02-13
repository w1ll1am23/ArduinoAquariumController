# ArduinoAquariumController

This is a person project for controllering my Turtle tank via an Arduino Mega plus Ethernet shield.
This was an update to a previous project using a Raspberry Pi v2 which stopped functioning due to a corrupt SD card :(

The requirements for this project were as follows.

1. No updates. i.e. No OS so Raspberry Pi was not an option.
2. Network. (So I could get the status via Home Assistant)
3. RTC. (Works even if the network is down) Don't want my turtle to die because the Internet isn't working...
4. 8 relay control for standard 120v outlets
5. 3 temperature sensors (Water, basking area, internal controller temperature)
6. water leak sensor (If my canister filter is leaking turn it off)
7. water level sensor (One day I would like to auto fill the tank)
8. Current sensor (Amps in use.)
