# stairway-light-automation-team-ok
The Python program was developed to integrate multiple sensors (PIR, LDR, Sound, ToF) with the NeoPixel LED strip and the Blynk Cloud platform.

The code follows a modular structure, consisting of initialization, sensor reading functions, decision logic, and LED control routines. It operates in an infinite loop, constantly reading sensor inputs and updating the LED brightness accordingly.

Each sensor was connected to the Raspberry Pi GPIO pins. The PIR sensor provides a digital HIGH signal upon detecting motion. The LDR (via the PCF8591 ADC) returns an analog value representing ambient light intensity. The sound sensor (also through PCF8591) captures sound amplitude, while the VL53L1X ToF sensor measures object proximity in millimeters. 

The control logic corresponds directly to the operational flowchart. When any motion, sound, or distance condition is satisfied, the system checks the LDR value to decide between “day” and “night” modes. 

If daylight is detected → NeoPixel LEDs are set to dim brightness (≈20%). 
If night is detected → LEDs are set to bright color (≈100%). 

After a predefined timeout (e.g., 5 s), the LEDs automatically turn off to conserve power 

The system transmits instantaneous power (W), daily energy usage, Energy Saving, Cost saving and cumulative energy (Wh) data to the Blynk Cloud via virtual pins V7, V20, V22, V24 and V6. This allows remote monitoring of daily energy usage.
