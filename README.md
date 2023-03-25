RFM69HCW Examples
=================
This repository contains some examples of work with the RFM69HCW for Arduino:
- FSK & OOK modulation
- Continuous mode - with bit synchroniser off and on
- Packet mode - no variable length, address filtering or CRC checking

No 3rd-party RFM69 libraries are used, in order to aid understanding and troubleshooting when experimenting.

Tested with the following:
- Raspberry Pi Pico W
- iLabs Challenger 2040 WiFi
- Adafruit ESP32-S3 Feather (with/without TFT)
- Adafruit RFM69HCW Transceiver Breakout (433MHz & 900MHz)
- Adafruit RFM69HCW FeatherWing (433MHz & 900MHz)
- Various RF devices across the 433MHz & 868MHz ranges

Resources
=========
[RFM69HCW Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69HCW-V1.1.pdf)

List of Examples
================
RFM69 Basics
- RFM69HCW internal version check using just SPI

Generic Examples
- Receiving
    - Continuous with/without Bit Sync (Comprehensive FSK + OOK)
    - Packet Mode (Basic FSK over SPI-only)
- Transmitting
    - Continuous mode (OOK, no shaping, -23dBm)

Device Specific Examples
- Maverick ET-731 BBQ Thermometer (Rx only)
    - Continuous Mode
    - OOK
    - Manchester Encoding
- RSL366 Generic Power Switch (Tx & Rx)
    - Continuous Mode
    - OOK 
    - Bit Positioning
- Mitsubishi Ecodan (Tx & Rx)
    - Packet Mode (Variable, faked CRC)
    - FSK, Gaussian
    - Manual CRC-8 & CRC-16 checking

Wiring diagram for the examples Adafruit Feather or Pico W:
![RFM69HCW with Feather or Pico W](/rfm69hcw-wiring.png)

Lessons Learned
===============
If starting out, here are some tips:
- The version example can be used to sanity check the SPI / RFM69HCW.
- Close the SPI - it will affect interrupts, multi-core
- Start wide on bandwidth, thresholds, deviation with the bit-synchronizer off.
- When ready to use the bit-synchronizer, ensure the bitrate is accurate.
- Switch on the AFC & AGC if there is intermittent reception.
- RTL-SDR for discovery:
  - rtl_433 sample rate is between 225k-300k & 900k-3200k.
  - rtl_433's "-S all" will do data capture, -A will do analysis 
  - Employ "-Y minmax" for FSK in rtl_433
  - GQRX / Triq.org / Sigrok Pulseview are really handy for signal analysis

Important lessons learned thus far (see the datasheet for references):
- Initialisation
  - RST/reset should be held LOW throughout normal operation.
  - RST/reset takes 6ms, not 5ms as declared in datasheet (7.2.2).
  - Power-on and reset results in different register states (Footnote, pg 62).
  - Default frequency deviation is +/-5kHz, bandwidth is 7-10kHz.
- Tuning
  - FSK criticals: Freq. deviation, bandwidth, RSSI threshold, AFC & AGC.
  - OOK criticals: OOK threshold type and OOK threshold level values.
  - OOK: RegOOKFix threshold (0x1D) does not just apply to Fixed - also Peak.
  - Bit synchronizer needs an precise bitrate; set to the shortest pulse length.
- Continuous (raw) mode:
  - Transmit via DIO2  / G2 (no DCLK needed unless using gaussian shaping).
  - Receive via DIO2 / G2 (5.3.1).
- Packet mode:
  - Can be completely operated over SPI, accessing FIFO on 0x00.
  - CRC-16 is probably true CCITT (not BUYPASS). 
  - Payload sizing (Section 5.5.2):
    - Fixed: 0x37=0x00, 0x38=size-of-packet
    - Variable: 0x37=0x80, First FIFO byte is size
    - Unlimited: 0x37=0x00, 0x38=0x00
  - Burst mode is mandatory for writing packet data to the FIFO
  - Burst mode holds NSS/CS LOW for entire SPI transfer to write beyond byte 1

RTL-SDR Discovery Examples
==========================
```
# Simple analysis of pulses if OOK:
$ rtl_433 -f 433.920M -R 0 -A

# Record and send FSK to Sigrok PulseView:
$ rtl_433 -f 433.535M -S all -T 3   
$ rtl_433 -s 250k -Y minmax -W test.sr -r g001_433.535M_250k.cu8

# Try to get rtl_433 to parse the data according to parameters:
$ rtl_433 -f 433.535M -Y minmax \
  -X "name=xyz,modulation=FSK_PWM,short=64,long=136,sync=500,gap=200,reset=400"

# Output a FM-modulated waveform for Audacity / Sigrok PulseView:
$ rtl_fm -f 868.3M -s 384k -E dc -E edge -A fast -g 40 867.8M_S16LE_Mono.raw
```
