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
- Adafruit ESP32-S3 Feather
- Adafruit RFM69HCW Transceiver Breakout (433MHz & 900MHz)
- Adafruit RFM69HCW FeatherWing Breakout (433MHz & 900MHz)
- Various RF devices across the 433MHz & 868MHz ranges

Resources
=========
[RFM69HCW Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69HCW-V1.1.pdf)


List of Examples
================
RFM69 Basics
- RFM69HCW internal version check using just SPI

RFM69 Radio Function Examples
- Continuous with/without Bit Sync (Comprehensive FSK + OOK)
- Packet Mode (Basic FSK over SPI-only)

Wiring diagram for Adafruit Feather or Pico W:
![RFM69HCW with Feather or Pico W](/rfm69hcw-wiring.png)

Lessons Learned
===============
If starting out, here are some tips:
- The version example can be used to sanity check the SPI / RFM69HCW.
- Use an RTL-SDR device to perform discovery / recon on the signals.
- Start wide on bandwidth, thresholds, deviation with the bit-synchronizer off.
- Switch on the AFC & AGC if there is intermittent reception.
- When ready to use the bit-synchronizer, ensure the bitrate is accurate.

Important lessons learned thus far (see the datasheet for references):
- In continous mode, data comes through DIO2 / G2, not DIO0 / G0 / IRQ (5.3.1).
- The RFM69HCW RST/reset should be held LOW throughout normal operation.
- The RFM69HCW RST/reset takes 6ms, not 5ms as declared in datasheet (7.2.2).
- Power-on and reset results in different register states (Footnote, pg 62).
- Default frequency deviation is +/-5kHz, bandwidth is 7-10kHz.
- FSK criticals: Freq. deviation, bandwidth, RSSI threshold, AFC & AGC.
- OOK criticals: OOK threshold type and OOK threshold level values.
- OOK: RegOOKFix threshold (0x1D) does not just apply to Fixed - also Peak.
- Bit synchronizer needs an precise bitrate; set to the shortest pulse length.
- rtl_433 and https://triq.org/pdv is invaluable for post-RTL-SDR analysis.
- FSK samples may be better analyzed via rtl_fm capture & Sigrok PulseView.
- Packet mode can be completely operated over SPI, accessing FIFO on 0x00.
- CRC-16 is probably true CCITT (not BUYPASS). 

RTL-SDR Discovery Examples
==========================
```
# Simple analysis:
$ rtl_433 -f 433.920M -R 0 -A

# Record and send FSK to Sigrok PulseView:
$ rtl_433 -f 433.535M -S all -T 3   
$ rtl_433 -s 250k -Y minmax -W test.sr -r g001_433.535M_250k.cu8

# Try to get rtl_433 to parse the data according to parameters:
$ rtl_433 -f 433.535M -R 0 \
  -X "name=capture001,modulation=FSK_PWM,short=64,long=136,sync=500,gap=200,reset=400"

# Output a FM-modulated waveformi for Audacity / Sigrok PulseView:
$ rtl_fm -f 867.476M -s 100000 -E dc -E edge -A fast -g 40 sample-867.800MHz_S16LE_Mono_100k.raw
```

TODO
====
- Transmission example(s) (change thermostat, operate a switch)
