# AERO SELFIE 30A

## 01 / PRODUCT OVERVIEW

The AERO SELFIE 30A is a compact four-in-one brushless ESC driven by an 8-bit high-speed MCU, delivering 30 A continuous and 40 A burst (30 s) on each of its four channels across a 6-18 V (2S-4S) input range. High-frequency PWM with hardware synchronous rectification and regenerative braking gives excellent throttle linearity, faster deceleration, and higher battery energy-recovery efficiency. PWM, DShot, and OneShot drive protocols, a current-sense output, and onboard stall protection round out a power stage sized for sub-300 mm multirotors. The integrated 28 x 30 x 5.4 mm board mounts on a 20 x 20 mm M3 pattern, weighs roughly 5.6 g, and cools by natural convection.

## 02 / SPECIFICATIONS

| Parameter | Value |
|---|---|
| Model | 4in1-ESC-30A |
| Type | 4-in-1 Brushless ESC |
| Input Voltage | 6-18 V (2S-4S) |
| Continuous Current | 30 A × 4 |
| Burst Current | 40 A × 4 (30 s) |
| Drive Protocol | PWM - DShot - OneShot |
| MCU | 8-bit |
| BEC Output | None |
| Stall Protection | Yes |
| Cooling | Natural convection |
| Dimensions | 28 x 30 x 5.4 mm |
| Mounting | M3 x 4 - 20 x 20 mm |
| Weight | ≈ 5.6 g |
| Operating Temp. | -10°C - 55°C |
| HW/SW Version | 1.0 / 1.0 |

## 03 / BOARD LAYOUT

![Board layout top and bottom](image-placeholder)

![Board layout bottom detail](image-placeholder)

## 04 / USAGE INSTRUCTIONS

**Throttle signal: (PWM as example)**

- Initialization signal range: 800-1050 us
- Start signal range: 1050-2000 us

**Motor tones:**

- Initialization complete: beep - beep - beep, low - medium - high, three tones
- No throttle signal input: one beep every three seconds
- Stall-protection reset: beep - beep, medium - low, two tones

## 05 / MECHANICAL DIMENSIONS

Outline dimensions: L x W x H = 28 x 30 x 5.4 mm (excluding electrolytic capacitor), tolerance ± 0.5 mm, see figure below.

![Mechanical dimensions](image-placeholder)

## 06 / PIN ASSIGNMENT

| Pin | Name | Function |
|---|---|---|
| + | Battery voltage positive | Power input |
| - | Battery voltage negative | Power input |
| 1A | OUT A | Motor 1 output A |
| 1B | OUT B | Motor 1 output B |
| 1C | OUT C | Motor 1 output C |
| 2A | OUT A | Motor 2 output A |
| 2B | OUT B | Motor 2 output B |
| 2C | OUT C | Motor 2 output C |
| 3A | OUT A | Motor 3 output A |
| 3B | OUT B | Motor 3 output B |
| 3C | OUT C | Motor 3 output C |
| 4A | OUT A | Motor 4 output A |
| 4B | OUT B | Motor 4 output B |
| 4C | OUT C | Motor 4 output C |
| - | Battery voltage negative | Power output |
| + | Battery voltage positive | Power output |
| 1 | Signal | ESC 1 throttle control signal |
| 2 | Signal | ESC 2 throttle control signal |
| 3 | Signal | ESC 3 throttle control signal |
| 4 | Signal | ESC 4 throttle control signal |
| NN | NC | No output |
| C | Current | Current signal ADC |

---

Thank you for choosing AERO SELFIE product. Please read this disclaimer carefully — by using this product, you acknowledge and agree to the following: brushless power systems can be dangerous, and improper use may result in personal injury or equipment damage. Follow the installation and operation instructions carefully. Because we cannot control how this product is used, installed, or maintained, we accept no liability for any loss or damage arising from its use. Any unauthorized modifications are at your own risk.

Read this manual to understand all power configuration and flight parameters before powering the system. Installation requires soldering and wiring. Make sure all wires are properly insulated before connection to avoid short circuits. Use a soldering iron with adequate power to ensure solid joints — poor connections can cause loss of control or damage to your equipment.