# Smart Blind Stick

A low-cost, Arduino Nano based assistive cane for visually impaired users.
It adds floor-level obstacle, surface-water, and ambient-light sensing with
private vibration feedback, a 433 MHz "Find-My-Stick" locator, and a
one-button GPS/GSM emergency SOS that texts a caretaker a Google Maps link
— all without requiring the user to carry a smartphone.

## 

## Features

|Feature|How it works|
|-|-|
|Floor-level obstacle detection|HC-SR04 ultrasonic, 50 cm threshold, mounted low and facing forward|
|Surface water detection|Contact moisture probe at the cane tip|
|Ambient light / darkness detection|LDR in a voltage divider|
|Private hazard feedback|Distinct vibration pattern per hazard (fast pulses / slow-long / double-long) — no buzzer, no voice|
|Power-on confirmation|One short vibration pulse on power-up confirms the system is active|
|Cane retrieval|433 MHz RF remote; five-beep buzzer on the cane|
|Emergency SOS|Push-button → GPS fix → SMS with a Google Maps link, via SIM800L|
|Alert priority|Obstacle > Water > Light, resolved in firmware so hazards never overlap|
|Power|Two rechargeable 3.7 V Li-ion cells (parallel) → boost converter → 5 V rail|

## 

## Hardware Overview

* **Controller:** Arduino Nano (ATmega328P, 16 MHz, 2 KB SRAM)
* **Sensors:** HC-SR04 ultrasonic, contact moisture probe, LDR
* **Inputs:** 433 MHz RF receiver, SOS push-button
* **Outputs:** vibration motor, indicator LEDs, locator buzzer, SIM800L GSM modem
* **Location:** external GPS receiver (UART) — separate from the SIM800L, which handles SMS only
* **Power:** two 3.7 V Li-ion cells in parallel, boost-regulated to 5 V, charged over USB via a protection/charging IC; \~3 days reported use per charge (not yet instrumented — see paper Table IX)

## 

## Firmware

The main loop reads all sensors each cycle and resolves the highest-priority
active hazard using:

```
Obstacle (< 50 cm)  >  Water (debounced)  >  Light (below threshold)
```

SOS and the RF locator are handled independently of this priority chain.
See `firmware/smart\_blind\_stick.ino` (or your actual filename) for the
implementation, and Section VIII of the paper for the full decision logic
and flowchart.

**Known firmware caveat:** the independence of SOS/RF handling from the
hazard patterns and the GPS/SMS sequence only holds if these are implemented
with non-blocking timing (`millis()`-based state machines) rather than
`delay()`. Confirm this in the actual code before relying on it.

## 

## Building / Flashing

1. Open in the Arduino IDE.
2. Install any required libraries (list them here once finalized — e.g. a
SIM800L/GSM library, a GPS/NMEA parsing library).
3. Select **Arduino Nano** as the board and the correct COM port.
4. Upload.

## 

## Validation Status

All six functional tests (obstacle, water, light, RF locator, GPS SOS, and
full integration) passed as pass/fail checks — see Section X of the paper.
**Not yet measured:** detection accuracy vs. distance/material, alert
latency, RF range, GPS time-to-fix, SMS success rate, or instrumented
battery runtime. See the paper's Table IX for the planned test campaign
before citing any of these as numbers.

## 

## Limitations

* Detects floor-level obstacles only (chest/head-height obstacles are out of scope)
* Water sensing is contact-only at the tip
* GPS performance degrades indoors; SOS SMS does not currently flag a stale/fallback fix as such
* Charging is manual via USB; no charging dock yet (planned — see paper Table VIII)
* No clinical, regulatory, or user-study validation

## 

## Authors

* H. T. Thushan 
* M. D. M. Wijesinghe 
* A. K. N. Tharushika 

Department of Computer Engineering, University of Sri Jayewardenepura, Sri Lanka.

## 

## License

Add a license here MIT for firmware,

