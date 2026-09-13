# Audio DAQ anti-alias filter and DDS research

Research date: 2026-09-13. Status: **research archive; no purchase approved or hardware combination qualified**.

## Decision at the end of the session

The user favors the assembled **LTC1068 low-pass module** as the next candidate to investigate. The **M5Stack DDS Unit U105** remains the preferred sine source. Do not rush a purchase: documentation, power, control, ADC interfacing, and total accessory cost must be resolved first.

This note records both useful candidates and rejected paths so the next session does not repeat the same searches. Prices are session observations, not quotations; stock, shipping, and listings can change. Seller descriptions and photographs establish only what the seller presents, not measured performance or genuine component provenance. Earlier-session findings below were not all independently rechecked during archival.

## Requirements and scope

- One audio-frequency input for bench work, including PP1 phono preamp characterization and later experiments. Approximately 20 Hz–20 kHz is the working audio-band assumption, not a finalized passband specification.
- Two channels were initially contemplated, then explicitly deferred: sharing ADC throughput can reduce the per-channel rate, and the user does not want to interrupt current USB throughput and GUI optimization.
- Fully assembled, **no soldering**, including fitted headers/connectors and practical power/signal adapters. SMA is attractive; screw terminals and headers are acceptable. BoosterPack compatibility would be convenient but is not required.
- Preferred budget: **under $100 for DDS, AAF, and necessary new accessories**. This supersedes the earlier under-$250 filter/accessory target.
- Firmware-controlled DDS desired; firmware-controlled AAF is a bonus, not essential. A fixed or manually adjusted filter is acceptable.
- Generic overseas Amazon/eBay-style boards are acceptable research candidates. Expensive laboratory instruments and empty evaluation PCBs do not meet the intent.
- No firmware, GUI, or hardware changes were made for this research.

## Existing equipment and interface boundaries

The repository describes an EK-TM4C123GXL / TM4C123GH6PM with ADC0 AIN0 on PE3. PB6 PWM is the existing square-wave test source. The desired DDS adds sine-wave stimulus.

Repository notes describe ADC presets from 100 to 400 kS/s and a safe continuous point near 333 kS/s, but also contain older contradictory streaming statements. These are repository-reported observations, not measurements from this research. Confirm current firmware and achieved acquisition rate before selecting the AAF; do not resolve throughput documentation inconsistencies as part of this archive.

The user owns an **ISL8203MEVAL2Z Rev B1** power board. It may be useful for positive rails such as 3.3 V. Do not assume it supplies a negative rail. The M5Stack DDS requires **5 V**, correcting an earlier 3.3 V assumption. A specific complete power wiring arrangement has not been approved or verified.

A filter output cannot automatically connect safely or accurately to PE3. The final chain must establish input attenuation, bias, output swing, ADC drive/settling, and protection. A nominal 3.3 V ADC supply suggests an approximately 1.65 V signal midpoint, but actual reference/supply and allowed range must be verified. A bipolar ±5 V filter output or a 2.5 V-centered output is not automatically compatible.

Conceptual chain:

```text
DDS -> device under test -> input scaling / analog filtering / bias and ADC driver -> PE3
```

The order and circuit implementation of conditioning remain open. The AAF protects the ADC acquisition path; a DDS reconstruction filter serves a different purpose.

## Leading assembled filter candidates

### LTC1068 low-pass board — preferred next investigation

[Amazon listing, B0CB8JMBSJ](https://www.amazon.com/dp/B0CB8JMBSJ), brand Senzooe, seller shown as ShenYanshu. Observed **$49.04 + $5.70 shipping = $54.74**, with low-pass and band-pass variants.

Enlarged product photographs show fitted SMA connectors labeled **S_IN**, **S_OUT**, and **S_CLK**. A fitted three-pin power header appears marked **+5V / GND / -5V**. This supports a mechanically solder-free approach but does not prove a complete compatible setup.

[ADI LTC1068 product documentation](https://www.analog.com/en/products/ltc1068.html) and [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/1068fc.pdf) establish:

- Four second-order switched-capacitor sections; external configuration can produce quad second-order, dual fourth-order, or eighth-order responses.
- Cutoff/section frequency follows an external clock. A suitable controller timer could tune it without a multi-bit filter register interface.
- Family clock-to-center-frequency ratios differ: LTC1068-200 = 200:1; LTC1068 = 100:1; LTC1068-50 = 50:1; LTC1068-25 = 25:1. External resistors can modify the relationship.
- Chip-level supply options include single 3.3 V, single 5 V, and ±5 V, with performance limits dependent on variant, supply, and configuration. This does **not** establish that the Amazon board supports all those supplies.

Unresolved: exact chip suffix, schematic, actual filter order and response, cutoff-to-clock relationship, useful cutoff range, input impedance, gain, clock logic levels, noise/distortion, output bias/swing, and clock-feedthrough suppression. Do not call this board an eighth-order Butterworth filter without evidence. Do not assume a 3.3 V timer output or a single supply works just because some chip configurations permit it.

### MAX262 programmable board — retained alternative

[Amazon listing, B0H1QJJ3HN](https://www.amazon.com/dp/B0H1QJJ3HN), generic brand, seller YanParts. User screenshot and live search showed **$54.23 + $1 shipping = $55.23**.

Photographs show installed SMA signal/clock connectors, jumpers, a multi-pin control header, and a three-position screw terminal with ±5 V markings. Board schematic and jumper map were not supplied.

Seller claims: low/high/band-pass, notch and all-pass modes; 64 frequency settings; 128 Q settings; four modes; external/RC/crystal clock options; 1 Hz–140 kHz center frequency; maximum 4 MHz clock; software support. Treat those as seller claims, not a guaranteed operating envelope for this assembled board.

[Official MAX260–MAX262 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX260-MAX262.pdf): two second-order sections, which can be cascaded for fourth order; independently programmable frequency and Q; parallel control using A0–A3, D0/D1, and WR, plus clock inputs. **It is not an SPI or I2C filter.**

Chip supports single or bipolar supplies, but single-supply grounding/bias and digital input thresholds require careful reading. Do not assume direct 3.3 V control in every supply arrangement. Datasheet clock feedthrough is on the order of 8 mV peak-to-peak, and output smoothing may be needed. Offset depends on mode/Q and can be substantial. The old software reference is not proof of a supported modern control application. Highest frequency claims depend on operating conditions.

Main disadvantages relative to the LTC1068 lead: more control wiring, lower maximum filter order per chip, and equally weak board documentation. Main advantage: programmable Q and mode as well as frequency.

### Fourth-order active RC board — lower-cost alternative

[Amazon listing, B0CB67BRHV](https://www.amazon.com/dp/B0CB67BRHV), Senzooe, title “Low Pass High Pass Filter 4th Order Active Filter RC Filter.” Observed **$21.02 + $5.70 shipping = $26.72**.

Photos show an assembled board with SMA input/output and screw terminals. Seller specifies a **100 kHz default cutoff**. Low-pass and high-pass variants are offered, but no 25–40 kHz cutoff option was visible. Supply range, response family, gain, and schematic were absent. Topology was not established; do not label it Sallen-Key from the title/photo alone.

This leaves much more accessory budget. A version supplied with an appropriate cutoff could be attractive. The default 100 kHz version is not qualified as the audio AAF: required attenuation depends on the actual sampling rate and out-of-band environment. Do not assume cutoff can be changed without soldering.

## Other filter paths researched

| Candidate | Findings and disposition |
|---|---|
| [Taidacent UAF42, Amazon B08VWPCPQQ](https://www.amazon.com/dp/B08VWPCPQQ) | $48.50 observed. Assembled SMA board, potentiometer adjustment, seller specifies ±5 to ±15 V. Default low-pass adjustment only up to about 5 kHz; reaching higher cutoffs requires RF1/RF2 changes. Reject stock version for solder-free full-audio use. Listing's generic “5 volts” summary is insufficient. |
| Other UAF42 boards | Amazon 300 Hz–15 kHz variant observed at $33.81 + $5.70 shipping; insufficient documented full-audio capability. Earlier ElectroPeak lead around $23.95 had the same default-range/component-change concern. Custom factory configuration remains an unverified possibility, not an agreed service. |
| [ADI DC338B-A / LTC1563-2](https://www.analog.com/media/en/technical-documentation/user-guides/DC338BF.PDF) | Assembled fourth-order Butterworth at 25.6 kHz; turret connections rather than SMA. One complete filter channel. Cutoff changes require resistor changes. Ideal fourth-order response would have about 0.57 dB droop at 20 kHz; this is a calculation, not a board measurement. Power/bias/connection suitability remains open. |
| [ADI DC393B / LTC1564](https://wiki.analog.com/resources/eval/dc393b) | Attractive SMA evaluation board. [LTC1564](https://www.analog.com/en/products/ltc1564.html) is an eighth-order continuous-time elliptic filter with digitally selected 10–150 kHz cutoff in 10 kHz steps and gain 1–16. Board includes input/output conditioning and distinct supply considerations. Earlier DigiKey observation: $232.20, zero stock. Outside current budget. |
| Kemo DR1200 / DR1600 | Instrument-style configurable filtering considered. User found DR1200 around £730; rejected on cost. |
| Alligator Technologies USBPGF-S1 | Enclosed programmable instrument considered; likely outside preferred budget. No confirmed affordable quotation. |
| TI DUAL-DIYAMP-EVM | Unpopulated/prototyping approach conflicts with no soldering. |
| Microchip EV58Y02A | Earlier review found gain/band configuration unsuitable without changes (approximately gain 50 and 30 Hz–20 kHz); not a drop-in solution. |
| Thorlabs EF122 | Passive BNC 20 kHz filter; earlier reseller price around $84.53. Source/load requirements and missing bias/drive complicate integration; too much of the total budget. |
| MAX292 generic SMA board | Earlier [seller PDF](https://ae-pic-a1.aliexpress-media.com/kf/S6766ca1c9ecf4798a9b0dc55a94ac47dg.pdf) described ±5 V, AC-coupled input/output, external clock, raw and smoothed outputs. Input specification only 0.1 Hz–10 kHz despite cutoff claims to 25 kHz. Not qualified for full audio. Amazon search found bare MAX292 chips, not a suitable assembled module. |
| [MAX295](https://www.analog.com/en/products/max295.html) | Promising eighth-order Butterworth IC with clock-controlled cutoff; no suitable assembled Amazon module found. Bare chips do not meet assembly preference. |
| [VBESTLIFE passive SMA filter, B0FM14T6PV](https://www.amazon.com/dp/B0FM14T6PV) | $36.99 observed. Title says DC–100 kHz; description says attenuation above 100 MHz. Internally contradictory and 50-ohm interfaces; do not buy on title alone. |
| Generic NE5532 subwoofer boards | Low prices, but typical 22–300 Hz cutoff is for bass/subwoofer use, not a full-audio AAF. |
| RF filters / power filters / AD637 RMS boards | Search results include many irrelevant products. MHz/GHz filters, power-ripple filters, notch-only filters, and RMS-to-DC detectors do not implement the required audio low-pass acquisition path. |

## DDS sine generator research

### Preferred: M5Stack DDS Unit U105

[Manufacturer documentation](https://docs.m5stack.com/en/unit/dds). Earlier session price **$28.50**. AD9833-based source with onboard STM32F0 and I2C interface (address 0x31). Reported waveform range up to 1 MHz and output approximately 0–0.6 V. Includes Grove control/power cable and SMA-to-2.54 mm signal cable, making it attractive for an assembled setup.

Requires **5 V power** and a controller; it is not an autonomous USB signal generator. Control logic/pull-ups and the exact cable-to-controller arrangement still need verification. Do not assume programmable amplitude. Output loading, offset, distortion, and suitability as a PP1 stimulus have not been measured. Phono-level stimulus may require attenuation.

### Alternatives retained

- [PMD Way AD9833 DDS module](https://pmdway.com/products/ad9833-dds-programmable-frequency-function-generator): earlier price $25.70; seller described fitted SMA/headers, SPI, MCP41010 amplitude control, and AD8051 buffer. Candidate if M5 integration becomes awkward; power and signal details still need verification.
- Mikroe Waveform Click: AD9833 with SMA and digital control/amplitude features considered; no verified budget-fit price captured.
- Amazon AD9850/AD9851 modules: observed examples around $17.99/$23.99. No complete verification of connector assembly, control wiring, output conditioning, or audio distortion. Advertised RF clock/output limits do not establish audio measurement quality.
- Keep the existing PWM test source for firmware diagnostics; its square wave does not replace a sine source for the intended characterization work.

## Budget snapshots

Using the earlier $28.50 M5Stack DDS price:

| Pair | Subtotal including observed filter shipping | Remaining below $100 |
|---|---:|---:|
| LTC1068 + M5Stack | $83.24 | $16.76 |
| MAX262 + M5Stack | $83.73 | $16.27 |
| Fourth-order RC board + M5Stack | $55.22 | $44.78 |

These exclude tax, DDS shipping, and any additional controller, power conversion, cables, adapters, bias/attenuator/driver, or protection. None is a verified complete sub-$100 bill of materials. Existing equipment may help, but its available rails and connections must be established.

## Resume here before spending

1. Obtain the exact LTC1068 board schematic/manual and chip suffix, including low-pass variant response/order, clock ratio/range/levels, supply rails, input impedance, gain, output bias/swing, and noise/clock-feedthrough data. No seller was contacted during this session.
2. Confirm the actual desired audio passband flatness and stopband attenuation, and current ADC acquisition rate. An AAF is chosen from those requirements, not merely by setting cutoff to 20 kHz or Nyquist.
3. Design and review the entire electrical connection to PE3, including bipolar-to-unipolar conditioning and ADC settling. A solder-free filter board alone does not solve this interface.
4. Select a control arrangement that avoids disrupting the DAQ firmware work. Verify DDS I2C and filter clock levels; reserve a suitable clock source rather than assuming an existing PWM pin is free.
5. Confirm a solder-free power solution, especially the LTC1068 board's apparent negative rail, and enumerate every cable/adapter. Reprice the complete chain.
6. If the LTC1068 cannot be documented or fit the accessory budget, investigate a factory-configured fourth-order RC board at a suitable cutoff. Do not buy the 100 kHz default hoping it is adjustable.
7. After selection, validate with a sine sweep and out-of-band tests: passband gain/phase, attenuation, bias/headroom, distortion/noise, clock artifacts, and ADC behavior. No such bench qualification has yet occurred.

## Research method and confidence

Amazon was successfully browsed in Chrome through native computer-use control. Search-engine/Amazon fetch failures earlier in the session were access-method limitations, not evidence that products were unavailable. Live searches included programmable filter modules, MAX292, MAX295, audio-range low-pass modules, and fourth-order low-pass filters. Product photos and seller descriptions were inspected; official IC documentation was used to separate chip capability from unverified module implementation.

No suitable BoosterPack-format assembled AAF was confirmed. No complete purchase-ready chain, hardware measurement, modern seller software package, or exact filter-board schematic was obtained. The final user decision was to archive the research and defer spending until the design is locked down.
