# Oversampling, decimation and how to measure this ADC

Reference notes for the TM4C123G DAQ. Written 2026-09-13, from a working
discussion about what higher streaming throughput actually buys the project.

The short version: **link bandwidth converts into resolution**, at a rate of
half a bit per doubling, and the conversion is done entirely on the host for
free. But the bigger practical payoff is that it makes the analog anti-alias
filter much easier to build.

---

## 1. Where the extra bits come from

A 12-bit ADC has an LSB of `q = FSR / 4096`. The error between the true analog
value and the reported code is bounded by `+/- q/2`; if that error is uniformly
distributed its RMS is `q/sqrt(12)`, which gives the familiar

```
SNR = 6.02*N + 1.76 dB        = 74 dB for N = 12
```

The key insight is that **this noise power is a fixed budget and does not
depend on sample rate.** Sampling faster does not make the per-sample error
smaller. What changes is where that power sits in the spectrum.

Sampling at `fs` spreads the fixed total uniformly from DC to `fs/2`, so the
noise power spectral density is

```
(q^2 / 12) / (fs / 2)
```

Double `fs` and the same total power is spread over twice the bandwidth --
**half the density**. It is the same bookkeeping as integrating an analog noise
density over a bandwidth: wider band, same total, lower density.

The signal of interest occupies only `BW` (20 kHz for audio). Everything above
that is noise being carried for no reason. Filter it away digitally and you
keep only `BW / (fs/2)` of the noise power while keeping 100% of the signal.

With the oversampling ratio defined as `OSR = (fs/2) / BW`:

```
processing gain = 10 * log10(OSR)
```

**Each doubling of OSR buys 3 dB, which is half a bit.** 4x is one bit, 16x is
two bits. Noise voltage falls as `sqrt(OSR)` -- the same square-root law as
averaging N independent measurements, because that is exactly what it is.

## 2. What decimation actually is

Two operations, usually named after the wrong one:

1. **Digitally low-pass filter** to the final band (20 kHz). *This is the step
   that buys the bits.*
2. **Discard samples** to bring the rate down (keep 1 in 16 -> 48 kS/s).

Step 2 is bookkeeping. After step 1 the signal is band-limited, so samples at
800 kS/s are redundant and Nyquist says 48 k is sufficient. Discarding them
loses nothing.

**The order is not negotiable.** Discard first and all the out-of-band noise
aliases straight back into the signal band, gaining precisely nothing.

This runs on the host, in numpy (`scipy.signal.decimate`), for free. No
firmware cost, no MCU cycles. The only thing standing between the DAQ and the
extra bits is link bandwidth -- which is why streaming throughput is worth
chasing even though audio does not need the raw sample rate.

## 3. The three catches

**3.1 It only works if the quantization error looks like noise.**
The derivation assumes the error is uniformly distributed and *uncorrelated
with the signal*. A clean, quiet input into a perfect ADC produces a
deterministic error that repeats with the signal -- it appears as harmonics, not
white noise, and averaging does not touch it. The cure is dither: roughly
1 LSB RMS of noise on the input.

This board has that by accident (unshielded LaunchPad, no front end, USB 5 V
supply). If a genuinely quiet front end is ever built, dither may have to be
added back deliberately.

**3.2 It only removes noise outside the final band.**
Broadband white noise -- quantization, S/H thermal noise, aperture jitter -- is
spread across Nyquist, and decimation captures the out-of-band portion. Noise
already inside 20 kHz is untouched: 1/f, reference noise, supply ripple, SMPS
harmonics, digital ground coupling. On a bare LaunchPad that in-band coupling
is the likely real floor, which means shielding and supply work may buy more
than bandwidth does.

**3.3 Static errors do not average away.**
INL, DNL, gain and offset error are deterministic functions of code. They set
THD and are completely unaffected by oversampling.

## 4. What this is worth here

| streaming rate | OSR to 20 kHz | processing gain | theoretical ENOB |
|---|---|---|---|
| 200 kS/s (current default) | 5.0 | 7.0 dB | 13.2 |
| 344 kS/s (today's link ceiling) | 8.6 | 9.3 dB | 13.5 |
| ~700 kS/s (double-buffered, optimistic) | 17.5 | 12.4 dB | 14.1 |

Read that honestly: **doubling the link is 3 dB, half a bit.** Real and
measurable, but modest. The square-root law is unforgiving -- each further bit
costs 4x the bandwidth. And the ENOB column is a ceiling that will not be
reached, because the part's actual ENOB at Nyquist is well below 12. The real
SINAD figure should be pulled from the datasheet rather than assumed.

## 5. The bigger win: filter relaxation

Sampling at 48 kS/s directly demands an analog filter that is flat at 20 kHz
and dead by 24 kHz -- a brick wall, 8th order or more, with the phase
distortion, tolerance sensitivity and cost that implies.

Sampling at 800 kS/s only demands it be dead by 400 kHz. That is **4.3 octaves
of transition room**, which a gentle 2nd-order filter handles. All the sharp
selectivity moves into the digital decimation filter, where it is free, exact,
and linear-phase if desired.

Trading sample rate for analog filter complexity is really the whole point of
oversampling as a technique. The extra bits are almost a bonus. This is the
argument that matters when the AAF board arrives.

---

## 6. Measuring the ADC with what is on the bench today

### Why the PWM square wave cannot give a SINAD number

SINAD assumes a pure single tone: the fundamental is "signal" and everything
else is "noise + distortion". A square wave is made of harmonics -- the
fundamental carries only 90% of its RMS, so harmonic content alone is about
-6.3 dB relative to it. The measurement would return roughly 6 dB and say
nothing about the converter.

Excluding the known harmonic bins does not rescue it, because **with no AAF the
harmonics alias**. Harmonics above `fs/2` fold back to unpredictable in-band
frequencies. Worked example already in the project notes: the 9th harmonic of
the 40 kHz preset (360 kHz) folds to 26.7 kHz at 333 kS/s -- *below* the
fundamental. Duty is not exactly 50%, so even harmonics are present too and the
folded spectrum is dense. Those aliased tones land in exactly the bins needed
to estimate the noise floor.

Folded frequency for harmonic `n` of `f0`:

```
f_apparent = | n*f0 - round(n*f0 / fs) * fs |
```

Blanking every one of those leaves too little spectrum to trust.

### The better experiment: a static input

Pull the PB6 -> PE3 jumper and feed PE3 a quiet DC voltage. Every feature in
the resulting spectrum is then noise, with nothing to separate out. This is how
a noise floor is characterised on the bench anyway.

Practically: a 1k/1k divider from 3.3 V puts PE3 at mid-scale (1.65 V), which
exercises the most code transitions and avoids rail effects. 1 uF from the tap
to ground makes the source itself quiet above ~300 Hz, so what is measured is
the ADC and the board rather than the divider. 500 ohm source impedance is
low -- worth confirming against the part's maximum source impedance for
full-rate sampling, which is still an unchecked datasheet number.

One capture then yields all of:

- **Noise in LSBs RMS**, from the standard deviation of the codes. Converts
  directly to noise-free bits; the single most useful number about this board.
- **A code histogram.** Spread across several codes means dither is present and
  oversampling will work. Collapsed onto one or two codes means the floor is
  below 1 LSB -- good news, but dither would have to be added deliberately.
- **Spectral shape.** Detrend to remove DC drift, then FFT. Flat means white,
  and decimation will deliver the full `10*log10(OSR)`. A 1/f rise, or discrete
  spurs from USB switching, mains or digital coupling, is in-band noise that
  decimation will *not* remove.
- **Verification of the decimation gain**, with no signal needed at all:
  decimate, recompute RMS, confirm it fell by `sqrt(OSR)`.

The square wave remains useful for what it is good at -- checking the
decimation filter does not mangle a real signal, and confirming aliases land
where the arithmetic predicts. Just not for SINAD.

### What a real signal source needs to be

To characterise a ~10-11 ENOB converter the source must be **cleaner than the
converter**: THD+N better than about -80 dBc, ideally -90. Many budget AWGs are
only -50 to -60 dBc and would end up characterising themselves. A dedicated
low-distortion oscillator, or a decent generator followed by a passive LC or RC
low-pass on the breadboard, both reach the requirement cheaply.

### Synthesising a sine on-board (and why it is not a standard)

Sine-weighted PWM would work: vary duty from a lookup table at the audio rate
with the carrier well above it, then RC away the carrier. Satisfying to see,
but at an 80 MHz clock a 100 kHz carrier gives only ~800 duty steps, about
9.6 bits of amplitude resolution. That sets spectral purity well short of what
is needed to characterise a 12-bit converter. A good demo, not a measurement
standard.

### Bench trick worth knowing

Distortion can be measured *below* the analyser's own noise floor by notching
out the fundamental before the input stage. Kill the large tone passively, then
amplify the remaining harmonics into a comfortable range. This is how THD was
measured long before anything had 24 bits, and it applies directly here.
