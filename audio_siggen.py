"""
Audio-jack test-signal generator for the TM4C123G DAQ  (EXAMPLE / EXPERIMENTAL)

Plays a looping test signal out of a laptop headphone jack (written for the
ThinkPad T480's Realtek ALC257) so it can drive PE3 through a coupling cap and
a 1.65 V bias network.  Not part of the DAQ firmware or GUI; nothing imports it.

Dependencies:
    pip install numpy sounddevice        (sounddevice only needed to play)

Examples:
    python audio_siggen.py --list-devices
    python audio_siggen.py sine --freq 1000 --level -6 --exclusive
    python audio_siggen.py sine --freq 1000 --adc-rate 200000 --adc-n 8192
    python audio_siggen.py multitone --freqs 1000 3100 7300 --level -12
    python audio_siggen.py sweep --f0 20 --f1 20000 --seconds 4
    python audio_siggen.py dc --level -6          # coupling test: read a DMM
    python audio_siggen.py sine --freq 997 --wav tone.wav --seconds 10

How it keeps the tone clean:
  * The loop buffer always holds a whole number of cycles of every tone, so
    looping never puts a phase step (and its spray of spurs) into the output.
  * --adc-rate/--adc-n snap the tone to an odd FFT bin of a DAQ capture of
    that length.  The laptop and LaunchPad clocks are independent, so this is
    near-coherent, not exact: still use a window (Blackman-Harris) in the FFT.
  * --exclusive uses WASAPI exclusive mode on Windows, which bypasses the
    Windows mixer (no resampling, no limiter, no system volume).
  * Square and triangle are band-limited (harmonics stop below 0.45 * rate),
    so they ring (Gibbs) instead of aliasing.  Use PB6 PWM for sharp edges.

Right channel (--right): copy | invert | off | marker.  "marker" puts a short
pulse at the start of every loop, for a scope trigger or, once the DAQ has a
second channel, a sync reference.
"""
import argparse
import math
import sys
import wave
from fractions import Fraction

import numpy as np

MAX_LOOP_SECONDS = 60.0


def dbfs_to_amp(dbfs):
    return 10.0 ** (dbfs / 20.0)


def snap_to_adc_bin(freq, adc_rate, adc_n):
    """Nearest odd bin k of an adc_n-point capture at adc_rate (odd k is
    coprime with a power-of-two N, so every ADC code phase gets exercised)."""
    k = max(1, round(freq * adc_n / adc_rate))
    if k % 2 == 0:
        k += 1
    return Fraction(adc_rate) * k / adc_n


def loop_length(freqs, rate, min_seconds):
    """Smallest sample count holding whole cycles of every frequency,
    repeated up to at least min_seconds."""
    period = 1
    for f in freqs:
        cycles_per_sample = Fraction(f) / rate
        period = period * cycles_per_sample.denominator // math.gcd(period, cycles_per_sample.denominator)
    if period / rate > MAX_LOOP_SECONDS:
        return None
    reps = max(1, math.ceil(min_seconds * rate / period))
    return period * reps


def exact_freqs(freqs, rate, n):
    """Round each tone to a whole number of cycles in n samples."""
    return [Fraction(max(1, round(f * n / rate)) * rate, n) for f in freqs]


def build(args):
    rate = args.rate
    want = max(args.seconds, 0.05)
    t = None

    if args.wave in ("sine", "multitone", "square", "triangle"):
        freqs = args.freqs if args.wave == "multitone" else [args.freq]
        if args.adc_rate:
            freqs = [snap_to_adc_bin(f, args.adc_rate, args.adc_n) for f in freqs]
        else:
            freqs = [Fraction(f).limit_denominator(1000) for f in freqs]
        n = loop_length(freqs, rate, want)
        if n is None:      # awkward ratio: settle for whole cycles in ~want s
            n = int(round(want * rate))
            freqs = exact_freqs(freqs, rate, n)
        t = np.arange(n) / rate
        if args.wave == "sine":
            x = np.sin(2 * np.pi * float(freqs[0]) * t)
        elif args.wave == "multitone":
            # Newman phases keep the crest factor down for a multitone
            m = len(freqs)
            x = sum(np.sin(2 * np.pi * float(f) * t + np.pi * (k * k) / m)
                    for k, f in enumerate(freqs))
        else:
            f0 = float(freqs[0])
            x = np.zeros(n)
            for h in range(1, int(0.45 * rate / f0) + 1, 2):   # odd harmonics
                if args.wave == "square":
                    x += np.sin(2 * np.pi * h * f0 * t) / h
                else:
                    x += ((-1) ** ((h - 1) // 2)) * np.sin(2 * np.pi * h * f0 * t) / (h * h)
        x /= np.max(np.abs(x))
        info = ", ".join(f"{float(f):.6f} Hz" for f in freqs)

    elif args.wave == "sweep":
        # log chirp f0 -> f1, then back down, so the loop joins without a step
        n_half = int(round(want * rate / 2))
        th = np.arange(n_half) / rate
        k = math.log(args.f1 / args.f0)
        dur = n_half / rate
        phase = 2 * np.pi * args.f0 * dur / k * (np.exp(th / dur * k) - 1)
        up = np.sin(phase)
        x = np.concatenate([up, up[-2:0:-1]])   # no repeated sample at either turn
        n = len(x)
        info = f"log sweep {args.f0}-{args.f1} Hz and back, {n / rate:.2f} s loop"

    elif args.wave == "noise":
        n = int(round(want * rate))
        x = np.random.default_rng(1).standard_normal(n)
        x /= 4 * np.std(x)            # ~4-sigma peak at the requested level
        x = np.clip(x, -1, 1)
        info = "white noise, 4-sigma peak at level"

    elif args.wave == "dc":
        n = int(round(want * rate))
        x = np.ones(n)
        info = "constant value (DC coupling test: read the jack on a DMM)"

    else:
        raise SystemExit(f"unknown wave {args.wave}")

    left = x * dbfs_to_amp(args.level)

    if args.right == "copy":
        right = left.copy()
    elif args.right == "invert":
        right = -left
    elif args.right == "marker":
        right = np.zeros(n)
        right[: max(1, int(rate * 0.0005))] = dbfs_to_amp(args.level)
    else:
        right = np.zeros(n)

    out = np.column_stack([left, right]).astype(np.float32)
    return out, info


def fade(buf, rate, ms=20):
    """Ramp the first few ms so starting the stream doesn't click."""
    k = min(len(buf), int(rate * ms / 1000))
    w = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, k))
    buf = buf.copy()
    buf[:k] *= w[:, None]
    return buf


def write_wav(path, buf, rate):
    """24-bit PCM, stereo."""
    pcm = np.round(np.clip(buf, -1, 1) * (2 ** 23 - 1)).astype("<i4")
    raw = pcm.reshape(-1).view(np.uint8).reshape(-1, 4)[:, :3].tobytes()
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(3)
        w.setframerate(rate)
        w.writeframes(raw)


def play(buf, args):
    import sounddevice as sd

    extra = None
    if args.exclusive:
        if not hasattr(sd, "WasapiSettings"):
            raise SystemExit("--exclusive needs Windows (WASAPI)")
        extra = sd.WasapiSettings(exclusive=True)

    first = [True]
    pos = [0]
    lead = fade(buf, args.rate)

    def callback(outdata, frames, _time, status):
        if status:
            print(status, file=sys.stderr)
        src = lead if first[0] else buf
        i, n, done = pos[0], len(buf), 0
        while done < frames:
            take = min(frames - done, n - i)
            outdata[done:done + take] = src[i:i + take]
            done += take
            i += take
            if i == n:
                i = 0
                first[0] = False
                src = buf
        pos[0] = i

    with sd.OutputStream(samplerate=args.rate, channels=2, dtype="float32",
                         device=args.device, extra_settings=extra,
                         callback=callback):
        print("Playing. Discard the first ~0.5 s of DAQ data; Ctrl+C to stop.")
        try:
            while True:
                sd.sleep(1000)
        except KeyboardInterrupt:
            pass


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("wave", nargs="?", default="sine",
                   choices=["sine", "multitone", "sweep", "square", "triangle", "noise", "dc"])
    p.add_argument("--freq", type=float, default=1000.0, help="tone frequency, Hz")
    p.add_argument("--freqs", type=float, nargs="+", default=[1000, 3100, 7300],
                   help="multitone frequencies, Hz")
    p.add_argument("--f0", type=float, default=20.0, help="sweep start, Hz")
    p.add_argument("--f1", type=float, default=20000.0, help="sweep end, Hz")
    p.add_argument("--level", type=float, default=-6.0, help="peak level, dBFS (default -6)")
    p.add_argument("--rate", type=int, default=48000, help="DAC sample rate (default 48000)")
    p.add_argument("--seconds", type=float, default=2.0, help="minimum loop length, s")
    p.add_argument("--right", default="copy", choices=["copy", "invert", "off", "marker"])
    p.add_argument("--adc-rate", type=int, help="DAQ sample rate, to snap tones to its FFT bins")
    p.add_argument("--adc-n", type=int, default=8192, help="DAQ capture length (default 8192)")
    p.add_argument("--device", help="output device name or index (see --list-devices)")
    p.add_argument("--exclusive", action="store_true", help="WASAPI exclusive mode (Windows)")
    p.add_argument("--wav", help="write a 24-bit WAV of one loop instead of playing")
    p.add_argument("--list-devices", action="store_true")
    args = p.parse_args()

    if args.list_devices:
        import sounddevice as sd
        print(sd.query_devices())
        return
    if args.level > 0:
        raise SystemExit("--level is dBFS and must be <= 0")
    if args.device is not None and args.device.isdigit():
        args.device = int(args.device)

    buf, info = build(args)
    print(f"{args.wave}: {info}; peak {args.level} dBFS; "
          f"loop {len(buf)} samples ({len(buf) / args.rate:.3f} s) at {args.rate} Hz")

    if args.wav:
        write_wav(args.wav, buf, args.rate)
        print(f"wrote {args.wav}")
    else:
        play(buf, args)


if __name__ == "__main__":
    main()
