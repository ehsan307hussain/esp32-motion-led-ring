#!/usr/bin/env python3
"""Laptop sounds for ESP32 LED Snake Game.

Firmware sends EVENT:EAT and EVENT:WIN over USB serial.
Install: python -m pip install pyserial pygame
Run: python snake_sound.py --port /dev/ttyUSB0
Close `idf.py monitor` before running (serial port can only have one reader).
"""
import argparse
import math
import sys
from array import array

try:
    import serial
    import pygame
except ImportError:
    print("Missing libraries. Run: python -m pip install pyserial pygame")
    sys.exit(1)

SAMPLE_RATE = 44100


def note(frequency, duration, volume=0.3):
    """Produce a softened sine-wave note as 16-bit mono PCM samples."""
    frames = max(1, int(SAMPLE_RATE * duration))
    samples = array('h')
    for i in range(frames):
        t = i / SAMPLE_RATE
        attack = min(1.0, t / 0.012)
        release = min(1.0, (duration - t) / 0.065)
        envelope = max(0.0, min(attack, release))
        value = math.sin(2 * math.pi * frequency * t)
        value += 0.18 * math.sin(4 * math.pi * frequency * t)
        samples.append(int(32767 * volume * envelope * value / 1.18))
    return samples


def silence(duration):
    return array('h', [0]) * int(SAMPLE_RATE * duration)


def as_sound(samples):
    if sys.byteorder != 'little':
        samples.byteswap()
    return pygame.mixer.Sound(buffer=samples.tobytes())


def build_sounds():
    eat = note(880, 0.085, 0.28) + note(1175, 0.105, 0.30)

    # Short cheerful victory tune (~2.2 s).
    win = array('h')
    for freq, duration in [
        (523.25, 0.20), (659.25, 0.20), (783.99, 0.20),
        (1046.50, 0.31), (783.99, 0.17), (1046.50, 0.52),
    ]:
        win.extend(note(freq, duration, 0.35))
        win.extend(silence(0.035))
    return as_sound(eat), as_sound(win)


def main():
    parser = argparse.ArgumentParser(description="ESP32 Snake Game laptop sound effects")
    parser.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 serial port')
    parser.add_argument('--baud', type=int, default=115200, help='ESP32 serial baud rate')
    args = parser.parse_args()

    try:
        pygame.mixer.init(frequency=SAMPLE_RATE, size=-16, channels=1, buffer=512)
        eat_sound, win_sound = build_sounds()
        eat_channel = pygame.mixer.Channel(0)
        win_channel = pygame.mixer.Channel(1)
    except pygame.error as exc:
        sys.exit(f"Unable to initialize laptop audio: {exc}")

    print(f"Listening to {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
    print("Close 'idf.py monitor' first. Opening the port may reset your ESP32.")
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as connection:
            while True:
                raw = connection.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
                if 'EVENT:WIN' in line:
                    print('WIN! Playing victory tune.')
                    eat_channel.stop()
                    win_channel.play(win_sound)
                elif 'EVENT:EAT' in line:
                    print('Food eaten: beep!')
                    eat_channel.play(eat_sound)
                elif line and ('SNAKE_RING' in line or 'EVENT:' in line):
                    print(line)
    except KeyboardInterrupt:
        print('\nSound listener stopped.')
    except serial.SerialException as exc:
        sys.exit(f"Serial error: {exc}. Check the port and close idf.py monitor.")
    finally:
        pygame.mixer.quit()


if __name__ == '__main__':
    main()
