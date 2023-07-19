# Copyright (C) 2022-2023 Thies Lennart Alff

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

import time
import pigpio
import threading


def synced(func):

    def wrap(s, *args):
        with s.lock:
            func(s, *args)

    return wrap


class Buzzer():

    def __init__(self, pin):
        self.pin = pin
        self.pi = pigpio.pi()
        self.lock = threading.RLock()

    @synced
    def blip(self):
        self.pi.write(self.pin, pigpio.HIGH)
        self.pi.write(self.pin, pigpio.LOW)

    @synced
    def high_pitch(self, duration=0.0):
        self.pi.set_PWM_frequency(self.pin, 4000)
        self.pi.set_PWM_dutycycle(self.pin, 128)
        if duration > 0:
            time.sleep(duration)
            self.off()

    @synced
    def low_pitch(self, duration=0.0):
        self.pi.set_PWM_frequency(self.pin, 2000)
        self.pi.set_PWM_dutycycle(self.pin, 128)
        if duration > 0:
            time.sleep(duration)
            self.off()

    @synced
    def happy(self, time_per_tone: float):
        self.low_pitch()
        time.sleep(time_per_tone)
        self.high_pitch()
        time.sleep(time_per_tone)
        self.off()

    @synced
    def double_low(self, length=0.1, delay=0.1):
        self.low_pitch(length)
        time.sleep(delay)
        self.low_pitch(length)

    @synced
    def sad(self, time_per_tone: float):
        self.high_pitch(time_per_tone)
        self.low_pitch(time_per_tone)

    @synced
    def off(self):
        self.pi.write(self.pin, pigpio.LOW)
