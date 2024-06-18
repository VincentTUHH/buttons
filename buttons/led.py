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

import threading
import time

from rpi_ws281x import Color, PixelStrip

LED_COUNT = 4
LED_PIN = 10
LED_FREQ_HZ = 900000
LED_DMA = 10
LED_INVERT = False
LED_CHANNEL = 0


class Strip:
    STATUS_INDEX = 0
    ARMING_INDEX = 1
    BATTERY_INDEX = 2

    COLOR_ARMED = Color(254, 0, 0)
    COLOR_DISARMED = Color(0, 254, 0)

    COLOR_BATTERY_GOOD = Color(0, 254, 0)
    COLOR_BATTERY_MEDIUM = Color(254, 100, 0)
    COLOR_BATTERY_BAD = Color(254, 0, 0)

    COLOR_STATE_GOOD = Color(0, 254, 0)
    COLOR_STATE_BAD = Color(254, 0, 0)
    COLOR_OFF = Color(0, 0, 0)
    COLOR_UNDEFINED = Color(0, 0, 254)

    def __init__(self):
        self.lock = threading.RLock()
        self.strip = PixelStrip(
            LED_COUNT,
            LED_PIN,
            LED_FREQ_HZ,
            LED_DMA,
            LED_INVERT,
            255,
            LED_CHANNEL,
        )
        self.strip.begin()
        self.armed = False
        self.t_arming_blinked = 0.0

    def blink_arming(self):
        with self.lock:
            if self.armed:
                self.strip.setPixelColorRGB(self.ARMING_INDEX, 254, 0, 0)
                self.strip.show()
                time.sleep(0.1)
                self.strip.setPixelColorRGB(self.ARMING_INDEX, 0, 0, 0)
                self.strip.show()

    def color_wipe(self, color: Color, duration: float):
        with self.lock:
            if duration > 0:
                delay_per_pixel = 1.0 / duration
            else:
                delay_per_pixel = 0
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, color)
                self.strip.show()
                time.sleep(delay_per_pixel)

    def switch_off(self):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColorRGB(i, 0, 0, 0)
        self.strip.show()

    def set_status(self, good=False):
        with self.lock:
            self.strip.setPixelColorRGB(
                self.STATUS_INDEX, (1 - int(good)) * 254, int(good) * 254, 0
            )
            self.strip.show()

    def set_arming(
        self,
        time: float,
        armed=False,
    ):
        with self.lock:
            if armed != self.armed:
                self.t_arming_blinked = time
                self.armed = armed
                if armed:
                    self.just_armed()
                else:
                    self.just_disarmed()
            else:
                if not self.armed:
                    self.just_disarmed()
                if time - self.t_arming_blinked >= 1.0:
                    self.blink_arming()

    def set_arming_state_undefined(self):
        with self.lock:
            self.strip.setPixelColor(self.ARMING_INDEX, self.COLOR_UNDEFINED)

    def just_armed(self):
        with self.lock:
            self.blink_arming()

    def just_disarmed(self):
        with self.lock:
            self.strip.setPixelColor(self.ARMING_INDEX, self.COLOR_DISARMED)
            self.strip.show()

    def set_battery_good(self):
        with self.lock:
            self.strip.setPixelColor(
                self.BATTERY_INDEX, self.COLOR_BATTERY_GOOD
            )
            self.strip.show()

    def set_battery_medium(self):
        with self.lock:
            self.strip.setPixelColor(
                self.BATTERY_INDEX, self.COLOR_BATTERY_MEDIUM
            )
            self.strip.show()

    def set_battery_low(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX, self.COLOR_BATTERY_BAD)
            self.strip.show()

    def set_battery_undefined(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX, self.COLOR_UNDEFINED)
            self.strip.show()
