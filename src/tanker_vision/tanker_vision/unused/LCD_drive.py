# -*- coding: utf-8 -*-
"""
Compiled, mashed and generally mutilated 2014-2015 by Denis Pleic
Made available under GNU GENERAL PUBLIC LICENSE
# Modified Python I2C library for Raspberry Pi
# as found on http://www.recantha.co.uk/blog/?p=4849
# Joined existing 'i2c_lib.py' and 'lcddriver.py' into a single library
# added bits and pieces from various sources
# By DenisFromHR (Denis Pleic)
# 2015-02-10, ver 0.1
"""
#
#
import smbus
from time import *
import threading
import shutil

class i2c_device:
   def __init__(self, addr, port=1):
      self.addr = addr
      self.bus = smbus.SMBus(port)

# Write a single command
   def write_cmd(self, cmd):
      self.bus.write_byte(self.addr, cmd)
      sleep(0.0001)

# Write a command and argument
   def write_cmd_arg(self, cmd, data):
      self.bus.write_byte_data(self.addr, cmd, data)
      sleep(0.0001)

# Write a block of data
   def write_block_data(self, cmd, data):
      self.bus.write_block_data(self.addr, cmd, data)
      sleep(0.0001)

# Read a single byte
   def read(self):
      return self.bus.read_byte(self.addr)

# Read
   def read_data(self, cmd):
      return self.bus.read_byte_data(self.addr, cmd)

# Read a block of data
   def read_block_data(self, cmd):
      return self.bus.read_block_data(self.addr, cmd)



# LCD Address
ADDRESS = 0x27

# commands
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

# flags for display entry mode
LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

# flags for display on/off control
LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00

# flags for display/cursor shift
LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE = 0x00
LCD_MOVERIGHT = 0x04
LCD_MOVELEFT = 0x00

# flags for function set
LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_1LINE = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS = 0x00

# flags for backlight control
LCD_BACKLIGHT = 0x08
LCD_NOBACKLIGHT = 0x00

En = 0b00000100 # Enable bit
Rw = 0b00000010 # Read/Write bit
Rs = 0b00000001 # Register select bit

class lcd:
   #initializes objects and lcd
   def __init__(self):
      self.lcd_device = i2c_device(ADDRESS)

      self.lcd_write(0x03)
      self.lcd_write(0x03)
      self.lcd_write(0x03)
      self.lcd_write(0x02)

      self.lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE)
      self.lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON)
      self.lcd_write(LCD_CLEARDISPLAY)
      self.lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT)
      sleep(0.2)


   # clocks EN to latch command
   def lcd_strobe(self, data):
      self.lcd_device.write_cmd(data | En | LCD_BACKLIGHT)
      sleep(.0005)
      self.lcd_device.write_cmd(((data & ~En) | LCD_BACKLIGHT))
      sleep(.0001)

   def lcd_write_four_bits(self, data):
      self.lcd_device.write_cmd(data | LCD_BACKLIGHT)
      self.lcd_strobe(data)

   # write a command to lcd
   def lcd_write(self, cmd, mode=0):
      self.lcd_write_four_bits(mode | (cmd & 0xF0))
      self.lcd_write_four_bits(mode | ((cmd << 4) & 0xF0))

   # write a character to lcd (or character rom) 0x09: backlight | RS=DR<
   # works!
   def lcd_write_char(self, charvalue, mode=1):
      self.lcd_write_four_bits(mode | (charvalue & 0xF0))
      self.lcd_write_four_bits(mode | ((charvalue << 4) & 0xF0))
  

   # put string function
   def lcd_display_string(self, string, line):
      if line == 1:
         self.lcd_write(0x80)
      if line == 2:
         self.lcd_write(0xC0)
      if line == 3:
         self.lcd_write(0x94)
      if line == 4:
         self.lcd_write(0xD4)

      for char in string:
         self.lcd_write(ord(char), Rs)

   # clear lcd and set to home
   def lcd_clear(self):
      self.lcd_write(LCD_CLEARDISPLAY)
      self.lcd_write(LCD_RETURNHOME)

   # define backlight on/off (lcd.backlight(1); off= lcd.backlight(0)
   def backlight(self, state): # for state, 1 = on, 0 = off
      if state == 1:
         self.lcd_device.write_cmd(LCD_BACKLIGHT)
      elif state == 0:
         self.lcd_device.write_cmd(LCD_NOBACKLIGHT)

   # add custom characters (0 - 7)
   def lcd_load_custom_chars(self, fontdata):
      self.lcd_write(0x40);
      for char in fontdata:
         for line in char:
            self.lcd_write_char(line)         
         
   # define precise positioning (addition from the forum)
   def lcd_display_string_pos(self, string, line, pos):
    if line == 1:
      pos_new = pos
    elif line == 2:
      pos_new = 0x40 + pos
    elif line == 3:
      pos_new = 0x14 + pos
    elif line == 4:
      pos_new = 0x54 + pos

    self.lcd_write(0x80 + pos_new)

    for char in string:
      self.lcd_write(ord(char), Rs)

import threading
from time import sleep

class StatusDisplay:
    """
    A class to manage a 20x4 LCD status display, with the custom characters and LCD 
    initialization performed within the class. The 'heartbeat' function is non-blocking,
    as it spawns a background thread to handle the brief 'flash' animation.
    """

    def __init__(self):
        # A lock to protect all state updates:
        self._lock = threading.Lock()

        # Define the three custom characters (CGRAM bitmaps) inside the class:
        self.check_mark = [
            0b00000,
            0b00000,
            0b00001,
            0b00010,
            0b10100,
            0b01000,
            0b00000,
            0b00000
        ]
        self.cross_mark = [
            0b00000,
            0b00000,
            0b10001,
            0b01010,
            0b00100,
            0b01010,
            0b10001,
            0b00000
        ]
        self.solid_square = [
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111,
            0b11111
        ]

        # The LCD instance is created within the class:
        self.lcd = lcd()

        # Load the custom characters into the LCD's CGRAM. 
        # Indices: 0 => check_mark, 1 => cross_mark, 2 => solid_square
        self.lcd.lcd_load_custom_chars([
            self.check_mark,
            self.cross_mark,
            self.solid_square
        ])

        # For convenience, store the escape codes we’ll use in the text:
        self.CHECK_CHAR = "\x00"      # check mark
        self.CROSS_CHAR = "\x01"      # cross mark
        self.HEARTBEAT_CHAR = "\x02"  # solid block

        # Create "shadow" lines for the display (20 chars each).
        # Initialize them to the default state.
        self.line1 = list("ON GROUND|CELLULAR:" + self.CROSS_CHAR)
        self.line2 = list("IMU:" + self.CROSS_CHAR +
                          "|GPS:" + self.CROSS_CHAR +
                          "|T.SYNC:" + self.CROSS_CHAR)
        self.line3 = list("CAM:----|RECORDING:-")
        self.line4 = list("STOR:--%|HEARTBEAT: " )

        # Display the defaults immediately.
        with self._lock:
            self._refresh_line(1)
            self._refresh_line(2)
            self._refresh_line(3)
            self._refresh_line(4)

    def _refresh_line(self, line_number):
        """
        Helper to rewrite the entire line (1..4) to the LCD 
        from our local shadow buffer.
        NOTE: Caller must hold self._lock while calling this.
        """
        text = "".join(getattr(self, f"line{line_number}"))
        self.lcd.lcd_display_string(text, line_number)

    def set_system_state(self, in_air=False):
        """
        Sets the system state on line 1, positions [0..8]:
            "ON GROUND" or "IN AIR   "
        """
        with self._lock:
            text = "IN AIR   " if in_air else "ON GROUND"
            for i, ch in enumerate(text):
                self.line1[i] = ch
            self._refresh_line(1)

    def set_cellular_status(self, connected=False):
        """
        Sets the last char on line 1 (position 19) to:
           check mark (\x00) if connected
           cross mark (\x01) if not
        """
        with self._lock:
            self.line1[19] = self.CHECK_CHAR if connected else self.CROSS_CHAR
            self._refresh_line(1)

    def set_imu_status(self, ok=False):
        """ Line 2, position 4 """
        with self._lock:
            self.line2[4] = self.CHECK_CHAR if ok else self.CROSS_CHAR
            self._refresh_line(2)

    def set_gps_status(self, ok=False):
        """ Line 2, position 10 """
        with self._lock:
            self.line2[10] = self.CHECK_CHAR if ok else self.CROSS_CHAR
            self._refresh_line(2)

    def set_timesync_status(self, ok=False):
        """ Line 2, position 19 """
        with self._lock:
            self.line2[19] = self.CHECK_CHAR if ok else self.CROSS_CHAR
            self._refresh_line(2)

    def set_camera_state(self, state="----"):
        """
        Line 3, positions [4..7]:  one of ["----", "TOUT", "ERR!", "OKAY"]
        Default is "----".
        """
        with self._lock:
            state = state[:4].ljust(4)
            for i in range(4):
                self.line3[4 + i] = state[i]
            self._refresh_line(3)

    def set_recording_status(self, recording=False, error=False):
        """
        Line 3, position 19 (\x00 => check, \x01 => cross).
        """
        with self._lock:
            if error:
                self.line3[19] = '-'
            else:
               self.line3[19] = self.CHECK_CHAR if recording else self.CROSS_CHAR
            
            self._refresh_line(3)

    def set_storage_value(self, val=0):
        """
        Line 4, positions [5..6], two digits for free storage (00..99).
        """
        with self._lock:
            s = str(val).rjust(2, "0")[:2]
            self.line4[5] = s[0]
            self.line4[6] = s[1]
            self._refresh_line(4)

    def heartbeat(self):
        """
        Initiates a "fire-and-forget" thread to display the solid box (\x02) at 
        line 4, position 19 for ~0.4s, then remove it. The main thread is NOT blocked.
        """
        # Spawn a background thread to do the flash logic
        t = threading.Thread(target=self._heartbeat_worker, daemon=True)
        t.start()

    def _heartbeat_worker(self):
        """
        The actual worker that runs in a background thread. 
        Displays the heartbeat character, sleeps briefly, then clears it.
        """
        with self._lock:
            self.line4[19] = self.HEARTBEAT_CHAR
            self._refresh_line(4)

        sleep(0.4)  # Thread-based sleep, no blocking of the main thread

        with self._lock:
            self.line4[19] = " "
            self._refresh_line(4)

def main():
    # Example usage of this class:
    display = StatusDisplay()  # creates lcd(), loads chars, and sets defaults

    # Adjust some statuses:
    display.set_system_state(in_air=True)
    sleep(0.5)
    display.set_cellular_status(True)
    sleep(0.5)
    display.set_imu_status(True)
    sleep(0.5)
    display.set_gps_status(False)
    sleep(0.5)
    display.set_timesync_status(True)
    sleep(0.5)
    display.set_camera_state("OKAY")
    sleep(0.5)
    display.set_recording_status(True)
    sleep(0.5)
    display.set_storage_value(42)
    sleep(0.5)

    # Show heartbeat in a loop
    while True:
        display.heartbeat()
        sleep(1)


if __name__ == "__main__":

    main()