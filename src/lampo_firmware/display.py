#!/usr/bin/python3

import time
import math
import smbus

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

class Display:
    
    def __init__(self, i2c_address=0x3C):

        self.display = disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_address=i2c_address)
        self.display.begin()
        self.width = self.display.width
        self.height = self.display.height
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default() 
        self.strings= []

        cmd = "hostname -I | cut -d\' \' -f1"
        self.ip = subprocess.check_output(cmd, shell = True ).decode('UTF-8')
        # Clear display.
        self.display.clear()
        self.display.display()

    def clear(self):
        self.draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)
        self.display.clear()
        self.display.display()
        
    def addString(self, string):
        self.strings.append(string)
        if len(self.strings) > 6:
            self.strings.pop(0)

    def text(self, text, line):
        " LINE 0,1 - yellow, LINE 2-7 - blue"
        self.draw.text((0, (line*8)-2), text,  font=self.font, fill=255)

    def drawIP(self):
        self.text(f"IP: {self.ip}", 0)
        self.text(f"-------------------", 1)        

    def push(self):
        self.clear()
        self.drawIP()
        for i, s in enumerate(self.strings[::-1]):
            print(s)
            self.text(s, 2 + i)
        self.display.image(self.image)
        self.display.display()            
