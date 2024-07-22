"""
@author: Ziad (https://github.com/ZiadHatab)

Code for driving SSD1306 OLED display using an FTID FT232H chip over I2C
"""

import time
import os
import pprint

import numpy as np     # pip install numpy -U
import PIL             # pip install pillow -U
from PIL import Image
import ftd2xx as ftdi  # pip install ftd2xx -U

## Commands hex values for FTDI FT232H chip
"""
Command definition from AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
Everything is sent/read in bytes (8bits). I'm including only commands for ISP and I2C

Main pins of interest for MPSSE (serial) protocols are D0-D3, see below table. Other pins are GPIOs.

D0-D7 is one byte (always triggered together)
D0 (TCK/SK) | output       | Clock
D1 (TDI/DO) | output       | Serial data out
D2 (TDO/DI) | input        | Serial data in
D3 (TMS/CS) | output       | Select signal
D4 (GPIOL0) | output/input | GPIO 
D5 (GPIOL1) | output/input | GPIO
D6 (GPIOL2) | output/input | GPIO 
D7 (GPIOL3) | output/input | GPIO 
"""

## SYNC writing and reading data frame structureS
# For writing:
#   BYTES: cmd lengthL lengthH BYTE1,... 
#   BITS : cmd length BITS  (total 8 bits)
#
# For reading:
#   BYTES: cmd lengthL lengthH 
#   BITS : cmd length

# commands that send/read MSB first
# writing to TDI/DO (no reading from TDO/DI)
MSB_W_BYTES_RISE = 0x10
MSB_W_BYTES_FALL = 0x11
MSB_W_BITS_RISE  = 0x12
MSB_W_BITS_FALL  = 0x13
# reading from TDO/DI (no writing to TDI/DO)
MSB_R_BYTES_RISE = 0x20
MSB_R_BYTES_FALL = 0x24
MSB_R_BITS_RISE  = 0x22
MSB_R_BITS_FALL  = 0x26
# writing to TDI/DO and reading from TDO/DI
MSB_R_W_BYTES_RISE_FALL = 0x31
MSB_R_W_BYTES_FALL_RISE = 0x34
MSB_R_W_BITS_RISE_FALL  = 0x33
MSB_R_W_BITS_FALL_RISE  = 0x36

# commands that send/read LSB first
# writing to TDI/DO (no reading from TDO/DI)
LSB_W_BYTES_RISE = 0x18
LSB_W_BYTES_FALL = 0x19
LSB_W_BITS_RISE  = 0x1a
LSB_W_BITS_FALL  = 0x1b
# reading from TDO/DI (no writing to TDI/DO)
LSB_R_BYTES_RISE = 0x28
LSB_R_BYTES_FALL = 0x2c
LSB_R_BITS_RISE  = 0x2a
LSB_R_BITS_FALL  = 0x2e
# writing to TDI/DO and reading from TDO/DI
LSB_R_W_BYTES_RISE_FALL = 0x39
LSB_R_W_BYTES_FALL_RISE = 0x3c
LSB_R_W_BITS_RISE_FALL  = 0x3b
LSB_R_W_BITS_FALL_RISE  = 0x3e

## Set and read pins status. 
# Low byte  : DBUS 7-0
# High byte : CBUS 7-0
#
# For writing (set): cmd value direction
#   value     : a byte setting the value of the 8 pins (DBUS or CBUS)
#   direction : a byte to set which pins of the 8 pins to be output (DBUS or CBUS)
#
# For reading: cmd
#
# When writing, the pins are latched!! you need to set them back manually (I think!?)
SET_LOW   = 0x80
SET_HIGH  = 0x82
READ_LOW  = 0x81
READ_HIGH = 0x83

# Loopback: connecting TDI/DO to TDO/DI (for I2C)
LOOPBACK_ON  = 0x84
LOOPBACK_OFF = 0x85

## Setting clock rate
# There are 2 options:
#   Divide by 5 ON:  f(MHz) = 12MHz/2/(1 + Value)
#   Divide by 5 OFF: f(MHz) = 60MHz/2/(1 + Value)
#       where Value = 0xValueH_ValueL = 0x0000->0xFFFF
# To set the clock: cmd ValueL ValueH
SET_CLK  = 0x86
# To set divide by 5 on/off: cmd
DIV_5_ON  = 0x8b
DIV_5_OFF = 0x8a


## CONSTANTS for the SSD1306 OLED display. To support the different display size, change values related to display size
"""
See documentation SSD1306.pdf
"""

# SSD1306 OLED register definitions
SET_CONTRAST        = 0x81
SET_ENTIRE_ON       = 0xA4
SET_NORM_INV        = 0xA6
SET_DISP            = 0xAE
SET_MEM_ADDR        = 0x20
SET_COL_ADDR        = 0x21
SET_PAGE_ADDR       = 0x22
SET_DISP_START_LINE = 0x40
SET_SEG_REMAP       = 0xA0
SET_MUX_RATIO       = 0xA8
SET_COM_OUT_DIR     = 0xC0
SET_DISP_OFFSET     = 0xD3
SET_COM_PIN_CFG     = 0xDA
SET_DISP_CLK_DIV    = 0xD5
SET_PRECHARGE       = 0xD9
SET_VCOM_DESEL      = 0xDB
SET_CHARGE_PUMP     = 0x8D

external_vcc    = False
page_addressing = False
width  = 128    # change for different display size
height = 64     # change for different display size
pages = height // 8

# compatibility:
#    w,  h: DISP_CLK_DIV  COM_PIN_CFG
#  128, 64:         0x80         0x12
#  128, 32:         0x80         0x02
#   96, 16:         0x60         0x02
#   64, 48:         0x80         0x12
#   64, 32:         0x80         0x12
# Initialize commands for the min OLED


# initialized OLED settings
oled_init = [
    SET_DISP | 0x00,  # off
    # address setting
    SET_MEM_ADDR,
    0x10  # Page Addressing Mode
    if page_addressing
    else 0x00,  # Horizontal Addressing Mode
    # resolution and layout
    SET_DISP_START_LINE | 0x00,
    SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
    SET_MUX_RATIO,
    height - 1,
    SET_COM_OUT_DIR | 0x00,  # 0x08 scan from COM[N] to COM0
    SET_DISP_OFFSET,
    0x00,
    SET_COM_PIN_CFG,
    0x02
    if (height == 32 or height == 16) and (width != 64)
    else 0x12,
    # timing and driving scheme
    SET_DISP_CLK_DIV,
    0x80 if height !=16 else 0x60,
    SET_PRECHARGE,
    0x22 if external_vcc else 0xF1,
    SET_VCOM_DESEL,
    0x50,  # 0.83*Vcc  # n.b. specs for ssd1306 64x32 oled screens imply this should be 0x40
    # display
    SET_CONTRAST,
    0xFF,  # maximum
    SET_ENTIRE_ON | 0x00,  # output follows RAM contents
    SET_NORM_INV,  # not inverted
    # charge pump
    SET_CHARGE_PUMP,
    0x10 if external_vcc else 0x14,
    SET_DISP | 0x01,
]

class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""

class Timer:
    '''
    A timer (stop watch). I didn't write this and don't remember from where I copied it.
    '''
    def __init__(self):
        self._start_time = None

    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError('Timer is running. Use .stop() to stop it')
    
        self._start_time = time.perf_counter()
    
    def elapsedTime(self):
        """Report the elapsed time"""
        if self._start_time is None:
            raise TimerError('Timer is not running. Use .start() to start it')
        
        return time.perf_counter() - self._start_time
    
    def stop(self):
        """Stop the timer"""
        if self._start_time is None:
            raise TimerError('Timer is not running. Use .start() to start it')
            
        self._start_time = None


def set_clock(ftdi, f, div_by_5=True):
    """
    Set the ftdi clock frequency
    
    ftdi: ftdi handle
    f: frequency in Hz
    div_by_5: boolean to enable division by 5
    if div_by_5==True, frequency range: 91.553Hz -- 6MHz
    if div_by_5==False, frequency range: 457.763Hz -- 30MHz
    """
    if div_by_5:
        ftdi.write(bytes([DIV_5_ON]))
        div = round(12e6/(f*2)) - 1
    else:
        ftdi.write(bytes([DIV_5_OFF]))
        div = round(60e6/(f*2)) - 1    
    valueL = div%256
    valueH = div//256
    ftdi.write(bytes([SET_CLK, valueL, valueH]))
    
    return None
  
def I2C_start():
    """
    build byte sequence to start I2C protocol
    """
    packet = ([SET_LOW, 0b00000011, 0b11111011]*3 + 
              [SET_LOW, 0b00000001, 0b11111011]*3 + 
              [SET_LOW, 0b00000000, 0b11111011]*3)
    return packet

def I2C_stop():
    """
    build byte sequence to stop I2C protocol
    """
    packet = ([SET_LOW, 0b00000000, 0b11111011]*3 + 
              [SET_LOW, 0b00000001, 0b11111011]*3 + 
              [SET_LOW, 0b00000011, 0b11111011]*3)
    return packet

def I2C_send_Byte_read_ACK(Byte):
    """
    build byte sequence to send a byte and read acknowledgment over I2C
    """
    packet = ([MSB_W_BYTES_FALL,0,0,Byte] +
              [SET_LOW, 0b00000000, 0b11111001] +
              [MSB_R_BITS_RISE, 0] +
              [0x87] +
              [SET_LOW, 0b00000010, 0b11111011]*3)
    return packet

if __name__ == '__main__':

    script_path = os.path.dirname(os.path.realpath(__file__)) + '\\GIFs_128_64\\'  # path of the GIF file
    # open a GIF file (it needs to have same size as your OLED display, e.g., 128x64)
    media = Image.open(script_path + 'Telepurte__I_am_done_with_this__128x64.gif')
    
    # get list of connected devices
    if ftdi.listDevices() is None:
        raise ValueError('No FTDI devices connected!')
    else:
        inxs = [int.from_bytes(x) for x in ftdi.listDevices()]

    print('Connected device:')
    for inx in inxs:
        dev_info  = ftdi.getDeviceInfoDetail(inx)
        pprint.pprint(dev_info)
    
    # choose from the list if you have more
    print('\nFirst one is selected. (change manually!!)')
    dev = ftdi.open(inxs[0])
    try:
        dev.resetPort()
        dev.purge()
        print('\nFTDI OPEN...')
        print(dev.getDeviceInfo())
        
        dev.setBitMode(0x00, 0x02)   # set mode to MPSSE (support all the serial protocols, I2C, SPI, JTAG,... )
        dev.write(bytes([LOOPBACK_ON, 0x8C, 0x97]))  # LOOPBACK_ON connects D1 and D2 internally for I2C 
        dev.write(bytes([SET_LOW, 0b00000011, 0b11111011]))  # clear the pins values
        
        set_clock(dev, 3e6, True)  # I won't recommend going too high. 
        dev.setLatencyTimer(2)
        dev.setTimeouts(5000, 5000)
        
        ## initialize OLED over I2C
        cmds =  [byte for cmd in oled_init for byte in I2C_send_Byte_read_ACK(cmd)]
        dev.write(bytes(I2C_start() + 
                        I2C_send_Byte_read_ACK(0b01111000) + 
                        I2C_send_Byte_read_ACK(0x00) +
                        cmds +
                        I2C_stop()))
        
        ## set OLED screen size addressing format (refer to SSD1306 documentation)
        cmd_list = [SET_COL_ADDR, 0, width-1, SET_PAGE_ADDR, 0, pages-1]
        cmds     = [byte for cmd in cmd_list for byte in I2C_send_Byte_read_ACK(cmd)]
        dev.write(bytes(I2C_start() + 
                        I2C_send_Byte_read_ACK(0b01111000) + 
                        I2C_send_Byte_read_ACK(0x00) +
                        cmds +
                        I2C_stop()))
        
        # make frames black and white
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        frames = np.array([np.array(frame.copy().convert('L').point(fn, mode='1').getdata()).reshape(frame.size[1],frame.size[0]) for frame in PIL.ImageSequence.Iterator(media)])
        frames = frames/frames.max()
        
        #img = np.random.randint(0,2,size=(64,128)) # example of random image
        
        # extract all frames in the GIF file and restructure the values following paging style of SSD1306 OLED display
        pics = []
        for inx, img in enumerate(frames):
            img_bytes = np.kron(np.eye(pages), 2**np.arange(0,8))@img[::-1]  # flip it to match the disply orientation
            pics.append(img_bytes.astype(int).flatten().tolist())   # truncate values as 0s and 1s (black and white)
        
        frame_rate = 23.976 #200   # number of frames in one seconds (max depends on clock)
        
        timer = Timer()
        timer.start()
        
        while True:
            dev.stopInTask()
            time.sleep(1/frame_rate/20)  # delays seems to be necessary, otherwise it gets stuck
            dev.restartInTask()
            time.sleep(1/frame_rate/20)  
            dev.purge()  # clear the buffer
            time.sleep(1/frame_rate/20)
            
            for pic in pics:
                # slow down to match the frame-rate
                while timer.elapsedTime() < 1/frame_rate:
                    time.sleep(1/frame_rate/10)
                timer.stop()
                timer.start()
                
                oled_buffer = pic
                #oled_buffer = [np.random.randint(0,2**8) for i in range(width)]  # random image
                
                # send the frame to the display
                cmds =  [byte for cmd in oled_buffer for byte in I2C_send_Byte_read_ACK(cmd)]
                dev.write(bytes(I2C_start() + 
                                I2C_send_Byte_read_ACK(0b01111000) + 
                                I2C_send_Byte_read_ACK(0x40) +
                                cmds +
                                I2C_stop()))
                    
    except:
        time.sleep(0.1)
        dev.close()
        print('FTDI CLOSED.')

# EOF