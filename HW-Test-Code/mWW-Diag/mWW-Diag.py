#!/home/guyfe/ww-venv/bin/python

# Diagnostic for MicroWhirlwind hardware

RasPi = True
try:
    import RPi.GPIO as gpio
    # https://pypi.org/project/smbus2/
    import smbus2  # also contains i2c support
except ModuleNotFoundError:
    print("no GPIO library found")
    RasPi = False


import argparse
import time


IS31_1_ADDR_U1 = 0x74    # U1
IS31_1_ADDR_U5 = 0x75    # U5
IS31_1_ADDR_U2 = 0x77    # U2
TCA8414_0_ADDR = 0x34 # 34
TCA8414_1_ADDR = 0x3b #3B
pin_pwr_ctl = 19
pin_tca_reset = 26  # keyboard scanner reset pin; low for reset
pin_tca_interrupt  = 16

pin_gpio_LED1 = 5
pin_gpio_LED2 = 6
pin_gpio_LED3 = 20
pin_gpio_LED4 = 21
pin_gpio_isKey = 27


PassCount = 0

Verbose = False
Debug_I2C = False  # specific to low-level I2c
Debug = True

# ******************************************************************** #
# Interface to wwsim
# g fedorkow, Feb 2025

# see https://stackoverflow.com/questions/12681945/reversing-bits-of-python-integer
def bit_reverse_16(x):
    x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1)
    x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2)
    x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4)
    x = ((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8)
    return x    


class CpuClass:
    def __init__(self):
        self._BReg = 0
        self._AC = 0    # Accumulator
        self._AReg = 0    # Address Register, used for subroutine return address
        self._SAM = 0   # Two-bit carry-out register; it's only legal values appear to be 1, 0, -1
                        # SAM is set by many instructions, but used only by ca cs and cm
        self.PC = 0o40  # Program Counter; default start address


class mWWRegisterDisplayClass:
    def __init__(self, u1_is31, u2_is31, u5_is31):
        self.run_state = 0
        self.ind_register = 0   # this is the eight-bit "user" indicator light display
        self.u1_is31 = u1_is31
        self.u2_is31 = u2_is31
        self.u5_is31 = u5_is31


    # CPU run state is displayed in the most significant three bits of U1 Register 8
    def set_run_state(self, alarm, run_stop):
        if alarm:
            st = 0x8000
        else:
            st = 0
        if run_stop:
            st |= 0x4000
        else:
            st |= 0x2000
        self.run_state = (self.run_state & 0x1FFF) | st


    def set_IndReg_leds(self, ind_register):
        self.ind_register = bit_reverse_16(ind_register) & 0xff

    def set_cpu_reg_display(self, cpu, mdr=0, mar=0, mar_bank=0):
        u1_led = [0] * 9
        u5_led = [0] * 2
        acc_r = bit_reverse_16(cpu._AC)
        pc_r = bit_reverse_16(cpu.PC & 0o3777)
        b_reg_r = bit_reverse_16(cpu._BReg)
        mdr_par_r = bit_reverse_16(mdr)
        mar_r = bit_reverse_16(mar & 0o3777 | (mar_bank & 0o7) << 12)
        u1_led[0] = ~mar_r
        u1_led[1] = mar_r
        u1_led[2] = ~mdr_par_r
        u1_led[3] = mdr_par_r
        u1_led[4] = ~acc_r
        u1_led[5] = acc_r
        u1_led[6] = ~b_reg_r
        u1_led[7] = b_reg_r

        u5_led[0] = ~pc_r
        u5_led[1] = pc_r

        # Carry-Out / SAM register
        # Bit 8 -  0x100 -> red -1 carry
        # Bit 9 -  0x200 -> white -1 carry
        # Bit 10 - 0x400 -> red +1 carry
        # Bit 11 - 0x800 -> white +1 carry
        if cpu._SAM > 0:
            cled = 0x600
        elif cpu._SAM < 0:
            cled = 0x900 
        else:
            cled = 0xa00
        u1_led[8] = self.run_state | self.ind_register | cled

        self.u1_is31.is31.write_16bit_led_rows(0, u1_led)
        self.u5_is31.is31.write_16bit_led_rows(0, u5_led)



class MappedDisplayClass:
    def __init__(self, i2c_bus):
        self.cpu = CpuClass()
        self.mar = 0
        self.mdr = 0
        self.AC = 0
        self.BReg = 0
        self.PC = 0

        u1_is31 = Is31(i2c_bus, IS31_1_ADDR_U1)
        u5_is31 = Is31(i2c_bus, IS31_1_ADDR_U5)
        u2_is31 = Is31(i2c_bus, IS31_1_ADDR_U2)

        self.reg_disp = mWWRegisterDisplayClass(u1_is31, u2_is31, u5_is31)
        self.reg_disp.set_cpu_reg_display(self.cpu)  # default everything to zero
        self.bit_num = 0
        self.reg_num = 0

        self.reg_list = [
            [self.mar, 16],  # mar
            [self.mdr, 16],  # mdr
            [self.AC,  16],
            [self.BReg, 16],
            [self.PC, 16],
        ]
        self.max_reg = 5


    def step(self, delay):
        self.reg_list[self.reg_num][0] = 1 << self.bit_num
        self.bit_num += 1
        if self.bit_num >= self.reg_list[self.reg_num][1]:
            self.bit_num = 0
            self.reg_num = (self.reg_num + 1) % self.max_reg

        self.cpu.PC = self.reg_list[4][0]
        self.cpu._AC = self.reg_list[2][0]
        self.cpu._BReg = self.reg_list[3][0]
        self.cpu._SAM = 0

#        if self.reg_num == 0:  # MAR
        self.reg_disp.set_cpu_reg_display(self.cpu, mar=self.reg_list[0][0], mdr=self.reg_list[1][0])

#        if self.reg_num == 1:  # MDR
#            self.reg_disp.set_cpu_reg_display(self.cpu, mdr=(1 << self.bit_num))

        time.sleep(delay)


# ******************************************************************** #
# Classes to run the hardware I/O devices to make up the Micro-Whirlwind
# Guy Fedorkow, Jun 2024



class gpio_switches:
    def __init__(self):
        gpio.setmode(gpio.BCM)

        gpio.setup(pin_gpio_LED1, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(pin_gpio_LED2, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(pin_gpio_LED3, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(pin_gpio_LED4, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(pin_gpio_isKey, gpio.IN, pull_up_down=gpio.PUD_UP)
        self.count = 0
        self.last_sw_state = 0

    def getKeys(self):
        """ [from Rainer] Key inquiry for the push button on the interface board
        and the four switches on the tap board(s).
        Result in a bit pattern with LSB for the push button
        and the other four keys, left to right, with 2, 4, 8, and 16.
        Note that setKeys() can virtually set a key
        """
        res = 0
        if gpio.input(pin_gpio_isKey) == 0:
            res = 1
        if gpio.input(pin_gpio_LED1) == 0:
            res += 2
        if gpio.input(pin_gpio_LED2) == 0:
            res += 4
        if gpio.input(pin_gpio_LED3) == 0:
            res += 8
        if gpio.input(pin_gpio_LED4) == 0:
            res += 16
        return res

    def setKey(self, n, b):
        """
            [from Rainer] Set virtual key (i.e. the LED) on (1) or off (0)
        """
        if n == 0:
            return  # ignore key 0
        LEDs = [pin_gpio_LED1, pin_gpio_LED2, pin_gpio_LED3, pin_gpio_LED4]
        led = LEDs[n - 1]
        if b == 0:
            gpio.setup(led, gpio.IN, pull_up_down=gpio.PUD_UP)
            self.last_sw_state &= ~(1 << (n - 1))
        else:
            gpio.setup(led, gpio.OUT)
            gpio.output(led, 0)
            self.last_sw_state |= 1 << (n - 1)
        if Verbose: print("set last_sw_state to 0x%x" % self.last_sw_state)

    def step(self, delay):
        # Note that Rainer numbered the LEDs 1-4, not 0-3
        led = self.count & 3
        on = self.count & 4
        self.setKey(led + 1, on)
        self.count += 1

        sw = self.getKeys()
        if sw != self.last_sw_state:
            if Verbose: print("gpio switches: new state = 0x%x" % sw)
            self.last_sw_state = sw

        time.sleep(delay)




# =============== TCA8414 Keypad Scanner ==================================

class TCA8414:
    def __init__(self, bus, i2c_addr):
        self.bus = bus
        self.i2c_addr = i2c_addr
        self.TCA8418_REG_CFG = 0x01  # < Configuration register
        self.TCA8418_REG_INT_STAT = 0x02  # < Interrupt status
        self.TCA8418_REG_KEY_LCK_EC = 0x03  # < Key lock and event counter
        self.TCA8418_REG_KEY_EVENT_A = 0x04  # < Key event register A
        self.TCA8418_REG_KEY_EVENT_B = 0x05  # < Key event register B
        self.TCA8418_REG_KEY_EVENT_C = 0x06  # < Key event register C
        self.TCA8418_REG_KEY_EVENT_D = 0x07  # < Key event register D
        self.TCA8418_REG_KEY_EVENT_E = 0x08  # < Key event register E
        self.TCA8418_REG_KEY_EVENT_F = 0x09  # < Key event register F
        self.TCA8418_REG_KEY_EVENT_G = 0x0A  # < Key event register G
        self.TCA8418_REG_KEY_EVENT_H = 0x0B  # < Key event register H
        self.TCA8418_REG_KEY_EVENT_I = 0x0C  # < Key event register I
        self.TCA8418_REG_KEY_EVENT_J = 0x0D  # < Key event register J
        self.TCA8418_REG_KP_LCK_TIMER = 0x0E  # < Keypad lock1 to lock2 timer
        self.TCA8418_REG_UNLOCK_1 = 0x0F  # < Unlock register 1
        self.TCA8418_REG_UNLOCK_2 = 0x10  # < Unlock register 2
        self.TCA8418_REG_GPIO_INT_STAT_1 = 0x11  # < GPIO interrupt status 1
        self.TCA8418_REG_GPIO_INT_STAT_2 = 0x12  # < GPIO interrupt status 2
        self.TCA8418_REG_GPIO_INT_STAT_3 = 0x13  # < GPIO interrupt status 3
        self.TCA8418_REG_GPIO_DAT_STAT_1 = 0x14  # < GPIO data status 1
        self.TCA8418_REG_GPIO_DAT_STAT_2 = 0x15  # < GPIO data status 2
        self.TCA8418_REG_GPIO_DAT_STAT_3 = 0x16  # < GPIO data status 3
        self.TCA8418_REG_GPIO_DAT_OUT_1 = 0x17  # < GPIO data out 1
        self.TCA8418_REG_GPIO_DAT_OUT_2 = 0x18  # < GPIO data out 2
        self.TCA8418_REG_GPIO_DAT_OUT_3 = 0x19  # < GPIO data out 3
        self.TCA8418_REG_GPIO_INT_EN_1 = 0x1A  # < GPIO interrupt enable 1
        self.TCA8418_REG_GPIO_INT_EN_2 = 0x1B  # < GPIO interrupt enable 2
        self.TCA8418_REG_GPIO_INT_EN_3 = 0x1C  # < GPIO interrupt enable 3
        self.TCA8418_REG_KP_GPIO_1 = 0x1D  # < Keypad/GPIO select 1
        self.TCA8418_REG_KP_GPIO_2 = 0x1E  # < Keypad/GPIO select 2
        self.TCA8418_REG_KP_GPIO_3 = 0x1F  # < Keypad/GPIO select 3
        self.TCA8418_REG_GPI_EM_1 = 0x20  # < GPI event mode 1
        self.TCA8418_REG_GPI_EM_2 = 0x21  # < GPI event mode 2
        self.TCA8418_REG_GPI_EM_3 = 0x22  # < GPI event mode 3
        self.TCA8418_REG_GPIO_DIR_1 = 0x23  # < GPIO data direction 1
        self.TCA8418_REG_GPIO_DIR_2 = 0x24  # < GPIO data direction 2
        self.TCA8418_REG_GPIO_DIR_3 = 0x25  # < GPIO data direction 3
        self.TCA8418_REG_GPIO_INT_LVL_1 = 0x26  # < GPIO edge/level detect 1
        self.TCA8418_REG_GPIO_INT_LVL_2 = 0x27  # < GPIO edge/level detect 2
        self.TCA8418_REG_GPIO_INT_LVL_3 = 0x28  # < GPIO edge/level detect 3
        self.TCA8418_REG_DEBOUNCE_DIS_1 = 0x29  # < Debounce disable 1
        self.TCA8418_REG_DEBOUNCE_DIS_2 = 0x2A  # < Debounce disable 2
        self.TCA8418_REG_DEBOUNCE_DIS_3 = 0x2B  # < Debounce disable 3
        self.TCA8418_REG_GPIO_PULL_1 = 0x2C  # < GPIO pull-up disable 1
        self.TCA8418_REG_GPIO_PULL_2 = 0x2D  # < GPIO pull-up disable 2
        self.TCA8418_REG_GPIO_PULL_3 = 0x2E  # < GPIO pull-up disable 3
        # #define TCA8418_REG_RESERVED          0x2F

        # FIELDS CONFIG REGISTER  1

        self.TCA8418_REG_CFG_AI = 0x80  # < Auto-increment for read/write
        self.TCA8418_REG_CFG_GPI_E_CGF = 0x40  # < Event mode config
        self.TCA8418_REG_CFG_OVR_FLOW_M = 0x20  # < Overflow mode enable
        self.TCA8418_REG_CFG_INT_CFG = 0x10  # < Interrupt config
        self.TCA8418_REG_CFG_OVR_FLOW_IEN = 0x08  # < Overflow interrupt enable
        self.TCA8418_REG_CFG_K_LCK_IEN = 0x04  # < Keypad lock interrupt enable
        self.TCA8418_REG_CFG_GPI_IEN = 0x02  # < GPI interrupt enable
        self.TCA8418_REG_CFG_KE_IEN = 0x01  # < Key events interrupt enable

        # FIELDS INT_STAT REGISTER  2
        self.TCA8418_REG_STAT_CAD_INT = 0x10  # < Ctrl-alt-del seq status
        self.TCA8418_REG_STAT_OVR_FLOW_INT = 0x08  # < Overflow interrupt status
        self.TCA8418_REG_STAT_K_LCK_INT = 0x04  # < Key lock interrupt status
        self.TCA8418_REG_STAT_GPI_INT = 0x02  # < GPI interrupt status
        self.TCA8418_REG_STAT_K_INT = 0x01  # < Key events interrupt status

        # FIELDS  KEY_LCK_EC REGISTER 3
        self.TCA8418_REG_LCK_EC_K_LCK_EN = 0x40  # < Key lock enable
        self.TCA8418_REG_LCK_EC_LCK_2 = 0x20  # < Keypad lock status 2
        self.TCA8418_REG_LCK_EC_LCK_1 = 0x10  # < Keypad lock status 1
        self.TCA8418_REG_LCK_EC_KLEC_3 = 0x08  # < Key event count bit 3
        self.TCA8418_REG_LCK_EC_KLEC_2 = 0x04  # < Key event count bit 2
        self.TCA8418_REG_LCK_EC_KLEC_1 = 0x02  # < Key event count bit 1
        self.TCA8418_REG_LCK_EC_KLEC_0 = 0x01  # < Key event count bit 0

    def writeRegister(self, command, val):
        if Debug_I2C: print("writeRegister: cmd=%x val=%x" % (command, val))
        self.bus.write_byte_data(self.i2c_addr, command, val)

    def readRegister(self, command):
        val = self.bus.read_byte_data(self.i2c_addr, command)
        if Debug_I2C: print("readRegister: cmd=%x val=%x" % (command, val))
        return val

    def i2c_reg_test(self):
        val = 0x55
        self.writeRegister(self.TCA8418_REG_GPIO_DIR_1, val)
        nval = self.readRegister(self.TCA8418_REG_GPIO_DIR_1)
        print(" I2C=0x%0x, Reg %x: val=%x, read=%x" % 
            (self.i2c_addr, self.TCA8418_REG_GPIO_DIR_1, val, nval))

    def init_tca8414(self, rows, columns):
        self.writeRegister(self.TCA8418_REG_CFG, 0x20)   # Interrupt for buffer overflow
        val = self.readRegister(self.TCA8418_REG_CFG)
        if val == 0x20:
            print("TCA8414 at 0x%x config_reg = 0x%x" % (self.i2c_addr, val))
        else:
            print("TCA8414 at 0x%x failed; val should be 0x20, is 0x%x" % (self.i2c_addr, val))

        #  GPIO
        #  set default all GIO pins to INPUT
        self.writeRegister(self.TCA8418_REG_GPIO_DIR_1, 0x00)
        self.writeRegister(self.TCA8418_REG_GPIO_DIR_2, 0x00)
        self.writeRegister(self.TCA8418_REG_GPIO_DIR_3, 0x00)

        #  add all pins to key events
        self.writeRegister(self.TCA8418_REG_GPI_EM_1, 0xFF)
        self.writeRegister(self.TCA8418_REG_GPI_EM_2, 0xFF)
        self.writeRegister(self.TCA8418_REG_GPI_EM_3, 0x00)

        #  set all pins to FALLING interrupts
        self.writeRegister(self.TCA8418_REG_GPIO_INT_LVL_1, 0x00)
        self.writeRegister(self.TCA8418_REG_GPIO_INT_LVL_2, 0x00)
        self.writeRegister(self.TCA8418_REG_GPIO_INT_LVL_3, 0x00)

        #  add all pins to interrupts
        self.writeRegister(self.TCA8418_REG_GPIO_INT_EN_1, 0xFF)
        self.writeRegister(self.TCA8418_REG_GPIO_INT_EN_2, 0xFF)
        self.writeRegister(self.TCA8418_REG_GPIO_INT_EN_3, 0xFF)

        self.matrix(rows, columns)

    """ from: /**
     *  @file Adafruit_TCA8418.cpp
     *
     * 	I2C Driver for the Adafruit TCA8418 Keypad Matrix / GPIO Expander Breakout
     *
     * 	This is a library for the Adafruit TCA8418 breakout:
     * 	https://www.adafruit.com/product/XXXX
     *

    /**
     * @brief configures the size of the keypad matrix.
     *
     * @param [in] rows    number of rows, should be <= 8
     * @param [in] columns number of columns, should be <= 10
     * @return true is rows and columns have valid values.
     *
     * @details will always use the lowest pins for rows and columns.
     *          0..rows-1  and  0..columns-1
     */
    """

    def matrix(self, rows, columns):
        if (rows > 8) or (columns > 10):
            return False

        # skip zero size matrix
        if (rows != 0) and (columns != 0):
            # set up the keypad matrix.
            mask = 0x00
            for r in range(0, rows):
                mask <<= 1
                mask |= 1
            self.writeRegister(self.TCA8418_REG_KP_GPIO_1, mask)
            print("matrix GPIO_1 set to 0x%x" % mask)

            mask = 0x00
            for c in range(0, 8):  # (int c = 0; c < columns && c < 8; c++) {
                if c < columns:
                    mask <<= 1
                    mask |= 1
            self.writeRegister(self.TCA8418_REG_KP_GPIO_2, mask)
            print("matrix GPIO_2 set to 0x%x" % mask)

            mask = 0
            if columns > 8:
                if columns == 9:
                    mask = 0x01
                else:
                    mask = 0x03
            self.writeRegister(self.TCA8418_REG_KP_GPIO_3, mask)
            print("matrix GPIO_3 set to 0x%x" % mask)
        return True

    # hack -- config two pins as output to blink an LED
    def init_gp_out(self):
        # column 8 and 9 pins
        self.writeRegister(self.TCA8418_REG_GPIO_DIR_3, 0x3)

    def set_gp_out(self, val):
        self.writeRegister(self.TCA8418_REG_GPIO_DAT_OUT_3, val)

    """ ... from Adafruit
    /**
     * @brief flushes the internal buffer of key events
     *        and cleans the GPIO status registers.
     *
     * @return number of keys flushed.
     */    """

    def flush(self):
        count = 0
        while self.getEvent() != 0:
            count += 1
        #  flush gpio events
        self.readRegister(self.TCA8418_REG_GPIO_INT_STAT_1)
        self.readRegister(self.TCA8418_REG_GPIO_INT_STAT_2)
        self.readRegister(self.TCA8418_REG_GPIO_INT_STAT_3)
        #  //  clear INT_STAT register
        self.writeRegister(self.TCA8418_REG_INT_STAT, 3)
        return count

    """
    /**
     * @brief gets first event from the internal buffer
     *
     * @return key event or 0 if none available
     *
     * @details
     *     key event 0x00        no event
     *               0x01..0x50  key  press
     *               0x81..0xD0  key  release
     *               0x5B..0x72  GPIO press
     *               0xDB..0xF2  GPIO release
     */
    """
    def getEvent(self):
        return self.readRegister(self.TCA8418_REG_KEY_EVENT_A)

    """
    /**
     * @brief checks if key events are available in the internal buffer
     *
     * @return number of key events in the buffer
     */
    """

    def available(self):
        eventCount = self.readRegister(self.TCA8418_REG_KEY_LCK_EC)
        eventCount &= 0x0F  # //  lower 4 bits only
        return eventCount


# =============== Rotary Encoder Test ==================================
# The following routine decodes the two signals from a Rotary Encoder to
# figure out which way the knob is being turned.
# The code relies on the key scanner to deliver an 'interrupt' when the pin
# changes state.
# We're also relying completely on the scanner to debounce the signals!
EncoderState = [0,0]
# 'which_key' is which one of the two Rotary phases changed.
def rotary_decode(pressed, which_key):
    global EncoderState

    EncoderState[which_key] = (pressed != 0)
    push_str = "Released"
    if pressed:
        push_str = "Pushed  "

    direction = 33
    if pressed:
        if which_key == 0:
            direction = EncoderState[1]
        if which_key == 1:
            direction = EncoderState[0] == 0
    else:
        if which_key == 0:
            direction = EncoderState[1] == 0
        if which_key == 1:
            direction = EncoderState[0]

    print("%s: key=%d dir=%d" % (push_str, which_key, direction))



# =============== IS31FL3731 LED Mux ==================================
    # This class was derived from an Adafruit example
class IS31FL3731:
    def __init__(self, i2c_bus, i2c_addr):
        self.bus = i2c_bus.bus
        self.i2c_addr = i2c_addr
        # converted from Adafruit library
        self.ISSI_REG_CONFIG = 0x00
        self.ISSI_REG_CONFIG_PICTUREMODE = 0x00
        self.ISSI_REG_CONFIG_AUTOPLAYMODE = 0x08
        self.ISSI_REG_CONFIG_AUDIOPLAYMODE = 0x18
        self.ISSI_CONF_PICTUREMODE = 0x00
        self.ISSI_CONF_AUTOFRAMEMODE = 0x04
        self.ISSI_CONF_AUDIOMODE = 0x08
        self.ISSI_REG_PICTUREFRAME = 0x01
        self.ISSI_REG_SHUTDOWN = 0x0A
        self.ISSI_REG_AUDIOSYNC = 0x06
        self.ISSI_COMMANDREGISTER = 0xFD
        self.ISSI_BANK_FUNCTIONREG = 0x0B  # helpfully called 'page nine'

    # this routine is used to transmit an easy-to-recognize pattern on
    # the I2C bus, for watching with a logic analyzer.  It doesn't make
    # the display do anything...
    def i2c_bus_test(self):
        msg = [2]
        self.bus.write_i2c_block_data(self.i2c_addr, 1, msg)
        msg = [4]
        self.bus.write_i2c_block_data(self.i2c_addr, 3, msg)
        msg = [6, 7]
        self.bus.write_i2c_block_data(self.i2c_addr, 5, msg)
        msg = [0xa, 0xb, 0xc]
        self.bus.write_i2c_block_data(self.i2c_addr, 9, msg)


    # write an IS31 control register.  Start by selecting "Page 9", the one that
    # has control registers instead of pixels.
    def writeRegister8(self, register, command, val=None):
        global PassCount
        print("%05d: writeRegister: reg=%x, cmd=%x " % (PassCount, register, command), "val=", val)
        msg = [register]
        self.bus.write_i2c_block_data(self.i2c_addr, self.ISSI_COMMANDREGISTER, msg)

        if val is not None:
            msg = [val]
        else:
            msg = []
        self.bus.write_i2c_block_data(self.i2c_addr, command, msg)


    def writeMultiRegister(self, register, val_list):
        #print("writeMultiRegister: reg=%x, " % (register), "val=", val_list)
        self.bus.write_i2c_block_data(self.i2c_addr, self.ISSI_COMMANDREGISTER, [0])
        self.bus.write_i2c_block_data(self.i2c_addr, register, val_list)


    def selectFrame(self, frame):
        msg = [frame]
        self.bus.write_i2c_block_data(self.i2c_addr, self.ISSI_COMMANDREGISTER, msg)

    def displayFrame(self, frame):
        self.writeRegister8(self.ISSI_BANK_FUNCTIONREG, self.ISSI_REG_PICTUREFRAME, val=frame)

    def set_brightness(self, bright):
        #for (uint8_t i = 0; i < 6; i++) {
        #  erasebuf[0] = 0x24 + i * 24;
        #  _i2c_dev->write(erasebuf, 25);
        #}
        IS31_LEDS = 192
        I2C_BLOCK = 24
        val = [bright] * I2C_BLOCK

        print("set brightness to %x" % bright)
        for i in range(0, IS31_LEDS // I2C_BLOCK):
            self.writeMultiRegister(i*I2C_BLOCK + 0x24, val)   # each 8 LEDs on (off)
        onoff = [0x5C] * (IS31_LEDS // 8)  # 8 bits per byte of on/off status
        print("set on-status to ", onoff)
        # for i in range(0, 18):
        self.writeMultiRegister(0, onoff)   # each 8 LEDs on (off)

    def write_16bit_led_rows(self, row, int_list):
        byte_list = []
        for val in int_list:
            byte_list.append(val & 0xff)
            byte_list.append(val >> 8)
        #self.writeMultiRegister(row * 2, byte_list)   # 16 bits in two bytes
        self.bus.write_i2c_block_data(self.i2c_addr, row * 2, byte_list)


    def init_IS31(self):
        _frame = 0
        # shutdown
        print("Shutdown")
        self.writeRegister8(self.ISSI_BANK_FUNCTIONREG, self.ISSI_REG_SHUTDOWN, val=0x00)
        time.sleep(0.01)

        # out of shutdown
        print("unShutdown")
        self.writeRegister8(self.ISSI_BANK_FUNCTIONREG, self.ISSI_REG_SHUTDOWN, val=0x01)
        #time.sleep(1)

        # picture mode
        print("picture mode")
        self.writeRegister8(self.ISSI_BANK_FUNCTIONREG, self.ISSI_REG_CONFIG,
                 val=self.ISSI_REG_CONFIG_PICTUREMODE)

        #time.sleep(1)
        print("display frame")
        self.displayFrame(_frame)

        #time.sleep(1)
        print("set brightness")
        # all LEDs to the same brightness, and turn them all off
        self.set_brightness(0x04)  # Red should be 16; white is less




# --------------------
# the remainder of this file gives a standalone test program that exercises
# the hardware (currently the IS31 LED driver)
# July 7, 2024

class PingPongStruct():
    def __init__(self, i):
        print("pong_struct preset = ", i)

        self.delay_count = 0
        self.delay_preset = i
        self.incr = 1
        self.invert = 0
        self.val = 0

    def pingpong(self):
        self.delay_count -= 1
        if self.delay_count < 0:
            self.delay_count = self.delay_preset

            self.val += self.incr
            if self.val < 0:
                self.val = 1
                self.incr = 1
                self.invert = 0

            if self.val > 15:
                self.val = 14
                self.incr = -1
                self.invert = ~self.invert

        return((1 << self.val) ^ self.invert)


_NPONGS = 9
def _init_pongs():
    pp = []
    for i in range(0, _NPONGS):
        pp.append(PingPongStruct(i))
    return pp


# =============== Devices and Tests ==================================

class I2C:
    def __init__(self, bus_number):
        print("I2C init bus #%d: " % bus_number)
        # bus = I2Cclass(channel = 1)
        self.bus = smbus2.SMBus(bus_number)
        print("  done")
        self.test_step = 0

    def writeRegister(self, i2c_addr, command, val):
        if Debug_I2C: print("writeRegister: addr=%x, cmd=%x val=%x" % (i2c_addr, command, val))
        self.bus.write_byte_data(i2c_addr, command, val)

    def readRegister(self, i2c_addr, command):
        val = self.bus.read_byte_data(i2c_addr, command)
        if Debug_I2C: print("readRegister:  addr=%x, cmd=%x val=%x" % (i2c_addr, command, val))
        return val

    def i2c_reg_test(self, addr_str):
        addr = int(addr_str, 16)
        val = 0x55
        self.writeRegister(addr, val, 0)
        nval = self.readRegister(addr, 0)
        print(" Reg %x: val=%x, read=%x" % (addr, val, nval))


class PwrCtl:
    def __init__(self):
        self.pwr_state: int = 0

    def pwr_on(self) -> None:
        global pin_pwr_ctl, pin_tca_reset, pin_tca_interrupt

        print("power control utility 1.1a")
        self.pwr_state = 1

        gpio.setmode(gpio.BCM)
        gpio.setup(pin_pwr_ctl, gpio.OUT)
        gpio.setup(pin_tca_reset, gpio.OUT)
        gpio.setup(pin_tca_interrupt, gpio.IN, pull_up_down=gpio.PUD_UP)

        gpio.output(pin_pwr_ctl, self.pwr_state)
        gpio.output(pin_tca_reset, 0)  # turn on the reset signal
        time.sleep(0.3)
        gpio.output(pin_tca_reset, 1)  # Enable the key scanners

        print("power control pin %d set to one" % pin_pwr_ctl)


    def step(self):
        self.pwr_state ^= 1  # flip the power state pin
        gpio.output(pin_pwr_ctl, self.pwr_state)




# initialize a test instance for LED patterns.
class Is31:
    def __init__(self, i2c_bus, addr, reverse_bits=False):
        self.is31 = IS31FL3731(i2c_bus, addr)
        self.i2c_bus = i2c_bus
        print("IS31 at 0x%0x Init" % addr)
        self.addr = addr
        # is31.i2c_reg_test()
        self.is31.init_IS31()
        self.is31.selectFrame(0)   # do this once, so it doesn't have to be done with each write of the LEDs
        self.is31.write_16bit_led_rows(0, [0, 0, 0, 0, 0, 0, 0, 0, 0])  # clear all the LEDs
        print("  IS31 init done")
        self.test_step = 0
        self.previous_test_step = 0
        self.previous_word_offset = 0
        self.bits_on = 1
        self.test_state = [0] * 18  # up to nine 16-bit registers
        self.exclusion = None
        self.register_range = 9     # default highest register number
        self.swap_size = 16
        self.reverse_bits = reverse_bits

        # there are some unpopulated LEDs that "shouldn't" be turned on to avoid a sneak-path
        # Each LED driver has a different list...
        self.u1_exclusion = {0: 0b10001, 1: 0b10001, 8: 0x1000}
        self.u5_exclusion = {0: 0b10001, 1: 0b10001}
        self.u2_exclusion = {}
        if addr == 0x74:   # LED driver U1
            self.exclusion =  self.u1_exclusion
            print("U1 Exclusion List")
        if addr == 0x75:   # LED driver U5
            self.exclusion =  self.u5_exclusion
            self.register_range = 6
            print("U5 Exclusion List")
        if addr == 0x77:   # LED driver U2
            self.exclusion =  self.u2_exclusion
            self.swap_size = 1
            print("U2 Exclusion List")



    # bit-reverse a 16-bit word
    def bit_reverse(self, word):
        wsize = self.swap_size
        new_word = 0
        if wsize == 16:        # reverse the bits in a 16-bit word
            for i in range(0, wsize):
                new_word |= (word >> i) & 1
                if i < wsize - 1:
                    new_word = new_word << 1
        elif wsize == 8:        # reverse the bits in two 8-bit bytes
            for i in range(0, wsize):
                new_word |= (word >> i) & 0x101
                if i < wsize - 1:
                    new_word = new_word << 1
        elif wsize == 1:       # swap bytes, leave bits in the same order
            new_word = ((word << 8) & 0xff00) | ((word >> 8) & 0xff)
        else:
            print("unexpected word length %d in bit_reverse" % wsize)
        return new_word



    def swap_bits_and_write_word(self, reg_offset, word_reg):
        if self.reverse_bits:
            reversed_word = self.bit_reverse(word_reg)
            if self.exclusion:
                if reg_offset in self.exclusion:
                    exc = self.exclusion[reg_offset]
                    if (exc & reversed_word) != 0:
                        print("Exclusion: Reg 0x%x, 0x%x" % (reg_offset, self.exclusion[reg_offset]))
                    reversed_word &= ~exc
            int_list = [reversed_word]
        else:
            int_list = [word_reg]
        self.is31.write_16bit_led_rows(reg_offset, int_list)



    def step(self, delay, pattern):
        global PassCount

        if pattern == 0 or pattern == 1:
            word_offset = self.test_step >> 4
            bit_num = self.test_step & 0xf
            word_reg = self.test_state[word_offset]  # Last-written state of the registers
        else:  # Patterns two and three blink adjacent LEDs, 2 is 'horizontaly adjacent', 3 is 'vertically adjacent'
            pattern23_addr = pattern >> 4
            pattern &= 0xf    # convert this into "pattern 2 or 3", i.e., separate command and address
            if pattern == 2:
                word_offset = pattern23_addr >> 4
                bit_num = pattern23_addr & 0xe   # round to the next lower even number
            elif pattern == 3:
                word_offset = (pattern23_addr >> 4) & 0xe   # in Pattern Three, we blink corresponding bits in adjacent registers
                bit_num = pattern23_addr & 0xf
            else:
                print("unexpected pattern")


        # with the Wave pattern, we just turn on bits one at a time until they're all on, then turn them off one at a time
        if pattern == 0:
            if self.bits_on:
                word_reg |= 1 << bit_num
            else:
                word_reg &= ~(1 << bit_num)

        elif pattern == 1:         # The one-hot pattern turns LEDs on one a time but turns off the previous one first
            word_offset = self.previous_test_step >> 4
            word_reg = 0
            self.swap_bits_and_write_word(word_offset,  word_reg)  # turn off the previous bit
            # int_list = [word_reg]  # turn off the last bit
            # self.is31.write_16bit_led_rows(word_offset, int_list)

            word_offset = self.test_step >> 4
            bit_num = self.test_step & 0xf
            word_reg = 1 << bit_num

        elif pattern == 2:    # blink back and forth between LEDs 0 and 1 of the register
            if self.test_step == 0:
                word_reg = 1 << bit_num
            else:
                word_reg = 2 << bit_num

        elif pattern == 3:    # blink back and forth between LEDs in two registers
            word_reg = 0
            self.swap_bits_and_write_word(word_offset,  word_reg)  # turn off the previous bit
            # int_list = [word_reg]  # turn off the last bit
            # self.is31.write_16bit_led_rows(self.previous_word_offset, int_list)

            word_reg = 1 << bit_num
            if self.test_step > 0:
                word_offset +=1

        else:
            print("Unexpected blink pattern %d" % pattern)

        if Verbose: print("%05d: write LED reg %d at 0x%x to 0x%x" % (PassCount, word_offset, self.addr, word_reg))
        self.test_state[word_offset] = word_reg

        self.swap_bits_and_write_word(word_offset,  word_reg)  # turn off the previous bit
        # int_list = [word_reg]  # pass in a list of one word
        # self.is31.write_16bit_led_rows(word_offset, int_list)


        restart_cycle = False
        self.previous_test_step = self.test_step
        self.previous_word_offset = word_offset
        if pattern == 0 or pattern == 1:
            # Pattern generator - sweep across all bits turning them on one a time, then
            # turn them all off one at a time, or turn on one at a time
            self.test_step += 1
            if self.test_step >= 16 * self.register_range:
                self.test_step = 0
                self.bits_on = self.bits_on ^ 1
                restart_cycle = True
        elif pattern == 2 or pattern == 3:   # alternate two LEDs
            self.test_step += 1
            if self.test_step > 1:
                self.test_step = 0
        else:
            print("unexpected blink pattern %d" % pattern)

        time.sleep(delay)
        return restart_cycle


def tc_init_1(bus, addr):
    tca84 = TCA8414(bus, addr)
    print("TCA8414 @0x%x U4 Test" % tca84.i2c_addr)
    # tca84.i2c_reg_test()
    tca84.init_tca8414(8, 4)  # scan 8 rows, 4 columns
    # tca84.init_gp_out()
    # flush the internal buffer
    tca84.flush()
    print("  TCA8414 U4 at 0x%x init done" % tca84.i2c_addr)
    return tca84


def tc_init_0(bus, addr):
    tca84 = TCA8414(bus, addr)
    print("TCA8414 @0x%x U3 Test" % tca84.i2c_addr)
    # tca84.i2c_reg_test()
    tca84.init_tca8414(8, 10) #was 8  # scan 8 rows, 10 columns
    # tca84.init_gp_out()  # hack test
    # flush the internal buffer
    tca84.flush()
    print("  TCA8414 U3 at 0x%x init done" % tca84.i2c_addr)
    return tca84


def run_pong(is31):
    pp = _init_pongs()
    int_val = [0] * _NPONGS
    i = 0
    while True:
        for j in range(0, _NPONGS):
            int_val[j] = pp[j].pingpong()
        is31.write_16bit_led_rows(0, int_val)


def run_tca(tca84):
    if tca84.available() > 0:
        key = tca84.getEvent()
        pressed = key & 0x80
        key &= 0x7F
        if (key == 111 or key == 112) and tca84.i2c_addr == TCA8414_1_ADDR:
            rotary_decode(pressed, key - 111)  # the two codes come in as 111 and 112

        else:
            key -= 1
            row = key // 10
            col = key % 10
            push_str = "Released"
            if pressed:
                push_str = "Pressed "
            print("%s: I2C=0x%0x, key=%d; row=%d, col=%d" % (push_str, tca84.i2c_addr, key + 1, row, col))


# =============== Main ==================================

def main():
    global PassCount, Verbose
    parser = argparse.ArgumentParser(description='Diagnostic for MicroWhirlwind PCB')
    parser.add_argument("-v", "--Verbose", help="Print lots of chatter", action="store_true")
    parser.add_argument("-d", "--Delay", help="wait time between steps", type=str)
    parser.add_argument("-g", "--GPIO_Switches", help="Activate GPIO switches/LEDs", action="store_true")
    parser.add_argument("--NoPowerControl", help="Don't mess with power control", action="store_true")
    parser.add_argument("--PowerControl_Loop", help="Loop power control on/off", action="store_true")
    parser.add_argument("--SMBus_Loop", help="Loop on SMBus addr read/write test", type=str) # hex number
    parser.add_argument("--U1_LED_Mux_Loop", help="Exercise LED Mux chip @ 0x74", action="store_true")
    parser.add_argument("--U5_LED_Mux_Loop", help="Exercise LED Mux chip @ 0x75", action="store_true")
    parser.add_argument("--U2_LED_Mux_Loop", help="Exercise LED Mux chip @ 0x77", action="store_true")
    parser.add_argument("--Key_0_Scan", help="Exercise Key Scanner chip @ 0x?", action="store_true")
    parser.add_argument("--Key_1_Scan", help="Exercise Key Scanner chip @ 0x?", action="store_true")
    parser.add_argument("--P0_wave", help="LED Pattern - Wave", action="store_true")
    parser.add_argument("--P1_hot", help="LED Pattern - One Hot scan", action="store_true")
    parser.add_argument("--P2H_blink", help="LED Pattern - Alternate two horizontal LEDs at hex-addr", type=str)
    parser.add_argument("--P3V_blink", help="LED Pattern - Alternate two vertical LEDs at hex-addr", type=str)
    parser.add_argument("-m", "--Mapped", help="Scan CPU State", action="store_true")

    args = parser.parse_args()

    tests = 0
    stop = False
    is31_U1 = None
    is31_U5 = None
    is31_U2 = None
    tca84_0 = None
    tca84_1 = None
    pwr_ctl = PwrCtl()

    pass_count = 0

    # not much can happen if we can't initialize the i2c bus...
    i2c_bus = I2C(1)
    bus = i2c_bus.bus
    gp_sw = gpio_switches()  # these are the switches and LEDs on Rainer's "Tap Board"

    if args.Verbose:
        Verbose = True

    if args.Delay:  # set the length of the delay time for each test step
        delay = float(args.Delay)
    else:
        delay = 0.04  # seconds

    if args.NoPowerControl is False:
        pwr_ctl.pwr_on()
        time.sleep(0.3)

    # Hack
    # args.U1_LED_Mux_Loop = True

    if args.SMBus_Loop:
        tests +=1

    if args.U1_LED_Mux_Loop:
        is31_U1 = Is31(i2c_bus, IS31_1_ADDR_U1)
        tests += 1

    if args.U5_LED_Mux_Loop:
        is31_U5 = Is31(i2c_bus, IS31_1_ADDR_U5)
        tests += 1

    if args.U2_LED_Mux_Loop:
        is31_U2 = Is31(i2c_bus, IS31_1_ADDR_U2)
        tests += 1

    if args.Key_0_Scan:
        tca84_0 = tc_init_0(bus, TCA8414_0_ADDR)
        tests += 1

    if args.Key_1_Scan:
        tca84_1 = tc_init_1(bus, TCA8414_1_ADDR)
        tests += 1

    # These test options don't need specific intialization
    if args.PowerControl_Loop or args.GPIO_Switches:
        tests += 1

    if args.Mapped:
        md = MappedDisplayClass(i2c_bus)
        tests += 1

    if tests == 0:
        print("no tests?")
        exit(1)

    led_pattern = 1    # default is one-hot
    if args.P0_wave:
        led_pattern = 0
    if args.P1_hot:
        led_pattern = 1
    if args.P2H_blink:
        try:
            register = int(args.P2H_blink, 16)
        except ValueError: 
            print("can't decode hex num %s" % args.P2H_blink)
        led_pattern = 2 | (register << 4)
    if args.P3V_blink:
        try:
            register = int(args.P3V_blink, 16)
        except ValueError: 
            print("can't decode hex num %s" % args.P3V_blink)
        led_pattern = 3 | (register << 4)



    while not stop:
        if args.PowerControl_Loop:
            pwr_ctl.step()
        if args.SMBus_Loop:
            i2c_bus.i2c_reg_test(args.SMBus_Loop)
        if args.U1_LED_Mux_Loop:
            is31_U1.step(delay, led_pattern)
        if args.U5_LED_Mux_Loop:
            is31_U5.step(delay, led_pattern)
        if args.U2_LED_Mux_Loop:
            is31_U2.step(delay, led_pattern)
        if args.Key_0_Scan:
            run_tca(tca84_0)
        if args.Key_1_Scan:
            run_tca(tca84_1)
        if args.GPIO_Switches:
            gp_sw.step(delay)
        if args.Mapped:
            md.step(delay)


#           run_pong(is31_U2)
        PassCount += 1
        # print("%05d" % PassCount)

    input("CR to Shutdown")
    # is31.writeRegister8(ISSI_BANK_FUNCTIONREG, ISSI_REG_SHUTDOWN, val=0x00)
    time.sleep(1)

""" 
class BlinkenLights:
    def __init__(self):
        print("I2C init: ")
        # bus = I2Cclass(channel = 1)
        self.i2c_bus = smbus2.SMBus(1)
        print("  done")

        self.is31_U5 = IS31FL3731(self.i2c_bus, IS31_1_ADDR)
        print("I2C Test")
        # is31.i2c_reg_test()
        self.is31_1.init_IS31()
        print("  IS31 init done")

    def update_panel(self, cb, bank, alarm_state=0, standalone=False, init_PC=None):
        cpu = cb.cpu
        lights = []
        lights.append(cpu.PC)
        lights.append(cpu._AC)
        lights.append(cpu._BReg)
        self.is31_1.write_16bit_led_rows(0, lights)
"""


if __name__ == "__main__":
    main()
