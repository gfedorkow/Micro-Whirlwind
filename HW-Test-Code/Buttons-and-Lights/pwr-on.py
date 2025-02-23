
import RPi.GPIO as gpio

def main():

    pin_pwr_ctl = 35

    print("pwrwe control utility 1.0")

    gpio.setmode(gpio.BCM)
    gpio.setup(pin_pwr_ctl, gpio.OUT)

    while True:
        gpio.output(pin_pwr_ctl, 1)
        print("1")
        gpio.output(pin_pwr_ctl, 0)
        print("0")
    print("power control pin 35 set to one")

main()

