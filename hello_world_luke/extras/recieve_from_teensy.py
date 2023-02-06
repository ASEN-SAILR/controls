#!/usr/bin/env python3
import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('COM12', 115200, timeout=1)
    ser.reset_input_buffer()
    while True:
        if ser.in_waiting>0:
            readdata = ser.readline()
            print(f"The py read \"{readdata}\" from the teensy")

        # if number != b'':
        #     if int.from_bytes(number, byteorder='big') == 18:
        #         led_number = random.randint(1,4)
        #         print("Button has been pressed.")
        #         print("Sending number " + str(led_number) + " to Arduino.")
        #         ser.write(str(led_number).encode('utf-8'))