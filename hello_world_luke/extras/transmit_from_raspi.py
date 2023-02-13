#!/usr/bin/env python3
import serial
import random
import time
import struct

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.reset_input_buffer()
    while True:
        #towrite = (input("enter command type to be writtent to teensy \nr: rotate \nt: translate \n s: stop \n m: magnetometer\n")).encode('utf-8')
        magnitude =float(input("enter magnitde for command\n"))
        #print(f"Writing \"{towrite}\" to the teensy.") 
        #print("toWrite", towrite)
        print("magnitude",magnitude)
        #ser.write(towrite)
        ser.write(b"\x01" + struct.pack("<f",magnitude))
        
        
        for i in range(5):
            while ser.in_waiting > 0:
                print(ser.readline())

        readdata = ser.readline().decode('utf-8').rstrip()
        print(f"The py read \"{readdata}\" from the teensy")

        # if number != b'':
        #     if int.from_bytes(number, byteorder='big') == 18:
        #         led_number = random.randint(1,4)
        #         print("Button has been pressed.")
        #         print("Sending number " + str(led_number) + " to Arduino.")
        #         ser.write(str(led_number).encode('utf-8'))
