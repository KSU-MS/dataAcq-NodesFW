#script for calibrating tire temp position
#this is a WIP
import serial
ser = serial.Serial('COM14')
ser.flushInput()
while True:
    try:
        ser_bytes = ser.readline()
        for i in ser_bytes:
            if i == '-':
                print("lol")
        print(ser_bytes)
    except:
        print("Keyboard Interrupt")
        break