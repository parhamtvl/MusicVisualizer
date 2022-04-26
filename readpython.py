
import serial
ser=serial.Serial("/dev/ttyS0",115200)

while True:
    bytesToRead = ser.inWaiting()
    #ser.read(bytesToRead)
    read_ser = ser.readline()
    print(read_ser)
