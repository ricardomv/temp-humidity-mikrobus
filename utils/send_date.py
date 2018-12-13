import serial, time

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
    print('Current local time and date: ', end='')
    print(time.asctime(time.localtime()))
    ser.write(b'SET_DATE')
    ser.write(int(time.time()).to_bytes(4,'little'))



