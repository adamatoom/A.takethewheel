
import serial
import time
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
time.sleep(55)

                     
ang =0
inc = 1     

while 1:

    
    ang = max(min(ang,200),-200)
    arduino.write('a'.encode('utf-8'))
    arduino.write(ang.to_bytes(2,byteorder = 'big', signed = True))
    # arduino.write(bytes([0]))
    print([int(max(min(ang,200),-200))])
    ang = ang + inc
    if ang ==200:
        inc = -1
    if ang ==-200:
        inc = 1
    time.sleep(1)

    

