
import serial
import time
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
time.sleep(55)

                     
ang =0
inc = 1     

while 1:

    
    ang = max(min(ang,255),0)
    arduino.write(bytes([int(max(min(ang,255),0))]))
    # arduino.write(bytes([0]))
    print([int(max(min(ang,255),0))])
    ang = ang + inc
    if ang ==255:
        inc = -1
    if ang ==0:
        inc = 1
    time.sleep(1)

    

