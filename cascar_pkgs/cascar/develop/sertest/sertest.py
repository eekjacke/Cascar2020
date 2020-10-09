import serial
from time import sleep

ser = serial.Serial('/dev/ttyUSB1', 115200)
ser.get_settings()
ser.write('T;50\rS;20\r')

f = 20.0
steer = -100
while True:
    ser.flush()
    n = ser.write('S;%d\r' % steer)
    sleep(1.0/f)


ser.close()


SER_PORTS = ['/dev/ttyUSB1', '/dev/ttyUSB0']
SER_RATE = 115200

def OpenCarSerialConnection():
    for p in SER_PORTS:
        ser = serial.Serial(p, SER_RATE)
        ser.flush()
        print "Waiting for serial port " + p + " to wake up ... "
        sleep(2) # Why do this wait has to be so long?
        ser.write('PING;\r')
        sleep(0.05)
        foundSer = False
        msg = ''
        while ser.inWaiting()>0:
            print msg
            msg = ser.readline()
        msg = msg.split(';')[0]
        if msg == 'PONG':
            foundSer = True
        
        if not foundSer:
            print 'Car NOT connected to ' + ser.name
            ser.close()
        else:
            print 'Car connected to ' + ser.name
            return ser

    return None


ser = serial.Serial(ports[0], 115200)
ser.write('PING;\r')
print ser.inWaiting()
ser.close()


while True:
    msg = ser.readline()
    print msg
