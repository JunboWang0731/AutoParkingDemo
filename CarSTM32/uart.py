#!/usr/bin/env python3
import serial
import serial.tools.list_ports
from time import sleep

def recv(serial):
    while True:
        data = serial.read_all()
        if data == '':
            continue
        else:
            break
        #sleep(0.02)
    return data

if __name__ == '__main__':
    
    
    plist = list(serial.tools.list_ports.comports())
    if len(plist) <= 0:
        print("没有发现端口!")
    else:
        s=plist[0]
        
        serial = serial.Serial('COM13', 115200, timeout=0.5)  #/dev/ttyUSB0
        if serial.isOpen() :
            print("open success")
            while True:
                a = input('按回车开始,q退出\n')
                if a == 'q':
                    break
                serial.write(b'\xc5')
                serial.write('37 200'.encode()) #数据写回 
                serial.write(b'\x00')
                sleep(2)
                serial.write(b'\xc5')
                serial.write('0 -200'.encode()) #数据写回 
                serial.write(b'\x00')
                sleep(0.5)
                serial.write(b'\xc5')
                serial.write('-30 -200'.encode()) #数据写回 
                serial.write(b'\x00')
                sleep(3.5)
                serial.write(b'\xc5')
                serial.write('15 -200'.encode()) #数据写回 
                serial.write(b'\x00')
                sleep(1)
                serial.write(b'\xc5')
                serial.write('0 0'.encode()) #数据写回 
                serial.write(b'\x00')
                sleep(0.01)
                serial.write(b'\xc5')
                serial.write('0 0'.encode()) #数据写回 
                serial.write(b'\x00')
            serial.close()
        else :
            print("open failed")
        
