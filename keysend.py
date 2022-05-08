import struct
import sys
import time
import keyboard
import numpy as np
import serial

port_name = 'COM21' if len(sys.argv) < 2 else str(sys.argv[1])
try: port = serial.Serial(port=port_name, baudrate=115200, timeout=0.5)
except Exception as e:
    print(f"Couldn't connect: {e}")
    sys.exit(0)

def sendSerial(data, type='int16'):
    if type == 'float': tsize = 4; tstr = '<f'
    if type == 'int16': tsize = 2; tstr = '<h'
    size = np.array([data.size], dtype=np.int16)
    send_buffer = bytearray([])
    i = 0
    while(i < size[0]):
        send_buffer += struct.pack(tstr, data[i])
        i = i+1
    port.write(b'START')
    port.write(struct.pack('<h', tsize*size[0]))
    port.write(send_buffer)
    print(send_buffer)
    #print('sent: ' + f'{data}'[1:-1])
    # port_write = b'start' + struct.pack('<h', 2*size[0]) + send_buffer
    # print(port_write)

def readSerial(type='int16'):
    if type == 'float': tsize = 4; tstr = '<f'
    if type == 'int16': tsize = 2; tstr = '<h'
    state = 0
    while(state != 5):
        c1 = port.read(1)
        if(c1 == b''):
            print('Timout...')
            return []
        if(state == 0):
            if(c1 == b'S'): state = 1
            else: state = 0
        elif(state == 1):
            if(c1 == b'T'): state = 2
            elif(c1 == b'S'): state = 1
            else: state = 0
        elif(state == 2):
            if(c1 == b'A'): state = 3
            elif(c1 == b'S'): state = 1
            else: state = 0
        elif(state == 3):
            if(c1 == b'R'): state = 4
            elif (c1 == b'S'): state = 1
            else: state = 0
        elif(state == 4):
            if(c1 == b'T'): state = 5
            elif (c1 == b'S'): state = 1
            else: state = 0
    size = struct.unpack('<h',port.read(2)) 
    size = size[0]  
    rcv_buffer = port.read(size*tsize)
    data = []
    if(len(rcv_buffer) == size*tsize):
        i = 0
        while(i < size):
            data.append(struct.unpack_from(tstr, rcv_buffer, i*tsize))
            i = i+1
        print('received !')
        return data
    else:
        print('Timout...')
        return []

while (True):
    #data = [1 if keyboard.is_pressed('wasd'[_]) else 0 for _ in range(4)]
    # data = np.array((0, 0, 0), dtype='float')
    # sendSerial(data, 'float')
    time.sleep(1)
    data = np.array((10, 10, 0), dtype='float')
    sendSerial(data, 'float')
    quit()
    # input = readSerial('float')
    # print(input)