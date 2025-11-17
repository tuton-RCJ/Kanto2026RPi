import serial
stm = serial.Serial('/dev/ttyAMA2', 115200, timeout=1)
stm.flush()

stm2=serial.Serial('/dev/ttyAMA5',115200,timeout=1)
stm2.flush()

def read_stm2():
    if stm2.in_waiting > 0:
        line = stm2.readline().decode('utf-8').rstrip()
        print(line)
        return line
    return None

def drive(speed,turnRate):
    stm.write(f'{speed},{turnRate}\n'.encode('utf-8'))
    stm.flush()


def stop():
    drive(0, 0)

def closeSTMPort():
    stm.close()