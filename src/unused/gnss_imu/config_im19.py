import serial, time

ser = serial.Serial(
    '/dev/im19_mems',
    115200,
    timeout=0.5,
    dsrdtr=False,
    rtscts=False
)

ser.dtr = False
ser.rts = False

time.sleep(1)

ser.write(b'AT+MEMS_OUTPUT=UART1,ON\r\n')
time.sleep(1)

print(ser.read(1024).decode(errors='ignore'))

ser.close()
