import time
import serial

def send_sms(phone_number: str, message: str,
             port: str = '/dev/ttyUSB3', baud: int = 115200):
    """
    Send a single SMS and report only success/failure.
    Max execution ≲2 s via short timeouts.
    """
    try:
        # Open with a short default timeout
        ser = serial.Serial(port, baud, timeout=0.5)
        
        # 1) Text mode
        ser.write(b'AT+CMGF=1\r')
        ser.readline()  # consume OK (or timeout)
        
        # 2) Trigger send, wait for '>' prompt
        ser.write(f'AT+CMGS="{phone_number}"\r'.encode())
        if b'>' not in ser.read_until(b'>'):
            print("SMS failed: no prompt")
            return
        
        # 3) Send body + terminator, wait for final OK
        ser.write(message.encode() + b'\x1A')
        resp = ser.read_until(b'OK')
        
        if b'OK' in resp:
            print("SMS sent successfully")
        else:
            print("SMS failed: no OK")
        
    except Exception as e:
        print(f"SMS failed: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

def main():
    # Open the serial port with appropriate settings:
    # - Replace '/dev/ttyUSB3' with your device name if different.
    # - Match the baudrate (e.g., 115200) to what your modem expects.
    # ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
    
    # try:
    #     # Send AT command to put device in airplane mode
    #     # Add "\r" or "\r\n" as the device expects.
    #     ser.write(b'AT+CFUN=1\r\n')
        
    #     # Read back a response (up to 128 bytes)
    #     response = ser.read(128)
    #     print("Response:", response.decode(errors='ignore'))
        
    # finally:
    #     ser.close()
    send_sms("+19022099739", "Hello from BC")

if __name__ == "__main__":
    main()