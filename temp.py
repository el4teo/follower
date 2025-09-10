import serial
import time

# Setup serial communication with HC-05 (check which COM port is assigned!)
ser = serial.Serial('COM3', 9600, timeout=1)

def read_data():
    """Read bytes from HC-05, return as a list of individual bytes."""
    if ser.in_waiting > 0:
        incoming_data = ser.read(ser.in_waiting)  # Read all available bytes
        return list(incoming_data)  # Convert to list of integers
    return []

def read_byte():
    """Read bytes from HC-05, return as a list of individual bytes."""
    while True:
        if ser.in_waiting > 0:
            incoming_byte = ser.read(1)  # Read a single byte
            return incoming_byte[0]
def send_hex_response(byte_val):
    """Send a custom hex response to HC-05 based on received byte."""
    if byte_val == 0x01:  # If received 0x01
        ser.write(bytes([0x12]))
        print("Sent: 0x12")
    elif byte_val == 0x34:  # If received 0x34
        ser.write(bytes([0x56]))
        print("Sent: 0x56")
    else:
        # Default response
        ser.write(bytes([0x00]))
        print("Sent: 0x00")

def send_k(k):
    """Send k factor """
    try:
        # handles.figure1.UserData.esperandoConfirmacion = true;
        print(f"Sending code: {0}")
        ser.write([0x00])
        print(f"Sending code: {252}")
        ser.write([252]) # This line produces an error
        myInt = int(k * 1000)
        print(f"Sending bytes")
        ser.write([0x00, 0x00, 0x00, 0x00])
        # ser.write(int([myInt]))
        # Esperar respuesta de que el número ha sido recibido exitosamente
        answer = read_byte()
        print(f"Received: {answer}")
        if answer == 252:
            print(f"Confirmation code recived")
    except:
        print('Error in send_k')
    finally:
        # handles.figure1.UserData.esperandoConfirmacion = false;
        pass

def main():
    """Main loop to read and respond to HC-05."""
    print("Starting communication with HC-05...")
    try:
        while True:
            data_bytes = read_data()
            for byte_val in data_bytes:
                print(f"Received: 0x{byte_val:02X}")
                send_hex_response(byte_val)
            time.sleep(0.05)
        # answer = read_byte()
        # print(f"Received: {answer}")
        # if answer == 0xff:
        #     send_k(1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        print(f"Program finished")

if __name__ == "__main__":
    main()
