import serial
import time
import sys

# Serial config
SERIAL_PORT =       '/dev/ttyUSB0'          # Port addrs (ex: '/dev/ttyUSB0')
BAUD_RATE =         115200                  # transmition rate
OUTPUT_FILE =       'serial3.txt'           # output file name

def read_serial_and_save():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to: {ser.portstr}")
            with open(OUTPUT_FILE, 'a', encoding='utf-8') as f:
                print(f"Data saved in: {OUTPUT_FILE}. Press Ctrl+C to exit.")
                
                while True:
                    line_bytes = ser.readline()
                    
                    if line_bytes:
                        try:
                            line = line_bytes.decode('utf-8').strip()
                            if line:
                                print(f"Recebido: {line}")
                                f.write(line + '\n')
                                f.flush() 
                        except UnicodeDecodeError:
                            print("Error decoding line (possible non-UTF-8 characters))")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print(f"Is serial port '{SERIAL_PORT}' correct and available?.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nRead interrupted by user.")
    except Exception as e:
        print(f"Unknown error: {e}")
    finally:
        print("End program.")

if __name__ == "__main__":
    read_serial_and_save()