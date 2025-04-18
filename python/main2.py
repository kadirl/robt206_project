import serial
import struct
import time
import threading
from collections import deque
from my_model import SLAMVisualizer

# Global buffer for storing received data
GLOBAL_BUFFER = deque(maxlen=100)  # Adjust maxlen as needed
BUFFER_LOCK = threading.Lock()

# Define markers globally
START_MARKER = bytes([126])  # ~
END_MARKER = bytes([127])    # DEL

def parse_serial_packet(buffer, struct_format):
    """
    Attempts to find and unpack one complete packet from the buffer.
    Returns tuple (unpacked_data, remaining_buffer) or (None, original_buffer)
    """
    struct_size = struct.calcsize(struct_format)
    unpacked_data = None
    processed = False

    start_index = buffer.find(START_MARKER)
    if start_index != -1:
        # Potential packet start found
        end_marker_expected_index = start_index + 1 + struct_size
        end_index = buffer.find(END_MARKER, end_marker_expected_index)

        if end_index == end_marker_expected_index:
            # Found start, data, and end marker correctly positioned
            struct_data_start = start_index + 1
            struct_data_end = end_index
            struct_data = buffer[struct_data_start:struct_data_end]

            if len(struct_data) == struct_size:
                try:
                    unpacked_data = struct.unpack(struct_format, struct_data)
                    # Successfully unpacked, consume packet from buffer
                    buffer = buffer[end_index + 1:]
                    processed = True
                except struct.error as e:
                    print(f"Error unpacking: {e}, Data: {struct_data.hex()}")
                    # Corrupted data, consume the invalid packet attempt
                    buffer = buffer[end_index + 1:]
                    processed = True
            else:
                print(f"Error: Size mismatch despite marker alignment. Expected {struct_size}, got {len(struct_data)}")
                buffer = buffer[end_index + 1:]
                processed = True

        elif end_index != -1 and end_index < end_marker_expected_index:
            # Found end marker too soon - data corruption
            print(f"Error: End marker found too soon at index {end_index} relative to start {start_index}. Discarding.")
            buffer = buffer[end_index + 1:]  # Discard corrupted segment
            processed = True

            if len(buffer) > start_index + 1 + struct_size + 1 + 50:  # Heuristic limit
                print(f"Error: No valid end marker found reasonably after start marker at {start_index}. Discarding partial.")
                buffer = buffer[start_index + 1:]  # Discard the start marker and try again
                processed = True

    # Prevent buffer from growing indefinitely
    if not processed and len(buffer) > (struct_size + 10) * 5:
        print("Warning: Buffer growing large without valid packets, clearing half.")
        buffer = buffer[len(buffer)//2:]

    return unpacked_data, buffer

def send_serial_data(ser, struct_format, data_tuple):
    """
    Packs data using struct_format and sends it over the serial port
    with start and end markers.
    """
    try:
        packed_data = struct.pack(struct_format, *data_tuple)
        message = START_MARKER + packed_data + END_MARKER
        bytes_sent = ser.write(message)
        return bytes_sent == len(message)
    except serial.SerialException as e:
        print(f"Error sending serial data: {e}")
        return False
    except struct.error as e:
        print(f"Error packing data: {e}")
        print(f"Format: {struct_format}, Data: {data_tuple}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during sending: {e}")
        return False

def serial_thread_function(port, baud_rate, receive_struct_format, stop_event):
    """
    Thread function that handles serial communication and stores data in the global buffer.
    """
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.05)
        print(f"Serial port {port} opened.")
        time.sleep(2)  # Give Arduino time to reset
        ser.flushInput()
    except serial.SerialException as e:
        print(f"FATAL: Could not open serial port {port}: {e}")
        return

    serial_buffer = b''

    try:
        while not stop_event.is_set():
            # Try to RECEIVE data from Arduino
            if ser.in_waiting > 0:
                serial_buffer += ser.read(ser.in_waiting)

            # Process the buffer to find complete packets
            while True:
                received_packet, serial_buffer = parse_serial_packet(serial_buffer, receive_struct_format)
                if received_packet:
                    with BUFFER_LOCK:
                        GLOBAL_BUFFER.append((time.time(), received_packet))
                else:
                    break

            time.sleep(0.01)  # Small delay to prevent high CPU usage

    except Exception as e:
        print(f"Error in serial thread: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

def start_serial_thread(port, baud_rate, receive_struct_format):
    """
    Starts the serial communication thread and returns the stop event.
    """
    stop_event = threading.Event()
    thread = threading.Thread(
        target=serial_thread_function,
        args=(port, baud_rate, receive_struct_format, stop_event),
        daemon=True
    )
    thread.start()
    return stop_event

def get_latest_data():
    """
    Returns the latest data from the global buffer.
    """
    with BUFFER_LOCK:
        if GLOBAL_BUFFER:
            return GLOBAL_BUFFER[-1]
    return None

def get_all_data():
    """
    Returns all data from the global buffer and clears it.
    """
    with BUFFER_LOCK:
        data = list(GLOBAL_BUFFER)
        GLOBAL_BUFFER.clear()
    return data

# Example usage:
if __name__ == "__main__":
    # Define the struct format (same as in your original code)
    receive_struct_format = (
        "<"              # Little-endian
        "hHhHhH"         # Lidar angles: sensor1.angle, sensor2.angle, sensor3.angle (shorts)
        "L"            # Lidar distances: sensor1.dist, sensor2.dist, sensor3.dist (unsigned shorts)
        "f"              # Azimuth (float)
        "ffffff"         # GyroRead: accelX, accelY, accelZ, gyroX, gyroY, gyroZ (floats)
        "L"              # Timestamp (unsigned long)
        "?"              # Mode (bool)
        "hh?"            # Joystick: speed, turn (shorts), k (bool)
    )

    # Start the serial thread
    stop_event = start_serial_thread(
        port='/dev/cu.usbmodem1201',  # Change as needed
        baud_rate=9600,
        receive_struct_format=receive_struct_format
    )
    slam = SLAMVisualizer()

    try:
        while True:
            # Example of accessing the data from another thread
            latest_data = get_latest_data()
            if latest_data:
                timestamp, data = latest_data
                slam.run(data)
                print(f"Latest data at {timestamp}: {data}")

            # time.sleep(1)  # Simulate other work

    except KeyboardInterrupt:
        print("\nStopping...")
        stop_event.set()