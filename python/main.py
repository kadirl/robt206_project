import serial
import struct
import time

# Define markers globally
START_MARKER = bytes([126]) # ~
END_MARKER = bytes([127])   # DEL

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
                    # print(f"DEBUG: Successfully parsed packet, remaining buffer: {buffer}")
                except struct.error as e:
                    print(f"Error unpacking: {e}, Data: {struct_data.hex()}")
                    # Corrupted data, consume the invalid packet attempt
                    buffer = buffer[end_index + 1:]
                    processed = True
            else:
                # Should not happen if end_index calculation is correct, but handle anyway
                print(f"Error: Size mismatch despite marker alignment. Expected {struct_size}, got {len(struct_data)}")
                buffer = buffer[end_index + 1:]
                processed = True

        elif end_index != -1 and end_index < end_marker_expected_index:
            # Found end marker too soon - data corruption
            print(f"Error: End marker found too soon at index {end_index} relative to start {start_index}. Discarding.")
            buffer = buffer[end_index + 1:] # Discard corrupted segment
            processed = True
            # else: # end_index == -1 or end_index > expected
            # Not enough data yet for a complete packet *starting at start_index*
            # OR end marker is further away than expected (corruption)
            # Check for buffer becoming too long if a start marker is present
            if len(buffer) > start_index + 1 + struct_size + 1 + 50: # Heuristic limit
                print(f"Error: No valid end marker found reasonably after start marker at {start_index}. Discarding partial.")
                buffer = buffer[start_index + 1:] # Discard the start marker and try again
                processed = True
            # Otherwise, just keep the buffer and wait for more data

    # Prevent buffer from growing indefinitely if no markers are ever found
    # Check only if no processing happened in this call to avoid excessive checks
    if not processed and len(buffer) > (struct_size + 10) * 5: # Increased arbitrary limit
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
        # print(f"DEBUG: Sent {bytes_sent} bytes: {message.hex()}") # Debug print
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


# ==============================================
#      MAIN EXECUTION
# ==============================================
if __name__ == "__main__":
    rand1 = 100
    rand2 = 50

    SERIAL_PORT = '/dev/tty.usbmodem1101' # CHANGE AS NEEDED!
    BAUD_RATE = 9600 # Must match Arduino

    # --- Define Struct Formats ---
    # Data RECEIVING From Arduino (RemoteToPC struct)
    receive_struct_format = (
        "<"     # little-endian
        "?"     # bool lidar_updated
        "HH"    # lidar.angle, lidar.distance (unsigned short)
        "?"     # bool gyro_updated
        "hhhhhh"# accelX, Y, Z, gyroX, Y, Z (short) - Changed from 6h
        "L"     # unsigned long timestamp (4 bytes on most Arduinos)
        "?"     # bool mode (manual/auto)
        "hh"    # joystick.speed, joystick.turn (short)
        "?"     # joystick.k (bool)
    )
    receive_struct_size = struct.calcsize(receive_struct_format)
    print(f"Receive Format (RemoteToPC): {receive_struct_format}, Size: {receive_struct_size}")


    # Data SENDING To Arduino (PCToRemote struct - controls car in auto mode)
    # Example: target speed and turn angle
    send_struct_format = (
        "<"     # little-endian
        "h"     # short target_speed
        "h"     # short target_turn
        "?"     # shit
    )
    send_struct_size = struct.calcsize(send_struct_format)
    print(f"Send Format (PCToRemote):    {send_struct_format}, Size: {send_struct_size}")


    # --- Serial Port Setup ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) # Short timeout for non-blocking reads
        print(f"Serial port {SERIAL_PORT} opened.")
        time.sleep(2) # Give Arduino time to reset
        ser.flushInput()
    except serial.SerialException as e:
        print(f"FATAL: Could not open serial port {SERIAL_PORT}: {e}")
        exit()

    # --- Main Loop ---
    serial_buffer = b''
    last_send_time = 0
    send_interval = 0.3 # Send data every 1 second (adjust as needed)

    try:
        while True:
            # --- Try to RECEIVE data from Arduino ---
            if ser.in_waiting > 0:
                serial_buffer += ser.read(ser.in_waiting)

            # Process the buffer to find complete packets
            while True: # Loop to process multiple packets if available
                received_packet, serial_buffer = parse_serial_packet(serial_buffer, receive_struct_format)
                if received_packet:
                    print(f"\n<<< RECEIVED From Arduino (RemoteToPC): {received_packet}")
                    # TODO: Process the received data (e.g., display, log)
                else:
                    break # No complete packet found in buffer this time


            # --- Try to SEND data to Arduino (e.g., on a timer) ---
            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                # Example data to send (replace with your actual logic)
                rand1 += 1
                rand2 += 1
                target_speed_cmd = rand1 # Example command
                target_turn_cmd = rand2  # Example command
                data_to_send = (target_speed_cmd, target_turn_cmd, 1)

                print(f"\n>>> SENDING To Arduino (PCToRemote): {data_to_send}")
                if send_serial_data(ser, send_struct_format, data_to_send):
                    # print("    Send successful.")
                    last_send_time = current_time
                else:
                    print("    Send FAILED.")
                    # Handle failure? Retry?

            # Optional small delay to prevent high CPU usage in main loop
            # time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Closing serial port.")
    except Exception as e:
        print(f"\nAn error occurred in the main loop: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")