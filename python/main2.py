import threading
import queue
import time
import random
import serial # Make sure PySerial is installed: pip install pyserial
import struct

# Define markers (can be class attributes or passed in __init__)
START_MARKER = bytes([0x7E]) # ~ (from Structs.h)
END_MARKER = bytes([0x7F])   # DEL (from Structs.h)

class SerialCommunicator:
    """
    A threaded class to handle serial communication (sending/receiving structured data).

    Manages serial port connection, data buffering, packet parsing/packing,
    and communication with other threads via input/output queues in a
    background thread.
    """
    def __init__(self, port, baud_rate,
                 receive_format, send_format,
                 name="SerialWorker", read_timeout=0.05, connect_delay=2.0):
        """
        Initializes the SerialCommunicator.

        Args:
            port (str): The serial port name (e.g., '/dev/ttyACM0', 'COM3').
            baud_rate (int): The serial communication speed.
            receive_format (str): The struct format string for data RECEIVED from the device.
            send_format (str): The struct format string for data SENT to the device.
            name (str): A name for the worker thread.
            read_timeout (float): Timeout for serial read operations in seconds.
            connect_delay (float): Time to wait after opening port before operations.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.read_timeout = read_timeout
        self.connect_delay = connect_delay

        self.receive_struct_format = receive_format
        self.send_struct_format = send_format
        self._receive_struct_size = struct.calcsize(receive_format)
        self._send_struct_size = struct.calcsize(send_format)

        # Threading and Queues (similar to BackgroundIoWorker)
        # Input queue: Other threads put data tuples here to be SENT over serial.
        self.send_queue = queue.Queue()
        # Output queue: Parsed data tuples RECEIVED from serial are put here.
        self.receive_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread_name = name
        self._worker_exception = None
        self._ser = None # Serial port object, initialized in start()
        self._serial_buffer = b'' # Internal buffer for incoming serial data

        self._thread = threading.Thread(
            target=self._run,
            name=self._thread_name,
            daemon=False # Usually False for clean shutdown
        )
        print(f"[{self._thread_name}] SerialCommunicator initialized for {self.port}.")
        print(f"    Receive size: {self._receive_struct_size} bytes")
        print(f"    Send size:    {self._send_struct_size} bytes")

    # --- Internal Worker Methods ---

    def _parse_serial_packet(self):
        """
        Attempts to find and unpack one complete packet from self._serial_buffer.
        Modifies self._serial_buffer directly.
        Returns unpacked_data tuple or None.
        """
        unpacked_data = None
        processed = False
        buffer = self._serial_buffer # Work with a local reference for clarity

        start_index = buffer.find(START_MARKER)
        if start_index != -1:
            # Ensure buffer has enough potential data for marker + struct + marker
            # before calculating expected end marker position
            if len(buffer) >= start_index + 1 + self._receive_struct_size + 1:
                end_marker_expected_index = start_index + 1 + self._receive_struct_size
                # Check if the end marker is exactly where expected
                if buffer[end_marker_expected_index:end_marker_expected_index + 1] == END_MARKER:
                    struct_data_start = start_index + 1
                    struct_data_end = end_marker_expected_index
                    struct_data = buffer[struct_data_start:struct_data_end]

                    # Double check size just in case (should be redundant with above check)
                    if len(struct_data) == self._receive_struct_size:
                        try:
                            unpacked_data = struct.unpack(self.receive_struct_format, struct_data)
                            # Consume the valid packet including markers
                            buffer = buffer[end_marker_expected_index + 1:]
                            processed = True
                            # print(f"DEBUG: Parsed packet, remaining buffer len: {len(buffer)}")
                        except struct.error as e:
                            print(f"[{self._thread_name}] Error unpacking: {e}, Data: {struct_data.hex()}")
                            # Consume the invalid packet attempt
                            buffer = buffer[end_marker_expected_index + 1:]
                            processed = True
                    else:
                        # Should not happen if logic above is correct, but as a safeguard
                        print(f"[{self._thread_name}] Error: Internal logic size mismatch. Expected {self._receive_struct_size}, got {len(struct_data)}")
                        buffer = buffer[end_marker_expected_index + 1:]
                        processed = True
                else:
                    # End marker not where expected. Indicates corruption or partial packet.
                    # Discard data up to and including the start marker to find the next potential start.
                    print(f"[{self._thread_name}] Warning: End marker not found at expected position {end_marker_expected_index} relative to start {start_index}. Discarding corrupt segment.")
                    buffer = buffer[start_index + 1:]
                    processed = True
            else:
                # Not enough data currently in buffer for a complete packet starting at start_index
                # Keep the buffer as is and wait for more data
                pass

        elif len(buffer) > (self._receive_struct_size + 2) * 5: # Heuristic limit if no start marker found
            # Prevent excessive buffer growth if only invalid data is received
            print(f"[{self._thread_name}] Warning: Buffer growing large without start markers, clearing half.")
            buffer = buffer[len(buffer)//2:]
            processed = True # Indicate buffer was modified

        self._serial_buffer = buffer # Update the instance buffer
        return unpacked_data

    def _send_serial_data(self, data_tuple):
        """
        Packs data tuple and sends it over serial with markers.
        Returns True on success, False on failure.
        """
        if not self._ser or not self._ser.is_open:
            print(f"[{self._thread_name}] Error: Serial port not open for sending.")
            return False
        try:
            # Ensure the data tuple has the correct number of elements
            # expected_elements = len(struct.calcsize(self.send_struct_format)) # Bit imprecise, better count format chars
            expected_elements = struct.calcsize(self.send_struct_format) # Bit imprecise, better count format chars
            format_elements = 0
            for char in self.send_struct_format:
                if char.isalpha() or char == '?':
                    format_elements += 1 # Rough count
            # A more robust way: try packing first
            # if len(data_tuple) != expected_elements: # Simple check
            #      print(f"[{self._thread_name}] Error: Data tuple elements ({len(data_tuple)}) do not match send format ({expected_elements}).")
            #      return False

            packed_data = struct.pack(self.send_struct_format, *data_tuple)
            if len(packed_data) != self._send_struct_size:
                # This can happen if format string calculation differs from actual packing
                print(f"[{self._thread_name}] Error: Packed data size ({len(packed_data)}) differs from expected ({self._send_struct_size}). Format: {self.send_struct_format}")
                return False

            message = START_MARKER + packed_data + END_MARKER
            bytes_sent = self._ser.write(message)
            # print(f"DEBUG: Sent {bytes_sent} bytes: {message.hex()}")
            return bytes_sent == len(message)
        except serial.SerialException as e:
            print(f"[{self._thread_name}] Error sending serial data: {e}")
            self._worker_exception = e
            self.stop() # Signal stop on serial error
            return False
        except struct.error as e:
            print(f"[{self._thread_name}] Error packing data: {e}")
            print(f"    Format: {self.send_struct_format}, Size: {self._send_struct_size}, Elements: {len(data_tuple)}, Data: {data_tuple}")
            return False
        except Exception as e:
            print(f"[{self._thread_name}] Unexpected error during sending: {e}")
            self._worker_exception = e
            self.stop()
            return False

    def _run(self):
        """ The main loop for the serial worker thread. """
        print(f"[{self._thread_name}] Worker thread starting execution loop.")
        if not self._ser or not self._ser.is_open:
            print(f"[{self._thread_name}] Error: Serial port not open at start of run loop.")
            self._worker_exception = serial.SerialException("Port not open at run start")
            return # Exit thread if port wasn't opened successfully

        while not self._stop_event.is_set():
            try:
                # --- 1. Handle Sending ---
                try:
                    # Check input queue for data to send (non-blocking)
                    data_to_send = self.send_queue.get_nowait()
                    # print(f"[{self._thread_name}] >>> SENDING Prep: {data_to_send}") # Debug
                    if self._send_serial_data(data_to_send):
                        self.send_queue.task_done()
                        # print(f"    Send successful.") # Debug
                    else:
                        print(f"    Send FAILED.")
                        # Failed send might warrant stopping or specific error handling
                        # Keep task in queue? Discard? For now, we mark done.
                        self.send_queue.task_done()

                except queue.Empty:
                    pass # Normal operation
                except Exception as e:
                    print(f"[{self._thread_name}] Error during send handling: {e}")
                    self._worker_exception = e


                # --- 2. Handle Receiving ---
                bytes_waiting = 0
                try:
                    if self._ser.in_waiting > 0:
                        bytes_waiting = self._ser.in_waiting
                        self._serial_buffer += self._ser.read(bytes_waiting)
                except serial.SerialException as e:
                    print(f"[{self._thread_name}] Error reading from serial port: {e}")
                    self._worker_exception = e
                    self.stop()
                    continue
                except Exception as e:
                    print(f"[{self._thread_name}] Unexpected error during serial read: {e}")
                    self._worker_exception = e
                    self.stop()
                    continue

                # Attempt to parse all complete packets currently in the buffer
                while True:
                    received_packet = self._parse_serial_packet()
                    if received_packet:
                        # print(f"[{self._thread_name}] <<< RECEIVED Parsed: {received_packet}") # Debug
                        try:
                            self.receive_queue.put(received_packet)
                        except queue.Full:
                            print(f"[{self._thread_name}] Warning: Receive queue full, discarding packet.")
                    else:
                        break # No more complete packets in buffer for now

                # --- 3. Small Sleep ---
                if not bytes_waiting and self.send_queue.empty():
                    time.sleep(0.01)

            except Exception as e:
                print(f"[{self._thread_name}] FATAL ERROR in worker loop: {e}")
                self._worker_exception = e
                self.stop()

        # --- Cleanup after loop exit ---
        print(f"[{self._thread_name}] Worker loop finished.")


    # --- Public Control Methods ---

    def start(self):
        """Opens the serial port and starts the background worker thread."""
        if self.is_running():
            print(f"[{self._thread_name}] Worker thread already running.")
            return False

        print(f"[{self._thread_name}] Attempting to open serial port {self.port}...")
        try:
            self._ser = serial.Serial(self.port, self.baud_rate, timeout=self.read_timeout)
            print(f"[{self._thread_name}] Serial port {self.port} opened.")
            time.sleep(self.connect_delay)
            self._ser.reset_input_buffer() # More reliable than flushInput sometimes
            print(f"[{self._thread_name}] Input buffer reset.")
        except serial.SerialException as e:
            print(f"[{self._thread_name}] FATAL: Could not open serial port {self.port}: {e}")
            self._ser = None
            self._worker_exception = e
            return False

        self._stop_event.clear()
        self._worker_exception = None
        self._serial_buffer = b''
        while not self.send_queue.empty(): self.send_queue.get_nowait()
        while not self.receive_queue.empty(): self.receive_queue.get_nowait()

        try:
            # Ensure thread object is created if restarting after join
            if not hasattr(self, '_thread') or not isinstance(self._thread, threading.Thread) or not self._thread.ident:
                self._thread = threading.Thread(
                    target=self._run, name=self._thread_name, daemon=False
                )

            self._thread.start()
            print(f"[{self._thread_name}] Worker thread started.")
            return True
        except RuntimeError as e:
            print(f"[{self._thread_name}] Error starting thread: {e}. Might need re-initialization.")
            if self._ser and self._ser.is_open: self._ser.close()
            self._ser = None
            return False

    def stop(self):
        """Signals the worker thread to stop processing."""
        if not self._stop_event.is_set():
            print(f"[{self._thread_name}] Signaling worker to stop...")
            self._stop_event.set()

    def join(self, timeout=None):
        """
        Waits for the worker thread to terminate and closes the serial port.

        Args:
            timeout (float, optional): Max time in seconds to wait. Defaults to None.

        Returns:
            bool: True if the thread terminated, False if timeout occurred.
        """
        print(f"[{self._thread_name}] Joining worker thread...")
        is_alive = self._thread.is_alive()
        if is_alive:
            self._thread.join(timeout)
            is_alive = self._thread.is_alive()

        if is_alive:
            print(f"[{self._thread_name}] Join timed out.")
        else:
            print(f"[{self._thread_name}] Worker thread joined.")

        if self._ser and self._ser.is_open:
            print(f"[{self._thread_name}] Closing serial port {self.port}.")
            try: self._ser.close()
            except Exception as e: print(f"[{self._thread_name}] Error closing serial port: {e}")
        # else: print(f"[{self._thread_name}] Serial port was not open or already closed.") # Less verbose
        self._ser = None

        # Thread object cannot be restarted. Handled in start() if needed.

        return not is_alive

    def put_data_to_send(self, data_tuple, block=True, timeout=None):
        """Adds a data tuple to the queue for sending over serial."""
        self.send_queue.put(data_tuple, block=block, timeout=timeout)

    def get_received_data(self, block=True, timeout=None):
        """Retrieves a parsed data tuple received from serial."""
        return self.receive_queue.get(block=block, timeout=timeout)

    def is_running(self):
        """Checks if the worker thread is currently executing."""
        # Check if thread object exists and is alive
        return hasattr(self, '_thread') and self._thread.is_alive()

    def get_exception(self):
        """Returns any exception captured from within the worker thread."""
        return self._worker_exception


# ==============================================
#      MAIN EXECUTION EXAMPLE
# ==============================================
if __name__ == "__main__":

    # SERIAL_PORT = '/dev/tty.usbmodem1101' # MacOS Example
    SERIAL_PORT = '/dev/cu.usbmodem1201'           # Linux Example
    # SERIAL_PORT = 'COM3'                 # Windows Example
    # --- CHANGE AS NEEDED! ---

    BAUD_RATE = 115200 # <<<<<<< COMMON BAUD RATE FOR ARDUINO SERIAL
    # Adjust if your Arduino sketch uses a different rate

    # --- Define Struct Formats based on Structs.h ---

    # Data RECEIVING From Arduino (RemoteToPC struct)
    # LidarRead(angle=H, dist=H), azimuth=f, GyroRead(6f), timestamp=L, mode=?, JoystickRead(speed=h, turn=h, k=?)
    receive_struct_format = "<HHf6fL?hh?"
    receive_struct_size_check = struct.calcsize(receive_struct_format)
    print(f"Receive Format (RemoteToPC): {receive_struct_format}, Expected Size: {receive_struct_size_check} bytes")
    # Expected Size should be: 2+2+4 + (6*4) + 4 + 1 + 2+2+1 = 42 bytes

    # Data SENDING To Arduino (PCToRemote struct)
    # JoystickRead(speed=h, turn=h, k=?)
    send_struct_format = "<hh?"
    send_struct_size_check = struct.calcsize(send_struct_format)
    print(f"Send Format (PCToRemote):    {send_struct_format}, Expected Size: {send_struct_size_check} bytes")
    # Expected Size should be: 2+2+1 = 5 bytes

    # --- Create and Start Communicator ---
    communicator = SerialCommunicator(
        port=SERIAL_PORT,
        baud_rate=BAUD_RATE,
        receive_format=receive_struct_format,
        send_format=send_struct_format
    )

    if not communicator.start():
        print("FATAL: Could not start Serial Communicator. Exiting.")
        exit()

    # --- Main Application Loop (Example) ---
    last_send_time = time.time()
    send_interval = 0.1 # Send data more frequently (e.g., 10 times/sec)

    # Example variables for sending data
    target_speed = 0
    target_turn = 0
    joystick_k_button = False

    try:
        while True:
            # --- Try to RECEIVE data from the communicator's queue ---
            try:
                received_data = communicator.get_received_data(block=False)
                # Unpack the received tuple based on the known structure RemoteToPC
                (lidar_angle, lidar_dist, azimuth,
                 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
                 timestamp, mode, joy_speed, joy_turn, joy_k) = received_data

                print("\n[MAIN] Received Data:")
                print(f"  Lidar: Angle={lidar_angle}, Dist={lidar_dist}")
                print(f"  Azimuth: {azimuth:.2f}")
                print(f"  Accel: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
                print(f"  Gyro:  X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
                print(f"  Timestamp: {timestamp}")
                print(f"  Mode (Manual=True): {mode}")
                print(f"  Joystick Echo: Speed={joy_speed}, Turn={joy_turn}, K={joy_k}")

            except queue.Empty:
                pass # No new data

            # --- Check worker thread status ---
            if not communicator.is_running():
                print("[MAIN] Error: Serial worker thread has stopped!")
                exception_info = communicator.get_exception()
                if exception_info: print(f"[MAIN] Worker stopped due to exception: {exception_info}")
                break

            # --- Try to SEND data periodically ---
            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                # Simulate changing control values (replace with actual UI/logic)
                target_speed = int(100 * random.uniform(-1, 1)) # Random speed -100 to 100
                target_turn = int(80 * random.uniform(-1, 1))   # Random turn -80 to 80
                joystick_k_button = (random.random() < 0.1)     # Randomly press button

                # Ensure values fit within 'short' range (-32768 to 32767) if necessary
                target_speed = max(-32767, min(32767, target_speed))
                target_turn = max(-32767, min(32767, target_turn))

                # Data tuple MUST match send_struct_format: (short, short, bool)
                data_to_send = (target_speed, target_turn, joystick_k_button)

                # print(f"\n[MAIN] Preparing to send: Speed={data_to_send[0]}, Turn={data_to_send[1]}, K={data_to_send[2]}") # Debug
                try:
                    communicator.put_data_to_send(data_to_send, block=False)
                    last_send_time = current_time
                except queue.Full:
                    print("[MAIN] Warning: Send queue is full. Skipping send this time.")

            # --- Other Main Thread Logic ---
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[MAIN] Keyboard interrupt detected. Stopping communicator.")
    except Exception as e:
        print(f"\n[MAIN] An error occurred in the main loop: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
    finally:
        # --- Clean Shutdown ---
        print("[MAIN] Initiating shutdown sequence...")
        communicator.stop()
        communicator.join(timeout=3.0) # Shorter timeout

        if communicator.is_running(): print("[MAIN] Warning: Worker thread did not stop gracefully.")
        else: print("[MAIN] Worker thread stopped.")

        print("[MAIN] Checking for final received items...")
        final_count = 0
        while True:
            try:
                final_data = communicator.get_received_data(block=False)
                print(f"[MAIN] Final Received Data: {final_data}")
                final_count += 1
            except queue.Empty: break
        if final_count == 0: print("    No final items.")

        print("[MAIN] Application finished.")