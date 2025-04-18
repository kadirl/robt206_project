import pygame
import math
from collections import defaultdict
from main2 import *

# Configuration
MAP_SIZE = (800, 600)
SCALE = 20  # pixels per meter
ROBOT_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (0, 0, 255)
WHITE = (255, 255, 255)
GRAY = (111, 111, 111)

class SLAMVisualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode(MAP_SIZE)
        self.clock = pygame.time.Clock()

        # Robot state (x in meters, y in meters, theta in radians)
        self.pose = (MAP_SIZE[0]//2/SCALE, MAP_SIZE[1]//2/SCALE, 0.0)
        self.azimuth = 0.0  # Compass heading in radians

        # Map storage
        self.occupancy_grid = defaultdict(int)
        self.max_occupancy = 5

    def update_pose(self, speed, turn, dt):
        x, y, theta = self.pose
        theta += turn * dt
        x += speed * math.cos(theta) * dt
        y += speed * math.sin(theta) * dt
        self.pose = (x, y, theta % (2*math.pi))

    def process_lidar(self, readings):
        self.obstacles = []
        self.out_of_range = []
        x_robot, y_robot, theta_robot = self.pose

        for reading in readings:
            distance = reading['distance']
            angle = math.radians(reading['angle'])  # Convert to radians

            if distance > 0 and distance < 150:  # Valid reading
                # Calculate absolute angle (robot heading + sensor angle)
                abs_angle = theta_robot + angle

                # Convert to global coordinates
                x = x_robot + (distance / 50.0) * math.cos(abs_angle)
                y = y_robot + (distance / 50.0) * math.sin(abs_angle)

                # Store obstacle position in pixels
                grid_x = int(x * SCALE)
                grid_y = int(y * SCALE)
                self.obstacles.append((grid_x, grid_y))
            else:
                # Calculate absolute angle (robot heading + sensor angle)
                abs_angle = theta_robot + angle

                # Convert to global coordinates
                x = x_robot + (200 / 50.0) * math.cos(abs_angle)
                y = y_robot + (200 / 50.0) * math.sin(abs_angle)

                # Store obstacle position in pixels
                grid_x = int(x * SCALE)
                grid_y = int(y * SCALE)
                self.out_of_range.append((grid_x, grid_y))

    def visualize(self):
        # self.screen.fill(WHITE)

        # Draw obstacles
        for (x, y) in self.obstacles:
            pygame.draw.circle(self.screen, OBSTACLE_COLOR, (x, y), 2)

        # Draw obstacles
        for (x, y) in self.out_of_range:
            pygame.draw.circle(self.screen, GRAY, (x, y), 2)

        # Draw robot
        rx = int(self.pose[0] * SCALE)
        ry = int(self.pose[1] * SCALE)
        pygame.draw.circle(self.screen, ROBOT_COLOR, (rx, ry), 5)

        # Draw heading line
        print(self.pose[2])
        end_x = rx + 20 * math.cos(self.pose[2])
        end_y = ry + 20 * math.sin(self.pose[2])
        pygame.draw.line(self.screen, (0, 255, 0), (rx, ry), (end_x, end_y), 2)

        pygame.display.flip()


        # for (x, y) in self.obstacles:
        #     pygame.draw.circle(self.screen, WHITE, (x, y), 3)

        pygame.draw.circle(self.screen, WHITE, (rx, ry), 5)

        pygame.draw.line(self.screen, WHITE, (rx, ry), (end_x, end_y), 2)

        # Draw obstacles
        for (x, y) in self.out_of_range:
            pygame.draw.circle(self.screen, WHITE, (x, y), 3)

        self.clock.tick(30)

    def run(self):
        counter = 0
        running = True
        last_time = pygame.time.get_ticks()
        self.screen.fill(WHITE)

        while running:
            received_packet = get_latest_data()
            if received_packet:
                ts, received_packet = received_packet
                print(received_packet)
                joy_speed = received_packet[-3]/255
                joy_turn = received_packet[-2]/255
                lidar_readings = [
                    {'distance': received_packet[1], 'angle': received_packet[0]},    # Front
                    {'distance': received_packet[3], 'angle': received_packet[2]},  # Left
                    {'distance': received_packet[5], 'angle': received_packet[4]}   # Right
                ]
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                current_time = pygame.time.get_ticks()
                dt = (current_time - last_time) / 1000.0
                last_time = current_time
                print(dt)

                # Update visualization
                # self.update_platform(gyro_z, dt)
                self.update_pose(joy_speed, joy_turn, math.radians(received_packet[7])/25)
                self.process_lidar(lidar_readings)
                self.visualize()

        pygame.quit()

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
    slam.run()
