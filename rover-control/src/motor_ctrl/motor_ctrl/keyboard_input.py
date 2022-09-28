import math
import sys
import numpy as np

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import Mov

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


def msg(speed):
    return """
    
---------------------------
Moving around:
        w      
   a         d
        s     

o : faster
l : slower

anything else : stop

SPEED:  {}
""".format(speed)

dirBindings = {
    'w': 1.0,
    's': -1.0,
}

steerBindings = {
    'a': 1.0,
    'd': -1.0,
}

speedBindings = {
    'o': 1.2,
    'l': 0.8
}

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def angle_between(v1, v2):
    unit_vector_1 = np.array(v1) / np.linalg.norm(v1)
    unit_vector_2 = np.array(v2) / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)


def rotate(vector, angle):
    x, y = vector
    extreme_direction = 0.0, math.copysign(1, y)
    if angle_between(vector, extreme_direction) < angle:
        return extreme_direction
    else:
        return  x * math.cos(angle) - y * math.sin(angle), \
                x * math.sin(angle) + y * math.cos(angle)


class KeyboardInput(Node):
    def __init__(self):
        super().__init__('KeyboardInput')
        self.publisher = self.create_publisher(Mov, 'mov', 10)
        self.settings = saveTerminalSettings()

        self.speed = 0.7
        self.movement = 0.0, 0.0
        self.steering_speed = math.pi / 12   # radians

        self.loop()

    def loop(self):
        try:
            self.print_info()
            while True:
                key = self.getKey()
                fwd, rot = self.movement
                forward = math.copysign(1, fwd)
                direction = math.copysign(1, rot)

                if key in dirBindings.keys():
                    self.set_movement(dirBindings[key], 0.0)

                elif key in steerBindings.keys():
                    steer = steerBindings[key]
                    if abs(fwd) > 0.01:
                        # moving - either forward or backward
                        rotation_base = self.movement
                        if abs(rot) < 0.001 or direction != math.copysign(1, steer):
                            # immediately counteract steering in opposite direction
                            rotation_base = fwd, 0.0

                        # apply rotation to base
                        self.set_movement(*rotate(rotation_base, forward * steer * self.steering_speed))
                    else:
                        # when standing sill, just rotate in place
                        self.set_movement(0.0, steerBindings[key])

                elif key in speedBindings.keys():
                    self.speed = max(0.0, min(self.speed * speedBindings[key], 1.0))
                    self.print_info()

                else:
                    self.set_movement(0.0, 0.0)
                    if key == '\x03':
                        break

                self.send_movement_order()

        except Exception as e:
            print(e)

        finally:
            self.speed = 0.7
            self.set_movement(0.0, 0.0)
            self.send_movement_order()

            restoreTerminalSettings(self.settings)

    def set_movement(self, new_forward, new_rotate):
        self.movement = new_forward, new_rotate

    def send_movement_order(self, normalize=True):
        mov = Mov()
        fwd, rot = self.movement
        mov.forward = fwd
        mov.rotate = rot

        if normalize:
            length = math.sqrt(fwd * fwd + rot * rot)
            if length > 0:
                mov.forward /= length
                mov.rotate /= length
            else:
                mov.forward = 0.0
                mov.rotate = 0.0

        mov.forward *= self.speed
        mov.rotate *= self.speed

        self.publisher.publish(mov)

    def print_info(self):
        print(msg(int(100 * self.speed)))

    def getKey(self):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)

    keyboard_input = KeyboardInput()

    rclpy.spin(keyboard_input)

    keyboard_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
