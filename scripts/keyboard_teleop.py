#!/usr/bin/env python3
"""
Keyboard teleop for Unicorn racing vehicle.
Publishes geometry_msgs/Twist to /cmd_vel.

Controls:
    w/x : increase/decrease linear speed
    a/d : increase/decrease steering (angular)
    s   : stop (all zero)
    q   : quit
"""

import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

HELP_MSG = """
------------------------------------------
  Unicorn Keyboard Teleop
------------------------------------------
  w : forward      s : backward
  a : steer left   d : steer right
  r : full stop
  q : quit
------------------------------------------
"""

LINEAR_STEP = 0.2
ANGULAR_STEP = 0.15
MAX_LINEAR = 5.0
MAX_ANGULAR = 0.8


def get_key(timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    speed = 0.0
    steer = 0.0

    print(HELP_MSG)

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == 'w':
                speed = min(speed + LINEAR_STEP, MAX_LINEAR)
            elif key == 's':
                speed = max(speed - LINEAR_STEP, -MAX_LINEAR)
            elif key == 'a':
                steer = min(steer + ANGULAR_STEP, MAX_ANGULAR)
            elif key == 'd':
                steer = max(steer - ANGULAR_STEP, -MAX_ANGULAR)
            elif key == 'r':
                speed = 0.0
                steer = 0.0
            elif key == 'q' or key == '\x03':
                pub.publish(Twist())
                break
            else:
                # No key pressed: decay both to zero
                speed *= 0.9
                steer *= 0.85
                if abs(speed) < 0.01:
                    speed = 0.0
                if abs(steer) < 0.01:
                    steer = 0.0

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = steer
            pub.publish(twist)

            sys.stdout.write(
                '\r  speed: {:+.2f} m/s  |  steer: {:+.2f} rad/s  '.format(speed, steer)
            )
            sys.stdout.flush()

            rate.sleep()

    finally:
        pub.publish(Twist())
        print('\nStopped.')


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
