#!/usr/bin/env python3

import rospy
from std_msgs.msg import Char
import sys, select, tty, termios

msg = """
Reading from the keyboard!
---------------------------
Moving around:
   w
q  s  e    (rotate)
   x

w: Forward
x: Backward
q: Rotate CCW
e: Rotate CW
s: Home Position (Stop)

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/cmd_char', Char, queue_size=1)

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key:
                char_msg = Char()
                char_msg.data = ord(key) # Convert character to its ASCII integer value
                pub.publish(char_msg)
                if (key == '\x03'): # Ctrl+C
                    break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
