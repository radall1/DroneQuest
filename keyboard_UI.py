#!/usr/bin/python3

import tkinter as tk
import rospy
from std_msgs.msg import String

def send_topic(gesture_publisher, message):
    gesture_publisher.publish(message)

def create_button(root, text, command, bg_color):
    button = tk.Button(root, text=text, width=10, height=2, command=command, bg=bg_color, fg="black")
    return button

def main():
    rospy.init_node('hand_sign_recognition', anonymous=True)

    # Publisher which will publish to the topic 
    gesture_publisher = rospy.Publisher("/gesture/hand_sign", String, queue_size=10)

    root = tk.Tk()
    root.title("Button Controller")
    root.configure(bg="light blue")

    bg_color_up = "sky blue"
    up_button = create_button(root, "Forward", lambda: send_topic(gesture_publisher, "Forward"), bg_color_up)
    up_button.grid(row=0, column=1)

    bg_color_down = "cornflower blue"
    down_button = create_button(root, "Backward", lambda: send_topic(gesture_publisher, "Backward"), bg_color_down)
    down_button.grid(row=2, column=1)

    bg_color_left = "light steel blue"
    left_button = create_button(root, "Left", lambda: send_topic(gesture_publisher, "Left"), bg_color_left)
    left_button.grid(row=1, column=0)

    bg_color_right = "deep sky blue"
    right_button = create_button(root, "Right", lambda: send_topic(gesture_publisher, "Right"), bg_color_right)
    right_button.grid(row=1, column=2)

    root.mainloop()

if __name__ == "__main__":
    main()

