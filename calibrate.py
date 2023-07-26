import tkinter as tk
import rospy
from geometry_msgs.msg import TransformStamped

class GUIApp:
    def __init__(self, root):
        self.root = root
        self.label = tk.Label(root, text="Translation: ", font=("Arial", 100))
        self.label.pack()

        self.subscriber = rospy.Subscriber("/vicon/GREG/GREG", TransformStamped, self.vicon_callback)

    def vicon_callback(self, data):
        # Extract the x, y, and z values from the translation
        x = round(data.transform.translation.x, 2)
        y = round(data.transform.translation.y, 2)
        z = round(data.transform.translation.z, 2)

        # Update the GUI label with the new translation values
        self.label.config(text="Translation: x={}, y={}, z={}".format(x, y, z))

if __name__ == "__main__":
    rospy.init_node("gui_node")
    
    root = tk.Tk()
    app = GUIApp(root)
    root.mainloop()
