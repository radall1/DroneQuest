#!/usr/bin/env python3

from pathlib import Path
import os
import subprocess
import time


# Get the current working directory where the program is located
current_dir = str(Path(os.getcwd()))

def execute_build():
    commands = [
        'cd ' + current_dir + '; catkin build; source devel/setup.bash',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)


def execute_camera():
    commands = [
        'cd ' + current_dir + '; source devel/setup.bash; roslaunch ros_hand_gesture_recognition hand_sign.launch',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)



def execute_vicon():
    commands = [
        'cd ' + current_dir + '; source devel/setup.bash; roslaunch vicon_bridge vicon.launch',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)


def execute_freyja():
    commands = [
        'cd ' + current_dir + '; source devel/setup.bash; roslaunch src/Freyja/freyja_controller.launch total_mass:=1.2',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)

def run_button_controller():
    commands = [
        'cd ' + current_dir + '; source devel/setup.bash; python3 keyboard_UI.py',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)

def execute_game():
    commands = [
        'cd ' + current_dir + '; source devel/setup.bash; python3 game.py',
    ]
    full_command = ' && '.join(commands)
    command = ['gnome-terminal', '--', 'bash', '-c', full_command]
    subprocess.Popen(command)

def execute_all():
    execute_build()
    time.sleep(3)
    execute_vicon()
    time.sleep(3)
    execute_camera()
    time.sleep(3)
    execute_freyja()
    time.sleep(3)
    execute_game()

execute_all()

"""
# Create the main window
window = tk.Tk()
window.title("Launcher")

# Set the window background color
window.configure(background='#c3ddf2')

# Set the font style and color
header_style = ("Times New Roman", 12, "bold")
font_style = ("Times New Roman", 12)
text_color = "black"

# Create a styled frame
style = ttk.Style()
style.configure('TFrame', background='#c3ddf2')
frame = ttk.Frame(window, style='TFrame')
frame.pack(padx=20, pady=10)

# Create a label in the frame
label = ttk.Label(frame, text="Welcome to the DroneQuest!", font=header_style, background='#c3ddf2', foreground=text_color)
label.pack(pady=10)

# Disable focus and highlight behavior of the label
label.focus_set()
label.bind("<FocusIn>", lambda event: label.config(relief=tk.FLAT))
label.bind("<FocusOut>", lambda event: label.config(relief=tk.FLAT))

# Create a button style
button_style = ttk.Style()
button_style.configure('TButton', font=font_style)


# Create a button for launching Vicon
message_button = ttk.Button(frame, text="Launch Vicon", command=execute_vicon, style='TButton')
message_button.pack(pady=10)

# Create a button for executing Camera
roslaunch_button = ttk.Button(frame, text="Launch Camera", command=execute_camera, style='TButton')
roslaunch_button.pack(pady=10)

# Create a button for executing Freyja
roslaunch_button = ttk.Button(frame, text="Launch Freyja", command=execute_freyja, style='TButton')
roslaunch_button.pack(pady=10)

# Create a button for executing Game
roslaunch_button = ttk.Button(frame, text="Launch Game", command=execute_game, style='TButton')
roslaunch_button.pack(pady=10)

# Create a button for executing Game
roslaunch_button = ttk.Button(frame, text="Button Controller", command=run_button_controller, style='TButton')
roslaunch_button.pack(pady=10)

# Create a button for executing Game
roslaunch_button = ttk.Button(frame, text="Instant Start", command=execute_all, style='TButton')
roslaunch_button.pack(pady=10)



# Create a label to display messages
message_label = ttk.Label(window, text="", font=font_style, foreground=text_color)
message_label.pack(pady=10)

# Run the GUI event loop
window.mainloop()
"""
