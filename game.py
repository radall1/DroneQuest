import tkinter as tk
from pathlib import Path
import time
import random
import rospy
from geometry_msgs.msg import TransformStamped
import os

# Get the current working directory where the program is located
current_dir = Path(os.getcwd())

# Define the relative path of the ASSETS folder
relative_assets_path = Path("frames")

# Combine the current directory with the relative path to get the full ASSETS_PATH
ASSETS_PATH = current_dir / relative_assets_path


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

class GameApp:
    def __init__(self, root):

        #remove unneccessary roots
        self.width = 1280
        self.height = 832

        self.root = root
        self.root.title("DroneQuest")

        self.root.geometry(str(self.width) + "x" + str(self.height))
        self.root.configure(bg = "#FFFFFF")

        self.canvas = tk.Canvas(
            self.root,
            bg = "#FFFFFF",
            height = self.height,
            width = self.width,
            bd = 0,
            highlightthickness = 0,
            relief = "ridge"
        )

        self.canvas.place(x = 0, y = 0)
        self.image_image_1 = tk.PhotoImage(file=relative_to_assets("image_1.png"))
        self.image_image_2 = tk.PhotoImage(file=relative_to_assets("image_2.png"))
        self.image_image_3 = tk.PhotoImage(file=relative_to_assets("image_3.png"))
        self.image_image_4 = tk.PhotoImage(file=relative_to_assets("image_4.png"))
        self.image_image_5 = tk.PhotoImage(file=relative_to_assets("image_5.png"))
        self.image_image_6 = tk.PhotoImage(file=relative_to_assets("image_6.png"))
        self.image_image_7 = tk.PhotoImage(file=relative_to_assets("image_7.png"))

        self.image = self.canvas.create_image(
            640.0,
            416.0,
            image=self.image_image_1
        )

        self.label_instruction = self.canvas.create_text(600, 157, text="", font=("ArcadeClassic", 60, "bold"), fill="green")
        self.label_distance = self.canvas.create_text(1125, 510, text="", font = ("ArcadeClassic", 50, "bold"), fill = "green")
        self.timer = self.canvas.create_text(1125, 330, text="", font = ("ArcadeClassic", 50, "bold"), fill = "green")

        self.winner_time = self.canvas.create_text(450, 680, text="", font = ("ArcadeClassic", 50, "bold"), fill = "black")
        self.loser_time = self.canvas.create_text(910, 540, text="", font = ("ArcadeClassic", 40, "bold"), fill = "black")

        self.subscriber = rospy.Subscriber("/vicon/GREG/GREG", TransformStamped, self.drone_data_callback)

        self.start_status = False
        self.instructions = []
        self.current_instruction_index = 0
        self.start_time = 0
        self.players = ["Player 1", "Player 2"]
        self.current_player_index = 0
        self.player_times = []
        self.x = 0
        self.y = 0

        self.root.resizable(True, True)
        self.root.bind('<space>', self.start_game)
        self.root.mainloop()

    def generate_instructions(self):
        if self.instructions:
            self.instructions = []

        sample=[1, 2, 3, 4, 5, 6, 7]
        sample = random.sample(sample,k=3)
        self.instructions.append(sample[0])
        self.instructions.append(sample[1])
        self.instructions.append(sample[2])

    def draw_circle(self, instruction_number):
        
        width = 90      # The circle's radius
        height = 90
        instruction_coordinates = {
            1: (204, 276),
            2: (291, 365),
            3: (463, 365),
            4: (117, 448),
            5: (378, 448),
            6: (291, 535),
            7: (550, 535)
        }
        
        x0, y0 = instruction_coordinates.get(instruction_number, (0,0))
        self.canvas.create_rectangle(x0, y0, x0 + width, y0 + height, outline="red", width=15)


    def start_game(self, event):
        self.start_time = time.time()
        self.start_status = True
        self.generate_instructions()
        self.show_next_instruction()
        self.root.unbind('<space>')

    def show_next_instruction(self):
        if self.current_instruction_index < len(self.instructions):
            if self.current_instruction_index==0:
                self.canvas.itemconfigure(self.image, image=self.image_image_2) 
            if self.current_instruction_index==1:
                self.canvas.itemconfigure(self.image, image=self.image_image_3)             
            if self.current_instruction_index==2:
                self.canvas.itemconfigure(self.image, image=self.image_image_4)

            instruction_number = self.instructions[self.current_instruction_index]
            self.canvas.itemconfigure(self.label_instruction, text=instruction_number)
            self.draw_circle(instruction_number)
        else:
            end_time = time.time()
            elapsed_time = round(end_time - self.start_time, 2)
            self.player_times.append(elapsed_time)
            self.current_player_index += 1
            if self.current_player_index < len(self.players):
                self.start_time = time.time()
                self.current_instruction_index = 0
                self.show_next_player()
            else:
                self.show_winner()

    def show_next_player(self):
        self.canvas.itemconfigure(self.timer, text="")
        self.canvas.itemconfigure(self.label_distance, text="")
        self.canvas.itemconfigure(self.label_instruction, text="")

        self.canvas.itemconfigure(self.image, image=self.image_image_5)
        self.root.bind('<space>', self.start_game)
        self.start_status = False


    def show_winner(self):
        self.canvas.itemconfigure(self.timer, text="")
        self.canvas.itemconfigure(self.label_distance, text="")
        self.canvas.itemconfigure(self.label_instruction, text="")

        min_time = min(self.player_times)
        winner_index = self.player_times.index(min_time)

        if winner_index==0:
            self.canvas.itemconfigure(self.image, image=self.image_image_6)
            self.canvas.itemconfigure(self.winner_time, text=self.player_times[0])
            self.canvas.itemconfigure(self.loser_time, text=self.player_times[1])
        if winner_index==1:
            self.canvas.itemconfigure(self.image, image=self.image_image_7)
            self.canvas.itemconfigure(self.winner_time, text=self.player_times[1])
            self.canvas.itemconfigure(self.loser_time, text=self.player_times[0])

    def get_instruction_location(self, instruction_number):
        instruction_locations = {
            1: (2.77, 0.09),
            2: (1.42, -0.06),
            3: (1.41, -1.94),
            4: (-0.03, 0.96),
            5: (-0.09, -1.02),
            6: (-1.09, 0),
            7: (-1.3, -2.37)
        }

        return instruction_locations.get(instruction_number, (0, 0))


    def drone_data_callback(self, data):
        # Process the received data and extract the x and y coordinates
        self.x = data.transform.translation.x
        self.y = data.transform.translation.y

        if self.start_status and self.current_instruction_index < len(self.instructions):
            instruction_location = self.get_instruction_location(self.instructions[self.current_instruction_index])
            distance = round(((self.x - instruction_location[0]) ** 2 + (self.y - instruction_location[1]) ** 2) ** 0.5, 2)
            distance_text = "{} M".format(distance)

            if distance < 0.2:
                time.sleep(3)
                self.current_instruction_index += 1
                self.show_next_instruction()
            else:
                self.canvas.itemconfigure(self.label_distance, text=distance_text)

                elapsed_time = round(time.time() - self.start_time)
                minutes = elapsed_time // 60 
                seconds = elapsed_time % 60
                timer_test = f"{minutes:02d}:{seconds:02d}"
                self.canvas.itemconfigure(self.timer, text=timer_test)

if __name__ == "__main__":
    rospy.init_node("game_node")

    root = tk.Tk()
    app = GameApp(root)
