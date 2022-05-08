import tkinter as tk
import json
import matplotlib
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.patches import Rectangle, Polygon, Circle, FancyArrow
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import ImageTk, Image
import cv2
import threading
import os
import time
from map import obstacles
from map import obstacles as obs, working_area, map_generator, path_planner
from communication.RPI_server import RPI_Communication_Server
from robot_handler import Robot
import robot_handler
import math

from xbee.xbee import Xbee


class GUI:
    def __init__(self, cell_size:int = 50, number_of_robots:int = 10, video_feed=0):
        '''
        :param: cell_size: step in [mm] for map plotting. Should be chosen accordingly to area size.
        :number_of_robots: how many robots will be handled
        '''
        self.cell_size = cell_size
        self.cap = cv2.VideoCapture(video_feed)
        
        # create window for aplication:
        self.window = tk.Tk()
        self.window.protocol("WM_DELETE_WINDOW", self._close_app)

        # some flags:
        self.is_map_drawed = False
        self.is_robot_connected = False

        # elements in aplication's window:
        self.map = plt.Figure(figsize=(6, 4), dpi=100)
        self.ax = self.map.add_subplot(111)
        self.camera_label = tk.Label(
            fg="white", bg="black", width=600, height=400)
        self.robot_communicaton_label_text = tk.Label(
            text="Robot communication status: ", bg="white", fg="black", font=16)
        self.rpi_communicaton_label_text = tk.Label(
            text="Raspberry PI communication status: ", bg="white", fg="black", font=16)
        self.diode = {"green": ImageTk.PhotoImage(Image.open(".\Computer\img\green_diode.png").resize((28, 28),
                                                                                                      Image.ANTIALIAS)), "red": ImageTk.PhotoImage(Image.open(".\Computer\img\\red_diode.png").resize((28, 28), Image.ANTIALIAS))}
        self.robot_diode = tk.Label(image=self.diode["red"],)
        self.rpi_diode = tk.Label(image=self.diode["red"])

        # object for map generaation
        self.raster_map: map_generator.MapGenerator = None
        self.pather = path_planner.PathPlanner(self.raster_map)
        self.robot = robot_handler.Robot(0, [800, 200], 0)
        self.robot_target = [150, 800]
        self.robot.set_target(self.robot_target)
        self.new_robot_target = False

        # robot control - entry
        # potem się to rozszerzy na N robotow
        self.robot_position = np.array([800., 200., 90.])
        self.coord_entry = tk.Entry(fg='black', bg='white', width=20)
        self.coord_label = tk.Label(
            text="Enter target coordinates (x,y,rot):", bg="white", fg="black", font=12)
        self.coord_confirm_button = tk.Button(
            text="Confirm coordinates", width=20, command=self._get_target_coord)

        # robot control - BUTTONS
        self.forward = tk.Button(text="FORWARD", width=10, height=2, font=12)
        self.forward.bind('<ButtonPress-1>', self._forward_button_true)
        self.forward.bind('<ButtonRelease-1>', self._forward_button_false)

        self.backward = tk.Button(text="BACKWARD", width=10, height=2, font=12)
        self.backward.bind('<ButtonPress-1>', self._backward_button_true)
        self.backward.bind('<ButtonRelease-1>', self._backward_button_false)

        self.rotate_right = tk.Button(
            text="ROTATE\nRIGHT", width=8, height=2, font=12)
        self.rotate_right.bind('<ButtonPress-1>', self._turn_right_button_true)
        self.rotate_right.bind('<ButtonRelease-1>',
                               self._turn_right_button_false)

        self.rotate_left = tk.Button(
            text="ROTATE\nLEFT", width=8, height=2, font=12)
        self.rotate_left.bind('<ButtonPress-1>', self._turn_left_button_true)
        self.rotate_left.bind('<ButtonRelease-1>',
                              self._turn_left_button_false)

        self.is_forward_pressed = False
        self.is_backward_pressed = False
        self.is_rotate_right_pressed = False
        self.is_rotate_left_pressed = False

        # robot selection
        self.controlled_robot_id = 1
        self.robots_id = tk.StringVar()
        robots_name = ["Robot_"+str(i+1) for i in range(number_of_robots)]
        self.robots_id.set(robots_name[0])
        self.robot_selection = tk.OptionMenu(
            self.window, self.robots_id, *robots_name)
        self.robot_selection.place(x=950, y=133)
        self.robots_id.trace("w", self._robot_selected)

        self.robot_artist = None
        self.background = None
        self.background_without_path = None

        #console - prealpha
        self.console = tk.Text(height=20, width=110, bg='white', fg='black')
        self.console_button = tk.Button(
            text="Execute line", width=10, font=12, command=self._get_command)
        self.command = ""
        self.line = 1

        #robot communication
        self.xbee = Xbee()

        #robots MAC
        self.robots_MAC={
            "1":"0013A200415E861B"
        }

    def _close_app(self):  # narazie watki sa niezalezne od siebie wiec ich nie lacze
        os._exit(1)

    # dodaje wszystkie elementy gui -> miejsce na kamere, pole do wykresów itp
    def window_configuration(self):
        width = self.window.winfo_screenwidth()
        height = self.window.winfo_screenheight()
        # setting tkinter self.window size
        self.window.geometry("%dx%d" % (width, height))
        self.window.title("Robotic swarm control application")
        self._place_widgets()
        self.is_map_drawed = True

    def _robot_selected(self, *args):
        self.controlled_robot_id = self.robots_id.get().split("_")[1]

    def _get_target_coord(self):
        self.new_robot_target = True
        coord_str = self.coord_entry.get()
        if(coord_str != ""):
            for i in "()":
                coord_str = coord_str.replace(i, "")
            temp = coord_str.split(",")
            self.robot_target = [float(x) for x in temp]
            self.coord_entry.delete(0, tk.END)

    def debug(self, message):
        self.console.insert(tk.END, str(message)+'\n')

    def _get_command(self):
        temp = str(self.line)+".0"
        self.command = self.console.get(temp, tk.END)
        self.line += 1
        self.console.insert(tk.END, '\n')

    def _console_function(self):  # tutaj jest bląd i srednio dziala
        # self.console.place(x=620,y=400)
        # self.console_button.place(x=620,y=725)
        last = self.command
        while(True):
            time.sleep(1)
            if(self.command != "" and self.command != last):
                print(self.command)
            last = self.command

    def draw_figure(self):  # tworzenie pola do wykresów, skalowanie osi itp
        dict_of_obstales, wa = self._read_data()
        axis_step = self.cell_size
        extremes = wa.get_extremes()
        self.ax.set_xlim([0, extremes[1]])
        self.ax.set_ylim([0, extremes[3]])
        # this locator puts ticks at regular intervals
        loc = plticker.MultipleLocator(base=axis_step)
        self.ax.xaxis.set_major_locator(loc)
        # this locator puts ticks at regular intervals
        loc2 = plticker.MultipleLocator(base=axis_step)
        self.ax.yaxis.set_major_locator(loc2)
        x = np.arange(-axis_step, int(extremes[1] + axis_step), axis_step)
        y = np.arange(-axis_step, int(extremes[3] + axis_step), axis_step)
        self.ax.set_xticklabels(x, rotation=45)
        self.ax.set_yticklabels(y)
        for index, label in enumerate(self.ax.xaxis.get_ticklabels()):
            if index % 2 != 0:
                label.set_visible(False)
        for index, label in enumerate(self.ax.yaxis.get_ticklabels()):
            if index % 2 != 0:
                label.set_visible(False)
        self.ax.set_xlabel("X axis [mm]")
        self.ax.set_ylabel("Y axis [mm]")
        self.ax.grid(color='k', linestyle="-.", linewidth=0.5, axis='both')

    def _place_widgets(self):
        # place  all elements in app's window:

        ax = FigureCanvasTkAgg(self.map, self.window)
        ax.get_tk_widget().pack(side=tk.TOP, anchor='nw')

        # communication diodes
        self.robot_communicaton_label_text.place(x=650, y=15)
        self.rpi_communicaton_label_text.place(x=650, y=45)
        self.robot_diode.place(x=1000, y=15)
        self.rpi_diode.place(x=1000, y=45)
        self.rpi_diode.configure(image=self.diode["red"])

        # console
        self.console.place(x=620, y=400)
        self.console_button.place(x=620, y=725)

        # coord label
        self.coord_label.place(x=650, y=100)
        self.coord_entry.place(x=650, y=140)
        self.coord_confirm_button.place(x=790, y=136)

        # controll button
        self.forward.place(x=1200, y=200)
        self.backward.place(x=1200, y=280)
        self.rotate_left.place(x=1090, y=240)
        self.rotate_right.place(x=1330, y=240)

        # camera
        self.camera_label.pack(anchor='w')

    def video_stream(self):
        # funkcja do przechwytywania obrazu z kamery i rzutowania go na label, cv2.VideoCapture(0) - oznacza, że rzutuje obraz z kamery w laptopie
        # Jak ktos ma podpieta dodatkowa kamera to podaje kolejny numer. docelowo bedziemy podawac IP websocket-a z RPI

        # while self.is_map_drawed:# przesyłanie zaczyna sie dopiero jak zostalo stworzone pole mapy
        _, frame = self.cap.read()
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        self.camera_label.configure(image=imgtk)
        self.camera_label.imgtk = imgtk
        self.camera_label.update()

    def _read_data(self):  # funkcja do odczytu plikow
        with open(".\Computer\\resources\\figures.json") as f:
            figures = json.load(f)['figures']
        with open(".\Computer\\resources\obstacles.json") as o:
            obstacles = json.load(o)['obstacles']
        dict_of_obstacles = obs.load_obstacles(obstacles, figures)
        with open(".\Computer\\resources\\area.json") as a:
            area = json.load(a)
        wa = working_area.WorkingArea(area)
        return dict_of_obstacles, wa

    def robot_control(self):
        if self.is_forward_pressed:
            R = self._rotation_matrix(self.robot_position[2]-90)
            local_vector = [0, 2]
            local_vector = R.dot(local_vector)
            self.robot_position[0:2] = [
                self.robot_position[0]+local_vector[0], self.robot_position[1]+local_vector[1]]
        if self.is_backward_pressed:
            R = self._rotation_matrix(self.robot_position[2]-90)
            local_vector = [0, -2]
            local_vector = R.dot(local_vector)
            self.robot_position[0:2] = [
                self.robot_position[0]+local_vector[0], self.robot_position[1]+local_vector[1]]
        if self.is_rotate_right_pressed:
            self.robot_position[2] = float(self.robot_position[2]-0.5)
        if self.is_rotate_left_pressed:
            self.robot_position[2] = float(self.robot_position[2]+0.5)
        self.map.canvas.restore_region(self.background)
        self.robot_artist = self._draw_robot(
            self.robot_position[0:2], self.robot_position[2], self.controlled_robot_id)
        self.ax.draw_artist(self.robot_artist[0])
        self.ax.draw_artist(self.robot_artist[1])
        self.ax.draw_artist(self.robot_artist[2])
        self.map.canvas.blit(self.ax.bbox)
        self.robot_artist[0].remove()
        self.robot_artist[1].remove()
        self.robot_artist[2].remove()

        self.robot._position = self.robot_position[0:2]
        self.robot._orientation = self.robot_position[2]

    def _draw_robot(self, position, rotation, id):
        # rysoanie kola ktore bedzei podstawa robota
        circle = self._draw_circle(
            [position[0], position[1]], radius=75, color='red')
        R = self._rotation_matrix(float(rotation-90))
        # reczy do rysowania strzalki, strzalka przyjmuje wspolrzedne pozatku i dlugosci x,y wzgledem poczatku
        # tworzenie localnych wektorow i ich obracanie, nic ciekawego generalnie
        local_arrow_base_coord = [0, -60]
        local_arrow_base_coord = R.dot(local_arrow_base_coord)
        arrow_base_position = [
            position[x] + local_arrow_base_coord[x] for x in range(len(position))]
        local_arrow_head_coord = [0, 100]
        local_arrow_head_coord = R.dot(local_arrow_head_coord)
        # rysowanie strzalki
        arrow = FancyArrow(arrow_base_position[0], arrow_base_position[1], local_arrow_head_coord[0],
                           local_arrow_head_coord[1], width=15, length_includes_head=True, animated=True)
        arrow = self.ax.add_patch(arrow)  # dodanie strzalki do wykresu
        # dodanie opisu ktory to robot
        temp = "R "+str(id)
        label = self.ax.annotate(temp, xy=(
            position[0], position[1]+75), fontsize=10, ha="center", va="center", weight='bold')  # ,rotation=rotation-90)
        robot = [circle, arrow, label]
        return robot

    def _draw_circle(self, center, radius, color='black'):  # rysuje kolo
        circle = matplotlib.patches.Circle(
            (center), radius, fill=False, ec=color)  # ,animated=True)
        self.ax.add_patch(circle)
        return circle

    def _draw_polygon(self, vertices, color="black"):
        rectangle = Polygon(vertices.transpose(), fill=False, ec=color)
        self.ax.add_patch(rectangle)

    def draw_obstacles(self):
        dict_of_obstacles, wa = self._read_data()
        size = self.cell_size
        self.raster_map = map_generator.MapGenerator(
            size, wa, dict_of_obstacles)
        self.pather.change_map(self.raster_map)
        for cell in self.raster_map.get_closed_cells():
            cell_size = size
            center = tuple(i * cell_size for i in cell)
            square = Rectangle(center, cell_size, cell_size, fc='red')
            self.ax.add_patch(square)
        for cell in self.raster_map.get_free_cells():
            cell_size = size
            center = tuple(i * cell_size for i in cell)
            square = Rectangle(center, cell_size, cell_size,
                               fc=(255/255, 120/255, 0))
            self.ax.add_patch(square)

        self._draw_polygon(wa.get_vertices())  # rysuje kontur stolu
        for i in dict_of_obstacles:
            if (isinstance(dict_of_obstacles[i], obs.Polygon)):
                self._draw_polygon(dict_of_obstacles[i].get_vertices())
            elif(isinstance(dict_of_obstacles[i], obs.Circle)):
                self._draw_circle(dict_of_obstacles[i].get_center(
                ), dict_of_obstacles[i].get_radius())

        _rated_cells, _ = self.pather.get_rated_cells(self.robot)

        def colorFader(mix=0):  # fade (linear interpolate) from color c1 (at mix=0) to c2 (mix=1)
            c1 = (255/255, 100/255, 0)
            c2 = (25/255, 255/255, 0/255)
            c1 = np.array(mpl.colors.to_rgb(c1))
            c2 = np.array(mpl.colors.to_rgb(c2))
            return mpl.colors.to_hex((1-mix)*c1 + mix*c2)
        # gradient depended on distance to the obstacle
        max_distance = max(
            [distance for cell, distance in self.raster_map.get_distance_cells().values()])
        for cell, distance in self.raster_map.get_distance_cells().items():
            if distance[1] is not None and distance[1] >= 2:
                cell_size = self.cell_size
                center = tuple(i * cell_size for i in cell)
                square = Rectangle(center, cell_size, cell_size,
                                   fc=colorFader(distance[1]/max_distance))
                self.ax.add_patch(square)

        self.map.canvas.draw()
        self.background = self.map.canvas.copy_from_bbox(self.ax.bbox)
        self.background_without_path = self.map.canvas.copy_from_bbox(
            self.ax.bbox)

    def draw_path(self):
        self.robot.set_target(self.robot_target)
        _rated_cells, _ = self.pather.get_rated_cells(self.robot)
        self.pather.add_robot(0, self.robot)
        self.pather._determine_paths()
        path = self.pather.get_paths()
        self.background = self.map.canvas.copy_from_bbox(self.ax.bbox)
        self.background = self.background_without_path
        for i in range(1, len(path)):
            cell_size = self.cell_size
            center_i = tuple(i * cell_size for i in path[i])
            center_i_1 = tuple(i*cell_size for i in path[i-1])
            x_data = [center_i_1[0], center_i[0]]
            y_data = [center_i_1[1], center_i[1]]
            self.map.canvas.restore_region(self.background)
            (temp,) = self.ax.plot(x_data, y_data,
                                   color=(0, 0/255, 128/255), linestyle="--")
            self.ax.draw_artist(temp)
            self.map.canvas.blit(self.ax.bbox)
            self.background = self.map.canvas.copy_from_bbox(self.ax.bbox)
        self.background = self.map.canvas.copy_from_bbox(self.ax.bbox)
        self.new_robot_target = False

    def _rotation_matrix(self, angle):
        R = np.array([[np.cos(angle*np.pi/180), -np.sin(angle*np.pi/180)],
                     [np.sin(angle*np.pi/180), np.cos(angle*np.pi/180)]])
        return R

    def _forward_button_true(self, event):
        self.is_forward_pressed = True
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<69>")

    def _forward_button_false(self, event):
        self.is_forward_pressed = False
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<169>")

    def _backward_button_true(self, event):
        self.is_backward_pressed = True
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<70>")

    def _backward_button_false(self, event):
        self.is_backward_pressed = False
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<170>")

    def _turn_right_button_true(self, event):
        self.is_rotate_right_pressed = True
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<71>")

    def _turn_right_button_false(self, event):
        self.is_rotate_right_pressed = False
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<171>")

    def _turn_left_button_true(self, event):
        self.is_rotate_left_pressed = True
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<72>")

    def _turn_left_button_false(self, event):
        self.is_rotate_left_pressed = False
        self.xbee.send_msg_unicast(self.robots_MAC[str(self.controlled_robot_id)],"<172>")


def main():
    App = GUI(cell_size=35)
    App.window_configuration()
    App.video_stream()
    App.window.mainloop()


if __name__ == "__main__":
    main()
