import tkinter as tk
import json
from turtle import color
from nbformat import read
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.patches import Rectangle, Polygon, Circle
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import ImageTk, Image
import cv2
import threading

class GUI:
    def __init__(self,figure_width=7,figure_height=4):
        self.window = tk.Tk()
        self.map=plt.Figure(figsize=(figure_width,figure_height),dpi=100)
        self.ax=self.map.add_subplot(111)
        self.label= tk.Label(fg="white",bg="black",width=700,height=700)
        self.Camera_Thread=threading.Thread(target=self.video_stream)
        self.Camera_Thread.start()
        self.if_map_drawed=False

    def __del__(self):
        self.Camera_Thread.join()

    def window_configuration(self): #dodaje wszystkie elementy gui -> miejsce na kamere, pole do wykresów itp
        width= self.window.winfo_screenwidth() 
        height= self.window.winfo_screenheight()
        #setting tkinter self.window size
        self.window.geometry("%dx%d" % (width, height))
        self.window.title("Robotic swarm control application")
        #adding map as a plot 
        self.draw_figure()
        self.if_map_drawed=True
        
    def draw_figure(self,x_lim=2000,y_lim=1600,axis_step=100):
        self.ax.set_xlim([0,x_lim])
        self.ax.set_ylim([0,y_lim])
        loc = plticker.MultipleLocator(base=axis_step) # this locator puts ticks at regular intervals
        self.ax.xaxis.set_major_locator(loc)
        loc2 = plticker.MultipleLocator(base=axis_step) # this locator puts ticks at regular intervals
        self.ax.yaxis.set_major_locator(loc2)
        self.ax.set_xticklabels([x for x in range(-axis_step,x_lim+1,axis_step)],rotation=45)
        self.ax.set_xlabel("X axis [mm]")
        self.ax.set_ylabel("Y axis [mm]")
        self.ax.grid(color='k',linestyle="-.",linewidth=0.5,axis='both')
        ax=FigureCanvasTkAgg(self.map,self.window)
        ax.get_tk_widget().pack(side=tk.TOP,anchor='nw')

    def video_stream(self):
        self.label.pack(anchor='w')
        cap = cv2.VideoCapture(0)
        while self.if_map_drawed:
            _, frame = cap.read()
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.label.configure(image=imgtk)
            self.label.imgtk = imgtk
            self.label.update()

    def read_data(self):
        f=open("figures.json")
        o=open("obstacles.json")
        figures=json.load(f)
        obstacles=json.load(o)
        f.close()
        o.close()
        return figures, obstacles

    def draw_obstacles(self):
        figures,obstacles=self.read_data()
        for i in obstacles["obstacles"]:
            obstacle_id=i["id"]
            obstacle_center=[i["center"]["x"],i["center"]["y"]]
            obstacle_rotation=i["rotation"]
            for j in figures["figures"]:
                if j["id"]==obstacle_id:
                    if j["type"]=="triangle":
                        points=np.zeros((3,2))
                        points=[[j["point_1"]["x"],j["point_1"]["y"]],[j["point_2"]["x"],j["point_2"]["y"]],[j["point_3"]["x"],j["point_3"]["y"]]]
                        self.draw_triangle(obstacle_center,points,obstacle_rotation)
                    elif j["type"]=="circle":
                        self.draw_circle(obstacle_center,j["radius"])
                    elif j["type"]=="rectangle":
                        self.draw_rectangle(obstacle_center,j["length_x"],j["length_y"],obstacle_rotation)

    def draw_circle(self,center,radius):
        theta = np.linspace(0, 2*np.pi, 100)
        x = center[0]+radius*np.cos(theta)
        y = center[1]+radius*np.sin(theta)
        self.ax.plot(x,y)

    def draw_robot(self,position,rotation,id):
        circle = Circle((position[0], position[1]), radius=75, color='red')
        self.ax.add_patch(circle)
        temp="R "+str(id)
        label = self.ax.annotate(temp, xy=(position[0], position[1]), fontsize=15, ha="center",va="center",rotation=rotation-90)
    
    def draw_rectangle(self, center, lenght_x, lenght_y,rotation): #jest gotowa funkcja na prostokąt ale tam jest problem z rotacją. Łatwiej zrobić samemu mnozac przez macierz rotacji
        R=self.rotation_matrix(rotation)
        final_points=np.zeros((4,2))
        local_vectors=np.zeros((4,2))
        local_vectors[0][:]=[-lenght_x/2,-lenght_y/2]
        local_vectors[1][:]=[-lenght_x/2,lenght_y/2]
        local_vectors[2][:]=[lenght_x/2,lenght_y/2]
        local_vectors[3][:]=[lenght_x/2,-lenght_y/2]
        for i in range(4):
            final_points[i][:]=R.dot(local_vectors[i][:])
            for j in range(2):
                final_points[i][j]=final_points[i][j]+center[j]
        rectangle=Polygon(final_points)
        self.ax.add_patch(rectangle)

    def draw_triangle(self,center,points,rotation):
        R=self.rotation_matrix(rotation)
        final_points=np.zeros((3,2))
        for i in range(3):
            final_points[i][:]=R.dot(points[i][:])
            for j in range(2):
                final_points[i][j]=final_points[i][j]+center[j]
        triangle=Polygon(final_points)
        self.ax.add_patch(triangle)

    def rotation_matrix(self,angle):
        R=np.array([[np.cos(angle*np.pi/180), -np.sin(angle*np.pi/180)],[np.sin(angle*np.pi/180),np.cos(angle*np.pi/180)]])
        return R

def start():
    App=GUI()
    App.window_configuration()
    App.draw_obstacles()
    App.window.mainloop()

    
if __name__ == "__main__":
    start()


