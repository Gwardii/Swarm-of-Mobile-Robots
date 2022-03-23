import tkinter as tk
import json
from matplotlib import scale
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.patches import Rectangle, Polygon, Circle, FancyArrow
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import ImageTk, Image
import cv2
import threading
import os
import time

from sympy import true

class GUI:
    def __init__(self,figure_width=6,figure_height=4,N=10):
        self.window = tk.Tk()
        self.map=plt.Figure(figsize=(figure_width,figure_height),dpi=100)
        self.ax=self.map.add_subplot(111)
        self.camera_label= tk.Label(fg="white",bg="black",width=600,height=400)

        self.is_map_drawed=False
        #communication status
        self.is_robot_connected=False
        self.is_rpi_connected=False
        self.robot_communicaton_label_text=tk.Label(text="Robot communication status: ",bg="white",fg="black",font=16)
        self.rpi_communicaton_label_text=tk.Label(text="Raspberry PI communication status: ",bg="white",fg="black",font=16)
        self.diode={"green":ImageTk.PhotoImage(Image.open(".\Computer\green_diode.png").resize((28,28),
        Image.ANTIALIAS)),"red":ImageTk.PhotoImage(Image.open(".\Computer\\red_diode.png").resize((28,28),Image.ANTIALIAS))}
        self.robot_diode=tk.Label(image=self.diode["red"],)
        self.rpi_diode=tk.Label(image=self.diode["green"])
        #robot control - entry
        self.robot_position=np.array([1000,800,90]) #potem się to rozszerzy na N robotow
        self.coord_entry=tk.Entry(fg='black',bg='white',width=20)
        self.coord_label=tk.Label(text="Enter target coordinates (x,y,rot):",bg="white",fg="black",font=12)
        self.coord_confirm_button=tk.Button(text="Confirm coordinates",width=20,command=self.get_target_coord)
        #robot control - BUTTONS
        self.forward=tk.Button(text="FORWARD",width=10,height=2,font=12)
        self.forward.bind('<ButtonPress-1>',self.forward_button_true)
        self.forward.bind('<ButtonRelease-1>',self.forward_button_false)

        self.backward=tk.Button(text="BACKWARD",width=10,height=2,font=12)
        self.backward.bind('<ButtonPress-1>',self.backward_button_true)
        self.backward.bind('<ButtonRelease-1>',self.backward_button_false)

        self.rotate_right=tk.Button(text="ROTATE\nRIGHT",width=8,height=2,font=12)
        self.rotate_right.bind('<ButtonPress-1>',self.turn_right_button_true)
        self.rotate_right.bind('<ButtonRelease-1>',self.turn_right_button_false)

        self.rotate_left=tk.Button(text="ROTATE\nLEFT",width=8,height=2,font=12)
        self.rotate_left.bind('<ButtonPress-1>',self.turn_left_button_true)
        self.rotate_left.bind('<ButtonRelease-1>',self.turn_left_button_false)

        self.is_forward_pressed=False
        self.is_backward_pressed=False
        self.is_rotate_right_pressed=False
        self.is_rotate_left_pressed=False
        #robot selection
        self.controlled_robot_id=1
        self.robots_id = tk.StringVar()
        robots_name=["Robot_"+str(i+1) for i in range(N)]
        self.robots_id.set(robots_name[0])
        self.robot_selection=tk.OptionMenu(self.window,self.robots_id,*robots_name)
        self.robot_selection.place(x=950,y=133)
        self.robots_id.trace("w",self.robot_selected)
        #console - prealpha
        self.console=tk.Text(height=20,width=110,bg='white',fg='black')
        self.console_button=tk.Button(text="Execute line",width=10,font=12,command=self.get_command)
        self.command=""
        self.line=1

        self.camera_Thread=threading.Thread(target=self.video_stream)
        self.camera_Thread.start()
        self.robot_thread=threading.Thread(target=self.robot_thread)
        self.robot_thread.start()

        self.console_Thread=threading.Thread(target=self.console_function)
        self.console_Thread.start()
        
    
    def window_configuration(self): #dodaje wszystkie elementy gui -> miejsce na kamere, pole do wykresów itp
        width= self.window.winfo_screenwidth() 
        height= self.window.winfo_screenheight()
        #setting tkinter self.window size
        self.window.geometry("%dx%d" % (width, height))
        self.window.title("Robotic swarm control application")
        #adding map as a plot 
        self.draw_figure()
        self.is_map_drawed=True
        self.communications_status()
        self.place_coord_entry()
        self.place_controll_buttons()

    def robot_selected(self,*args):
        # self.controlled_robot_id=self.robots_id.get()
        self.controlled_robot_id=self.robots_id.get().split("_")[1]

    def get_target_coord(self):
        coord_str=self.coord_entry.get()
        if(coord_str !=""):
            for i in "()":
                coord_str=coord_str.replace(i,"")
            temp=coord_str.split(",")
            self.robot_position=[float(x) for x in temp]
            self.coord_entry.delete(0,tk.END)

    def get_command(self):
        temp=str(self.line)+".0"
        self.command=self.console.get(temp,tk.END)
        self.line+=1
        self.console.insert(tk.END,'\n')

    def console_function(self): #tutaj jest bląd i srednio dziala
        self.console.place(x=620,y=400)
        self.console_button.place(x=620,y=725)
        last=self.command
        while(True):
            time.sleep(1)
            if(self.command!="" and self.command!=last):
                print(self.command)
            last=self.command
    def draw_figure(self,x_lim=2000,y_lim=1600,axis_step=100): #tworzenie pola do wykresów, skalowanie osi itp
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

    def communications_status(self): #wyswietlanie napisu i diody z aktualnym stanem - połaczono lub nie, narazei bez sprawdzenia zadnego po prostu zielone i czerwone
        #sprawdzenie sie wymyśli jak bedzie komunikacja
        self.robot_communicaton_label_text.place(x=650,y=15)
        self.rpi_communicaton_label_text.place(x=650,y=45)
        self.robot_diode.place(x=1000,y=15)
        self.rpi_diode.place(x=1000,y=45)

    def video_stream(self):
        #funkcja do przechwytywania obrazu z kamery i rzutowania go na label, cv2.VideoCapture(0) - oznacza, że rzutuje obraz z kamery w laptopie
        #Jak ktos ma podpieta dodatkowa kamera to podaje kolejny numer. docelowo bedziemy podawac IP websocket-a z RPI
        self.camera_label.pack(anchor='w')
        cap = cv2.VideoCapture(0)
        while self.is_map_drawed:# przesyłanie zaczyna sie dopiero jak zostalo stworzone pole mapy
            _, frame = cap.read()
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.camera_label.configure(image=imgtk)
            self.camera_label.imgtk = imgtk
            self.camera_label.update()

    def read_data(self): #funkcja do odczytu plikow
        f=open(".\Computer\\figures.json")
        o=open(".\Computer\obstacles.json")
        figures=json.load(f)
        obstacles=json.load(o)
        f.close()
        o.close()
        return figures, obstacles

    def draw_obstacles(self): # odczytuje pliki .json i na podstawie zawartych danych rysuje przefszkody
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

    def robot_thread(self): # chwilowo to wyglada tak, ze robot pojawia sie w miejscu zadanym w tym polu "Enter target coord.."
        while(True):
            #robot=self.draw_robot([random.random()*2000,random.random()*1600],random.random()*180,1)
            d_p=20# co ile ma sie poruszac robot po osiach, obraca sie o 3 razy mniej
            if self.is_forward_pressed:
                R=self.rotation_matrix(self.robot_position[2]-90)
                local_vector=[0,d_p]
                local_vector=R.dot(local_vector)
                self.robot_position[0:2]=[self.robot_position[0]+local_vector[0],self.robot_position[1]+local_vector[1]]
            if self.is_backward_pressed:
                R=self.rotation_matrix(self.robot_position[2]-90)
                local_vector=[0,-d_p]
                local_vector=R.dot(local_vector)
                self.robot_position[0:2]=[self.robot_position[0]+local_vector[0],self.robot_position[1]+local_vector[1]]
            if self.is_rotate_right_pressed:
                self.robot_position[2]-=d_p/3
            if self.is_rotate_left_pressed:
                self.robot_position[2]+=d_p/3

            robot=self.draw_robot(self.robot_position[0:2],self.robot_position[2],self.controlled_robot_id)
            self.map.canvas.draw()
            time.sleep(0.5)
            robot[0].pop(0).remove()
            robot[1].remove()
            robot[2].remove()
            self.map.canvas.draw()

    def draw_circle(self,center,radius,color='blue'):#rysuje kolo
        theta = np.linspace(0, 2*np.pi, 100)
        x = center[0]+radius*np.cos(theta)
        y = center[1]+radius*np.sin(theta)
        circle=self.ax.plot(x,y,color=color)
        return circle

    def draw_robot(self,position,rotation,id):
        circle=self.draw_circle([position[0], position[1]], radius=75, color='red') #rysoanie kola ktore bedzei podstawa robota
        R=self.rotation_matrix(rotation-90)
        #reczy do rysowania strzalki, strzalka przyjmuje wspolrzedne pozatku i dlugosci x,y wzgledem poczatku
        #tworzenie localnych wektorow i ich obracanie, nic ciekawego generalnie 
        local_arrow_base_coord=[0,-60]
        local_arrow_base_coord=R.dot(local_arrow_base_coord)
        arrow_base_position= [position[x] + local_arrow_base_coord[x] for x in range(len(position))]
        local_arrow_head_coord=[0,100]
        local_arrow_head_coord=R.dot(local_arrow_head_coord)
        #rysowanie strzalki
        arrow=FancyArrow(arrow_base_position[0],arrow_base_position[1],local_arrow_head_coord[0],local_arrow_head_coord[1], width=15,length_includes_head=True)
        arrow = self.ax.add_patch(arrow)#dodanie strzalki do wykresu
        #dodanie opisu ktory to robot
        temp="R "+str(id)
        label = self.ax.annotate(temp, xy=(position[0], position[1]+75), fontsize=10, ha="center",va="center", weight='bold')#,rotation=rotation-90)
        robot=[circle,arrow,label]
        return robot

    def draw_rectangle(self, center, lenght_x, lenght_y,rotation,color='blue'): #jest gotowa funkcja na prostokąt ale tam jest problem z rotacją. Łatwiej zrobić samemu mnozac przez macierz rotacji
        R=self.rotation_matrix(rotation)
        final_points=np.zeros((4,2)) #macierz do przechowywania wierzcholkow po obrocie
        local_vectors=np.zeros((4,2)) #lokalne wspolrzedne prostokata
        local_vectors[0][:]=[-lenght_x/2,-lenght_y/2] #przypisanie wierzcholkow na podstawie wymiarow prostokata
        local_vectors[1][:]=[-lenght_x/2,lenght_y/2]
        local_vectors[2][:]=[lenght_x/2,lenght_y/2]
        local_vectors[3][:]=[lenght_x/2,-lenght_y/2]
        for i in range(4): #obliczenie obroonych wektorow marcierz rotacji te sprawy
            final_points[i][:]=R.dot(local_vectors[i][:])
            for j in range(2):
                final_points[i][j]=final_points[i][j]+center[j]
        rectangle=Polygon(final_points,color=color)
        self.ax.add_patch(rectangle)#dodanie figury do wykresu

    def draw_triangle(self,center,points,rotation,color='blue'):
        R=self.rotation_matrix(rotation)
        final_points=np.zeros((3,2))#macierz na koncowe wspołrzedne 
        for i in range(3):#obliczanie obroconych wektorow
            final_points[i][:]=R.dot(points[i][:])
            for j in range(2):
                final_points[i][j]=final_points[i][j]+center[j]
        triangle=Polygon(final_points,color=color)
        self.ax.add_patch(triangle)#dodanie figury do wykresu

    def rotation_matrix(self,angle):
        R=np.array([[np.cos(angle*np.pi/180), -np.sin(angle*np.pi/180)],[np.sin(angle*np.pi/180),np.cos(angle*np.pi/180)]])
        return R
    
    def place_coord_entry(self):
        self.coord_label.place(x=650,y=100)
        self.coord_entry.place(x=650,y=140)
        self.coord_confirm_button.place(x=790,y=136)

    def place_controll_buttons(self):
        self.forward.place(x=1200,y=200)
        self.backward.place(x=1200,y=280)
        self.rotate_left.place(x=1090,y=240)
        self.rotate_right.place(x=1330,y=240)
    
    def forward_button_true(self,event):
        self.is_forward_pressed=True

    def forward_button_false(self,event):
        self.is_forward_pressed=False

    def backward_button_true(self,event):
        self.is_backward_pressed=True

    def backward_button_false(self,event):
        self.is_backward_pressed=False

    def turn_right_button_true(self,event):
        self.is_rotate_right_pressed=True

    def turn_right_button_false(self,event):
        self.is_rotate_right_pressed=False
    
    def turn_left_button_true(self,event):
        self.is_rotate_left_pressed=True

    def turn_left_button_false(self,event):
        self.is_rotate_left_pressed=False

def main():
    App=GUI()
    App.window_configuration()
    App.draw_obstacles()
    App.window.mainloop()
    
    
if __name__ == "__main__":
    main()


