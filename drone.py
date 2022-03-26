from asyncio import constants
from atexit import register
from cProfile import label
from cmath import sqrt, tan
from contextlib import nullcontext
from xmlrpc.client import boolean
import numpy as np
from numpy import ones,vstack
from numpy.linalg import lstsq
from shapely.geometry import LineString
import operator
from datetime import datetime
import tkinter as tk
from PIL import ImageTk, Image  
import math

RANGE_LIMIT = 10

#(m,c)


class Drone():
    id = 0
    equation = [(0,4)] #m,c
    src = (0.0,0.0)
    dst = []
    lauched_time = 0.0
    last_known_position = (-2.0,4.0)
    last_known_speed = 0.0
    speed = []

    def __init__(self,id,src,dst) -> None:
        self.id = id
        self.src = src
        self.dst = dst
    
    #fly x distance in delta time
    #speed (vx,vy)
    def fly(self,speed,delta):
        #update last known position
        self.last_known_position = tuple(map(operator.add,self.last_known_position, speed*delta))
        self.last_known_speed = speed

active_drones = [Drone(0,(-2.0,4.0),[(3.0,4.0)])]




class Example(tk.Frame):
    cur_pos = 0
    src_entry = 0
    label_pointer  = 0
    last_clicked = 0
    src = []
    registered = []
    drone_position = []
    drone_label = []
    last_speed = []
    root = 0
    id = 0
    new_speed = 0.0
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.root = parent
        image1 = Image.open("./Interview drone.png")
        image1 = image1.resize((1000, 500), Image.ANTIALIAS)
        test = ImageTk.PhotoImage(image1)
        
        label1 = tk.Label(image=test)
        label1.image = test
        # Position image
        label1.place(x=0, y=0)
        label1.bind("<Button-1>", self.callback)
        self.label_pointer = label1
        entry1 = tk.Entry (name = "src") 
        entry1.place( x= 500, y=550)
        self.src_entry = entry1

        button1 = tk.Button(text="Submit",command = self.submit)
        button1.place(x=450, y=550)
        button2 = tk.Button(text="Run",command = self.run)
        button2.place(x=400, y=550)

    def submit(self):
        self.last_clicked = 0
        if len(self.registered) == self.id:
            self.registered.insert(self.id,[self.cur_pos])
        else :
            self.registered[self.id].append(self.cur_pos)
        print(self.registered)
        

    def run(self):
        self.new_speed = self.calculate_path((self.registered[self.id][0],self.registered[self.id][1]))
        if self.calculate_path((self.registered[self.id][0],self.registered[self.id][1])) != 0.01:
            print("new speed is :" + str(self.new_speed))
        print("new speed is ", self.new_speed)
        image2 = Image.open("./drone.png")
        image2 = image2.resize((10, 10), Image.ANTIALIAS)
        test = ImageTk.PhotoImage(image2)
        label1 = tk.Label(image=test)
        label1.image = test
        # Position image
        self.drone_label.append(label1)
        label1.place(x=self.registered[self.id][0][0], y=self.registered[self.id][0][1])
        self.drone_position.append(self.registered[self.id][0])
        self.src.append(self.drone_position[self.id])
        _ = self.registered[self.id].pop(0)
        self.fly(self.new_speed,self.registered[self.id][0],self.id)
        self.id += 1
        #self.last_speed.append(new_speed)
        #self.src[self.id],self.new_speed,self.registered[self.id][0],self.id
        
    def fly(self,speed, dst,id):
        speed = 0.01
        delta = 100
        
        if(abs(self.drone_position[id][0]-self.registered[id][0][0])<0.5 and abs(self.drone_position[id][1]-self.registered[id][0][1]) <0.5):
            if len(self.registered[id]) > 1:
                self.registered[id].pop(0)
                self.src[id] = self.drone_position[id]
                self.after(delta, self.fly,self.src[id],self.registered[id][0],id)
                
                print(self.drone_position[id])
                return
            else:
                return

        direction_x = 1
        direction_y = 1
        m = abs((dst[1] - self.drone_position[id][1])/(dst[0]-self.drone_position[id][0]))
        vx = math.sqrt(abs(speed**2/(1+m)))
        vy = math.sqrt(speed**2 - vx**2)
        #print(vx,vy)
        if (dst[0] < self.drone_position[id][0]):
            direction_x = -1
        if (dst[1] < self.drone_position[id][1]):
            direction_y = -1
        self.drone_label[id].place(x = self.drone_position[id][0]+vx*delta*direction_x, y = self.drone_position[id][1]+vy*delta*direction_y)
        temp =(self.drone_position[id][0] + vx*delta*direction_x, self.drone_position[id][1] + vy*delta*direction_y)
        self.drone_position[id] = temp
        self.after(delta, self.fly,self.src[id],dst,id)

    def callback(self,event):
        if self.last_clicked != 0 :
            self.cur_pos = 0
            self.last_clicked.destroy()
        print("clicked at", event.x, event.y)
        self.src_entry.delete(0,tk.END)
        self.src_entry.insert(0,str(event.x))
        self.cur_pos = (event.x,event.y)
        image2 = Image.open("./circle.png")
        image2 = image2.resize((10, 10), Image.ANTIALIAS)
        test = ImageTk.PhotoImage(image2)
        label1 = tk.Label(image=test)
        label1.image = test
        # Position image
        label1.place(x=event.x, y=event.y)
        self.last_clicked = label1

    def calculate_path (self,points) -> float:
        # if abs(dst-src) > RANGE_LIMIT:
        #   pass

        #get linear equation
        x_coords, y_coords = zip(*points)
        A = vstack([x_coords,ones(len(x_coords))]).T
        m, c = lstsq(A, y_coords)[0]
        print("Line Solution is y = {m}x + {c}".format(m=m,c=c))

        #iterate through the list of active drones
        for id in range(self.id):
            for i in range(len(self.registered[id])):
                #src,dst
                line = LineString([points[0],points[1]])
                other = LineString([self.drone_position[id],self.registered[id][i]])

                #check if the line section intersect
                if(line.intersects(other)) :
                    #if intersect, find the collision point
                    M = ((self.registered[id][i][1] - self.src[id][0])/abs(self.registered[id][i][1]-self.src[id][0]))
                    C = self.src[id][1]-self.src[id][0]*M
                    a = np.asarray([(M,1), (m,1)])
                    b = np.asarray((C,c))
                    ans = np.linalg.solve(a,b)
                    time_left_old = math.hypot(ans[0]-self.drone_position[id][0],ans[1]-self.drone_position[id][1])/0.01
                    time_left_new = math.hypot(ans[0]-points[0][0],ans[1]-points[0][1])/0.01
                    if(abs(time_left_new-time_left_old) < 100000):
                        return float(math.hypot(ans[0]-points[0][0],ans[1]-points[0][1])/100000.0)
                    
                else :
                    break
        return 0.01

def test():
    #src,dst
    now = datetime.now()
    current_time = now 
    print("Current Time =", current_time)

test()
root = tk.Tk()
root.geometry("1000x600")

Example(root).pack(fill="both", expand=True)
root.mainloop()

