from tkinter import *
from tkinter import font
from tkinter import messagebox
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.figure import Figure
import numpy as np
from rkd.transformations import *
from rkd.kinematics import *
global a

class GUI(Tk):

    def __init__(self, *args, **kwargs):
        Tk.__init__(self, *args, **kwargs)        
        self.title_font = font.Font(family = "Helvetica", size = 18, weight = "bold", slant = "italic")
        self.Arial16 = font.Font(family = "Arial", size = 16, weight = "bold")
        self.Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        self.Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others

        self.title("Robot Kinematics")
        img = PhotoImage(file = "img/robot.png")        
        self.call("wm", "iconphoto", self._w, img)
        self.geometry("1200x650+300+10")        
        barMenu = Menu(self)
        menuInfo = Menu(barMenu)
        menuhelp = Menu(barMenu)        
        menuInfo.add_command(label = "Open Message", command = self.info)
        menuhelp.add_command(label = "Open Message", command = self.help)        
        barMenu.add_cascade(label = "Information", menu = menuInfo)
        barMenu.add_cascade(label = "Help", menu = menuhelp)        
        self.config(menu = barMenu)
        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (main, transformations, forward_kinematics, rotations):
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("main")

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()

    def info(self):
        messagebox.showinfo("Information","Version: 1.0\n\nMIT License\n\nCopyright (c) 2018 IRO\n\nPermission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the Software), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\n\nThe above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.\n\nTHE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.")

    def help(self):
        messagebox.showinfo("Help", "Hello world")

class main(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        label = Label(self, text="Robot Kinematics", font=controller.title_font)
        label.pack(side="top", fill="x", pady=10)        
        btn_transformations = Button(self, text = "Transformations", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations"))
        btn_transformations.pack(padx = 10, pady = 20)
        btn_kinematics = Button(self, text = "Forward Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("forward_kinematics"))
        btn_kinematics.pack(padx = 10, pady = 10)
        btn_rkinematics = Button(self, text = "Inverse Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1")# command = self.m_inverse_kinematics)
        btn_rkinematics.pack(padx = 10, pady = 10)
        btn_dkinematics = Button(self, text = "Differential Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1")# command = self.m_differential_kinematics)
        btn_dkinematics.pack(padx = 10, pady = 10)

#First level of windows

class transformations(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        label = Label(self, text="Transformations", font = controller.title_font)
        label.pack(side="top", fill="x", pady=10)
        btn_rotations = Button(self, text = "Rotations", font = controller.Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("rotations"))
        btn_rotations.pack(padx = 10, pady = 20)
        btn_parameterization = Button(self, text = "Parameterization\nof\nrotations", font = controller.Arial14, width = 20, height = 5, borderwidth = 5)# cursor = "hand1")
        btn_parameterization.pack(padx = 10, pady = 20)
        btn_axis_angle = Button(self, text = "Axis / Angle", font = controller.Arial14, width = 20, height = 5, borderwidth = 5, cursor = "hand1")
        btn_axis_angle.pack(padx = 10, pady = 20)
        btn_htmDH = Button(self, text = "Matrix DH", font = controller.Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1")
        btn_htmDH.pack(padx = 10, pady = 10)        
        btn_back = Button(self, text = "Back", font = controller.Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("main")) 
        btn_back.pack(padx = 10, pady = 10)

class forward_kinematics(Frame):    
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        btn_back = Button(self, text = "Back", font = controller.Arial14, width = 15, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("main"))
        btn_back.pack()
        f = Figure(figsize = (5,5), dpi = 100)
        a = f.add_subplot(111, projection="3d")
        a.plot([1,2,3,4,5,6,7,8],[5,6,1,3,8,9,3,5])

        canvas = FigureCanvasTkAgg(f, self)  # A tk.DrawingArea.
        canvas.draw()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas.get_tk_widget().pack(side= TOP, fill=BOTH, expand=True)
        

#Second level of windows

class rotations(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        label = Label(frame1, text = "Rotations", font = controller.title_font)
        label.pack(side = "top", fill = "x", padx = 5, pady = 10)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Axis:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)        
        txt_axis = Entry(frame1, font = controller.Arial14)
        txt_axis.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Angle:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        txt_angle = Entry(frame1, font = controller.Arial14)
        txt_angle.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        f = Figure(figsize = (5,5), dpi = 100)
        a = f.add_subplot(111, projection='3d')
        a.mouse_init(self)     
        a.clear()
        canvas = FigureCanvasTkAgg(f, self)  # A tk.DrawingArea.
        canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        btn_go = Button(frame1, text = "GO", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.go_pulse(txt_axis.get(), txt_angle.get(), a, canvas))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(frame1, text = "Back", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations")) 
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        

    #Functions for buttons
    
    def go_pulse(self, axis, angle, ax, canvas):
        if ((axis == "") or (angle == "")):
            messagebox.showwarning("Warning", "Check the TextBox")

        if ((axis != "") or (angle != "")):
            answer = messagebox.askquestion("Important to answer", "Are you entering the angles in degrees?")
            if ((answer == "sí") or (answer == "Sí") or (answer == "yes") or (answer == "Yes")):             
                a = ax
                canvas = canvas
                a.clear()
                self.draw_uvw(np.eye(4), a)
                if ((axis == "x") or (axis == "X")):                    
                    rot_x = rotx(float(angle), True)                    
                    self.draw_uvw_rot(rot_x, a)

                canvas.draw()
                
            
                

    def draw_uvw(self, T, ax):
        O=(0,0,0)
        U=T[:3,0]
        V=T[:3,1]
        W=T[:3,2]        
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(U[0]),float(U[1]),float(U[2]),color="red", alpha = 0.4, label = "Xo") # Eje u
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(V[0]),float(V[1]),float(V[2]),color="green", alpha = 0.4, label = "Yo") # Eje v
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(W[0]),float(W[1]),float(W[2]),color="blue", alpha = 0.4, label = "Zo") # Eje w
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])
        ax.legend()
        ax.set_aspect("equal")
        return

    def draw_uvw_rot(self, T, ax):
        O=(0,0,0)
        U=T[:3,0]
        V=T[:3,1]
        W=T[:3,2]        
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(U[0]),float(U[1]),float(U[2]),color="red", label = "Xr") # Eje u
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(V[0]),float(V[1]),float(V[2]),color="green", label = "Yr") # Eje v
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(W[0]),float(W[1]),float(W[2]),color="blue", label = "Zr") # Eje w
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])
        ax.legend()
        ax.set_aspect("equal")
        return

if __name__ == "__main__":
    app = GUI()
    app.mainloop()