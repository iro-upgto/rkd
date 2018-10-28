from tkinter import *
from tkinter import font
from tkinter import messagebox
from tkinter import ttk
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.figure import Figure
import numpy as np
from numpy import *
from rkd.abc import *
from rkd.transformations import *
from rkd.mathematical_algorithms import *
from rkd.kinematics import *


class GUI(Tk):

    def __init__(self, *args, **kwargs):
        Tk.__init__(self, *args, **kwargs)        
        self.title_font = font.Font(family = "Helvetica", size = 20, weight = "bold", slant = "italic")
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
        for F in (main, transformations, forward_kinematics, rotations,parameterization):
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
        btn_parameterization = Button(self, text = "Parameterization\nof\nrotations", font = controller.Arial14, width = 20, height = 5, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("parameterization"))
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
        Label(frame1, text = "Axis X:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)        
        txt_angle_x = Entry(frame1, font = controller.Arial14)
        txt_angle_x.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Axis Y:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        txt_angle_y = Entry(frame1, font = controller.Arial14)
        txt_angle_y.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Axis Z:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)        
        txt_angle_z = Entry(frame1, font = controller.Arial14)
        txt_angle_z.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        f = Figure(figsize = (5,5), dpi = 100)
        a = f.add_subplot(111, projection='3d')        
        a.clear()
        self.draw_uvw(np.eye(4), a)
        canvas = FigureCanvasTkAgg(f, self)  # A tk.DrawingArea.
        canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        canvas.draw()
        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        btn_go = Button(frame1, text = "GO", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.GO(txt_angle_x.get(), txt_angle_y.get(), txt_angle_z.get(), a, canvas))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_reset = Button(frame1, text = "Reset", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.reset(a, canvas))
        btn_reset.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(frame1, text = "Back", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations")) 
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        a.mouse_init() # for mouse rotation
        

    #Functions for buttons
    
    def GO(self, x, y, z, ax, canvas):
        if ((x == "") or (y == "") or (z == "")):
            messagebox.showwarning("Warning", "Check the 'Text Boxes'")

        if ((x != "") or (y != "") or (z != "")):
            answer = messagebox.askquestion("Important to answer", "Are you entering the angles in degrees?")
            a = ax
            canvas = canvas
            a.clear()
            self.draw_uvw1(np.eye(4), a)
            if ((answer == "sí") or (answer == "Sí") or (answer == "SI") or (answer == "SÍ") or (answer == "yes") or (answer == "Yes") or (answer == "YES")):
                H = m_mult(rotx(float(x), True),roty(float(y), True),rotz(float(z), True))
                self.draw_uvw_rot(H, a)

            if ((answer == "no") or (answer == "No") or (answer == "NO")):
                H = m_mult(rotx(float(x)),roty(float(y)),rotz(float(z)))
                self.draw_uvw_rot(H, a)


            canvas.draw()



    def draw_uvw(self, T, ax):
        O=(0,0,0)
        U=T[:3,0]
        V=T[:3,1]
        W=T[:3,2]        
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(U[0]),float(U[1]),float(U[2]),color="red", label = "$X$") # Eje u
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(V[0]),float(V[1]),float(V[2]),color="green", label = "$Y$") # Eje v
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(W[0]),float(W[1]),float(W[2]),color="blue", label = "$Z$") # Eje w
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])
        ax.legend(loc = 0)
        ax.set_aspect("equal")
        return

    def draw_uvw1(self, T, ax):
        O=(0,0,0)
        U=T[:3,0]
        V=T[:3,1]
        W=T[:3,2]        
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(U[0]),float(U[1]),float(U[2]),color="red", alpha = 0.4, label = "$X_o$") # Eje u
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(V[0]),float(V[1]),float(V[2]),color="green", alpha = 0.4, label = "$Y_o$") # Eje v
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(W[0]),float(W[1]),float(W[2]),color="blue", alpha = 0.4, label = "$Z_o$") # Eje w
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])
        ax.legend(loc = 0)
        ax.set_aspect("equal")
        return

    def draw_uvw_rot(self, T, ax):
        O=(0,0,0)
        U=T[:3,0]
        V=T[:3,1]
        W=T[:3,2]        
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(U[0]),float(U[1]),float(U[2]),color="red", label = "$X_r$") # Eje u
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(V[0]),float(V[1]),float(V[2]),color="green", label = "$Y_r$") # Eje v
        ax.quiver(float(O[0]),float(O[1]),float(O[2]),float(W[0]),float(W[1]),float(W[2]),color="blue", label = "$Z_r$") # Eje w
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])
        ax.legend(loc = 1)
        ax.set_aspect("equal")
        return

    def reset(self, ax, canvas):
        a = ax
        canvas = canvas
        a.clear()
        self.draw_uvw(np.eye(4), a)
        canvas.draw()

class parameterization(Frame):    
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = "Parameterization of rotations", font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Rotation matrix:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        txt_rotation_matrix = Entry(frame1, font = controller.Arial14)
        txt_rotation_matrix.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Types of angles:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        option_angles = ttk.Combobox(frame1, font = controller.Arial14, width = 25)
        option_angles.pack(side = TOP, padx = 15, pady = 2)
        option_angles["values"] = ['' ,"Euler angles", "Roll, Pitch, Yaw angles"]
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Choose an option:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        options_angles_euler = ttk.Combobox(frame1, font = controller.Arial14, width = 5)
        options_angles_euler.pack(side = TOP, padx = 5, pady = 2)
        options_angles_euler["values"] = ['' ,"XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"]
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = 'Theta angle solutions:', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        solutions = ttk.Combobox(frame1, font = controller.Arial14, width = 5)
        solutions.pack(side = TOP, padx = 5, pady = 2)
        solutions['values'] = ['', '# 1', '# 2']
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        btn_go = Button(frame1, text = "GO", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.GO(txt_rotation_matrix.get(), option_angles.get(), options_angles_euler.get(), solutions.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 10)        
        btn_back = Button(frame1, text = "Back", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations")) 
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 50)
        Label(self, text = 'Phi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
        Label(self, text = phi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'Theta:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
        Label(self, text = theta, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'Psi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
        Label(self, text = psi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)

    def GO(self, rotation_matrix, type_of_angles, combination_euler, solutions):
        matrix = rotation_matrix
        type_a = type_of_angles
        combination_e = combination_euler
        sol = solutions

        Arial16 = font.Font(family = 'Arial', size = 16, weight = 'bold')

        if ((matrix == '') or (sol == '')):
            messagebox.showwarning("Warning", "Check your 'Matrix' or 'Theta Solutions'")

        if type_a == '':
            messagebox.showwarning('Warning', "Check you 'Types of angles'")

        if ((type_a == "Roll, Pitch, Yaw angles") and (matrix != '') and (sol != '') and ((combination_e == "XYX") or (combination_e == "XZX") or (combination_e == "YXY") or (combination_e == "YZY") or (combination_e == "ZXZ") or (combination_e == "ZYZ"))):
            messagebox.showerror('Error', "You can't use a combination of angles with this type of angle")
            messagebox.showinfo('Information', "If you want to use a combination of angles, we suggest using 'Euler angles' in the angle types section")

        if ((type_a == 'Roll, Pitch, Yaw angles') and (combination_e == '') and (matrix != '') and (sol != '')):
            answer = messagebox.askquestion('Important to answer','Do you want the angles in degrees?')

            if ((answer == "sí") or (answer == "Sí") or (answer == "SI") or (answer == "SÍ") or (answer == "yes") or (answer == "Yes") or (answer == "YES")):
                if sol == '# 1':
                    H = eval(matrix.split()[0])
                    matrix = np.array(H)
                    results = rot2RPY(matrix, True)
                    r = []
                    r.append((results))    
                    result = np.array(r)
                    phi = result[0,0]
                    phi = round(phi, 5)
                    theta = result[0,1]
                    theta = round(theta, 5)
                    psi = result[0,2]
                    psi = round(psi, 5)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 50)
                    Label(self, text = 'Phi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = phi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Theta:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = theta, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Psi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = psi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                elif sol == '# 2':
                    H = eval(matrix.split()[0])
                    matrix = np.array(H)
                    results = rot2RPY(matrix, True, True)
                    r = []
                    r.append((results))    
                    result = np.array(r)
                    phi = result[0,0]
                    phi = round(phi, 5)
                    theta = result[0,1]
                    theta = round(theta, 5)
                    psi = result[0,2]
                    psi = round(psi, 5)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 50)
                    Label(self, text = 'Phi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = phi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Theta:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = theta, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Psi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = psi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)

            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                if sol == '# 1':
                    H = eval(matrix.split()[0])
                    matrix = np.array(H)
                    results = rot2RPY(matrix)
                    r = []
                    r.append((results))    
                    result = np.array(r)
                    phi = result[0,0]
                    phi = round(phi, 5)
                    theta = result[0,1]
                    theta = round(theta, 5)
                    psi = result[0,2]
                    psi = round(psi, 5)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 50)
                    Label(self, text = 'Phi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = phi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Theta:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = theta, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
                    Label(self, text = 'Psi:', font = Arial16).pack(side = TOP, padx = 100, pady = 2)
                    Label(self, text = psi, font = Arial16, fg = 'red').pack(side = TOP, padx = 100, pady = 2)
                elif sol == '# 2':
                    H = eval(matrix.split()[0])
                    matrix = np.array(H)
                    results = rot2RPY(matrix, False, True)
                    r = []
                    r.append((results))    
                    result = np.array(r)
                    phi = result[0,0]
                    phi = round(phi, 5)
                    theta = result[0,1]
                    theta = round(theta, 5)
                    psi = result[0,2]
                    psi = round(psi, 5)
        

if __name__ == "__main__":
    app = GUI()
    app.mainloop()