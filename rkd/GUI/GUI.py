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
from rkd.util import *
from rkd.transformations import *
from rkd.mathematical_algorithms import *
from rkd.kinematics import *


class GUI(Tk):

    def __init__(self, *args, **kwargs):
        Tk.__init__(self, *args, **kwargs)        
        self.title_font = font.Font(family = "Helvetica", size = 20, weight = "bold", slant = "italic")
        self.Arial20 = font.Font(family = "Arial", size = 20, weight = "bold")
        self.Arial16 = font.Font(family = "Arial", size = 16, weight = "bold")
        self.Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        self.Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")
        self.Arial10 = font.Font(family = "Arial", size = 10, weight = "bold")

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
        menufile = Menu(barMenu)
        menufile.add_command(label = 'Exit', command = quit)
        menuInfo.add_command(label = "Open Message", command = self.info)
        menuhelp.add_command(label = "Open Message", command = self.help)
        barMenu.add_cascade(label = 'File', menu = menufile)
        barMenu.add_cascade(label = "Information", menu = menuInfo)
        barMenu.add_cascade(label = "Help", menu = menuhelp)
        self.config(menu = barMenu)
        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (main, transformations, forward_kinematics, rotations,parameterization, axis_angle, matrixDH):
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
        btn_axis_angle = Button(self, text = "Axis / Angle", font = controller.Arial14, width = 20, height = 5, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame('axis_angle'))
        btn_axis_angle.pack(padx = 10, pady = 20)
        btn_htmDH = Button(self, text = "Matrix DH", font = controller.Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame('matrixDH'))
        btn_htmDH.pack(padx = 10, pady = 10)        
        btn_back = Button(self, text = "Back", font = controller.Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("main")) 
        btn_back.pack(padx = 10, pady = 10)     

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
        self.txt_angle_x = Entry(frame1, font = controller.Arial14)
        self.txt_angle_x.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Axis Y:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.txt_angle_y = Entry(frame1, font = controller.Arial14)
        self.txt_angle_y.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Axis Z:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)        
        self.txt_angle_z = Entry(frame1, font = controller.Arial14)
        self.txt_angle_z.pack(side = TOP, padx = 25, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        f = Figure(figsize = (5,5), dpi = 100)
        a = f.add_subplot(111, projection='3d')        
        a.clear()
        self.draw_uvw(np.eye(4), a)
        canvas = FigureCanvasTkAgg(f, self)  # A tk.DrawingArea.
        canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        canvas.draw()        
        btn_go = Button(frame1, text = "GO", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.GO(self.txt_angle_x.get(), self.txt_angle_y.get(), self.txt_angle_z.get(), a, canvas))
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
        self.txt_angle_x.delete(0, 'end')
        self.txt_angle_y.delete(0, 'end')
        self.txt_angle_z.delete(0, 'end')

class parameterization(Frame):    
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = "Parameterization of rotations", font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Rotation matrix:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.txt_rotation_matrix = Entry(frame1, font = controller.Arial14)
        self.txt_rotation_matrix.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Types of angles:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.option_angles = ttk.Combobox(frame1, font = controller.Arial14, width = 25)
        self.option_angles.pack(side = TOP, padx = 50, pady = 2)
        self.option_angles["values"] = ['' ,"Euler angles", "Roll, Pitch, Yaw angles"]
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = "Choose an option:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.options_angles_euler = ttk.Combobox(frame1, font = controller.Arial14, width = 5)
        self.options_angles_euler.pack(side = TOP, padx = 5, pady = 2)
        self.options_angles_euler["values"] = ['' ,"XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"]
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = 'Theta angle solutions:', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.solutions = ttk.Combobox(frame1, font = controller.Arial14, width = 5)
        self.solutions.pack(side = TOP, padx = 5, pady = 2)
        self.solutions['values'] = ['', '# 1', '# 2']
        Label(frame1, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 5)
        btn_go = Button(frame1, text = "GO", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.GO(self.txt_rotation_matrix.get(), self.option_angles.get(), self.options_angles_euler.get(), self.solutions.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 5)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 5)
        btn_back = Button(frame1, text = "Back", font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations")) 
        btn_back.pack(side = TOP, padx = 5, pady = 5)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 25)
        Label(self, text = 'RESULTS', font = controller.title_font).pack(side = TOP, padx = 5, pady = 50)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = '\u03c6 :', font = controller.Arial16).pack(side = TOP, padx = 50, pady = 2)
        self.Phi_value = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.Phi_value.pack(side = TOP, padx = 50, pady = 2)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = '\u03b8 :', font =controller. Arial16).pack(side = TOP, padx = 50, pady = 2)
        self.Theta_value = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.Theta_value.pack(side = TOP, padx = 50, pady = 2)
        Label(self, text = "").pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = '\u03c8 :', font = controller.Arial16).pack(side = TOP, padx = 50, pady = 2)
        self.Psi_value = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.Psi_value.pack(side = TOP, padx = 50, pady = 2)

        #Funtions for Buttons

    def GO(self, rotation_matrix, type_of_angles, combination_euler, solutions):
        matrix = rotation_matrix
        type_a = type_of_angles
        combination_e = combination_euler
        sol = solutions

        if matrix == '':
            messagebox.showwarning("Warning", "Check your 'Matrix'")
        if sol == '':
            messagebox.showwarning('Warning', "Check your 'Theta angle solutions'")
        if type_a == '':
            messagebox.showwarning('Warning', "Check your 'Types of angles'")

        matrix = '['+matrix+']'
        H = eval(matrix)
        matrix1 = np.array(H)        
        try:
            nrow, ncolumn = matrix1.shape
            if ((nrow != 3) and (ncolumn != 3)):
                messagebox.showerror('Error', 'Check that your Rotation Matrix is ​​3x3')

            if ((nrow == 3) and (ncolumn == 3)):
            
                if ((type_a == "Roll, Pitch, Yaw angles") and (matrix != '') and (sol != '') and ((combination_e == "XYX") or (combination_e == "XZX") or (combination_e == "YXY") or (combination_e == "YZY") or (combination_e == "ZXZ") or (combination_e == "ZYZ"))):
                    messagebox.showerror('Error', "You can't use a combination of angles with this type of angle")
                    messagebox.showinfo('Information', "If you want to use a combination of angles, we suggest using 'Euler angles' in the 'Types of angles' section OR leave in the clean the combination of angles")

                if ((type_a == 'Roll, Pitch, Yaw angles') and (combination_e == '') and (matrix != '') and (sol != '')):
                    answer = messagebox.askquestion('Important to answer','Do you want the angles in degrees?')

                    if ((answer == "sí") or (answer == "Sí") or (answer == "SI") or (answer == "SÍ") or (answer == "yes") or (answer == "Yes") or (answer == "YES")):
                        if sol == '# 1':                    
                            results = rot2RPY(matrix1, True)                        
                            
                        elif sol == '# 2':                        
                            results = rot2RPY(matrix1, True, True)                        

                    if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                        if sol == '# 1':                        
                            results = rot2RPY(matrix1)                        

                        elif sol == '# 2':                        
                            results = rot2RPY(matrix1, False, True)                         

                if ((type_a == 'Euler angles') and (matrix != '') and (sol != '') and (combination_e == '')):
                    messagebox.showwarning('Warning', 'You must choose a combination of angles')

                if ((type_a == 'Euler angles') and (matrix != '') and (sol != '') and (combination_e != '')):
                    answer = messagebox.askquestion('Important to answer','Do you want the angles in degrees?')

                    if ((answer == "sí") or (answer == "Sí") or (answer == "SI") or (answer == "SÍ") or (answer == "yes") or (answer == "Yes") or (answer == "YES")):
                        if sol == '# 1':                        
                            results = rot2eul(matrix1, combination_e, True)                        
                            
                        elif sol == '# 2':                        
                            results = rot2eul(matrix1, combination_e, True, True)                                                

                    if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                        if sol == '# 1':                        
                            results = rot2eul(matrix1, combination_e)                        

                        elif sol == '# 2':
                            results = rot2eul(matrix1, combination_e, False, True)
                
                r = []
                r.append((results))    
                result = np.array(r)
                phi = result[0,0]
                phi = round(phi, 5)
                theta = result[0,1]
                theta = round(theta, 5)  
                psi = result[0,2]
                psi = round(psi, 5)
                self.Phi_value.configure(text = phi)
                self.Theta_value.configure(text = theta)
                self.Psi_value.configure(text = psi)

        except:
            messagebox.showerror('Error', 'Check your entered values')

    def reset(self):
        self.txt_rotation_matrix.delete(0, 'end')
        self.option_angles.delete(0, 'end')
        self.options_angles_euler.delete(0, 'end')
        self.solutions.delete(0, 'end')
        self.Phi_value.configure(text = '')
        self.Theta_value.configure(text = '')
        self.Psi_value.configure(text = '')
        

class axis_angle(Frame):
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller 
        Label(self, text = 'Axis / Angle', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)   
        Label(self, text = "Rotation matrix:", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.txt_rotation_matrix = Entry(self, font = controller.Arial14)
        self.txt_rotation_matrix.pack(side = TOP, padx = 5, pady = 2)
        btn_go = Button(self, text = 'GO', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.GO(self.txt_rotation_matrix.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_reset = Button(self, text = 'Reset', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(self, text = 'Back', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('transformations'))
        btn_back.pack(side = TOP, padx = 5, pady = 10)      
        Label(self, text = 'RESULTS', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 20)
        Label(self, text = '\u03b8 :', font = controller. Arial14).pack(side = TOP, padx = 50, pady = 2)
        self.Theta_value = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.Theta_value.pack(side = TOP, padx = 50, pady = 2)
        Label(self, text = "", font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'k :', font = controller. Arial14).pack(side = TOP, padx = 50, pady = 2)
        self.k_value = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.k_value.pack(side = TOP, padx = 50, pady = 2)

    def GO(self, matrix):
        if matrix == '':
            messagebox.showwarning('Warning', "Check your 'Rotation Matrix'")
        if matrix != '':            
            H = eval(matrix)
            matrix = np.array(H)
            try:
                nrow, ncolumn = matrix.shape
                if ((nrow != 3) and (ncolumn != 3)):
                    messagebox.showerror('Error', 'Check that your Rotation Matrix is ​​3x3')

                if ((nrow == 3) and (ncolumn == 3)):

                    answer = messagebox.askquestion('Important to answer', 'Do you want the angle in degrees?')

                    if ((answer == 'sí') or (answer == 'si') or (answer == 'Sí') or (answer == 'Si') or (answer == 'SÍ') or (answer == 'SI') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                        theta, k = rot2axa(matrix, True)                    

                    if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                        theta, k = rot2axa(matrix)
                    
                    kx = k[0,0]
                    ky = k[1,0]
                    kz = k[2,0]
                    kx = round(kx, 5)
                    ky = round(ky, 5)
                    kz = round(kz, 5)

                    self.Theta_value.configure(text = str(theta))
                    self.k_value.configure(text = '['+str(kx)+', '+str(ky)+', '+str(kz)+']')
            except:
                messagebox.showerror('Error', 'Check your entered values')

    def reset(self):
        self.txt_rotation_matrix.delete(0, 'end')
        self.Theta_value.configure(text = '')
        self.k_value.configure(text = '')

class matrixDH(Frame):    
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Matrix DH (Denavir - Hartenberg)', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = '', font = controller.Arial16).pack(side = TOP, padx = 35, pady = 15)
        Label(frame1, text = 'ai:', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 2)
        self.ai = Entry(frame1, font = controller.Arial14)
        self.ai.pack(side = TOP, padx = 15, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 5)
        Label(frame1, text = '\u03b1i:', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 2)
        self.alphai = Entry(frame1, font = controller.Arial14)
        self.alphai.pack(side = TOP, padx = 15, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 5)
        Label(frame1, text = 'di:', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 2)
        self.di = Entry(frame1, font = controller.Arial14)
        self.di.pack(side = TOP, padx = 15, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 5)
        Label(frame1, text = '\u03b8i:', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 2)
        self.thetai = Entry(frame1, font = controller.Arial14)
        self.thetai.pack(side = TOP, padx = 15, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 35, pady = 5)
        btn_go = Button(frame1, text = 'GO', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.GO(self.ai.get(), self.alphai.get(), self.di.get(), self.thetai.get()))
        btn_go.pack(side = TOP, padx = 35, pady = 10)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 35, pady = 10)
        btn_back = Button(frame1, text = 'Back', font = controller.Arial14, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('transformations'))
        btn_back.pack(side = TOP, padx = 35, pady = 10)
        Label(self, text = 'RESULTS', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 50)        
        Label(self, text = 'Matrix', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = '', font = controller.Arial16).pack(side = TOP, padx = 35, pady = 50)
        self.matrixDH_value = Label(self, text = '', font = controller.Arial20, fg = 'red')
        self.matrixDH_value.pack(side = TOP, padx = 10, pady = 10)

    def GO(self, ai, alphai, di, thetai):

        if ai == '':
            messagebox.showwarning('Warning', "Check your 'ai'")

        if alphai == '':
            messagebox.showwarning('Warning', "Check your '\u03b1i'")

        if di == '':
            messagebox.showwarning('Warning', "Check your 'di'")

        if thetai == '':
            messagebox.showwarning('Warning', "Check your '\u03b8i'")

        if ((ai != '') or (alphai != '') or (di != '') or (thetai != '')):
            answer = messagebox.askquestion('Important to answer','Are you entering the angles in degrees?')

            ai = float(ai)
            alphai = float(alphai)
            di = float(di)
            thetai = float(thetai)

            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                DH = htmDH(ai, alphai, di, thetai, True)                

            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                DH = htmDH(ai, alphai, di, thetai)
            
            DH = str(DH)
            DH = DH[1:-1]
            self.matrixDH_value.configure(text = ' '+DH)

    def reset(self):
        self.ai.delete(0, 'end')
        self.alphai.delete(0, 'end')
        self.di.delete(0, 'end')
        self.thetai.delete(0, 'end')
        self.matrixDH_value.configure(text = '')

class forward_kinematics(Frame):    
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Forward Kinematics', font = controller.title_font).pack(side = TOP, padx = 10, pady = 15)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = 'Choose the number of degrees of freedom:', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)        
        self.dof = ttk.Combobox(frame1, font = controller.Arial12, width = 3)
        self.dof.pack(side = TOP, padx = 5, pady = 2)
        self.dof['values'] = ['', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
        Label(frame1, text = 'DH1 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH1 = Entry(frame1, font = controller.Arial12)
        self.DH1.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH2 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH2 = Entry(frame1, font = controller.Arial12)
        self.DH2.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH3 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH3 = Entry(frame1, font = controller.Arial12)
        self.DH3.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH4 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH4 = Entry(frame1, font = controller.Arial12)
        self.DH4.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH5 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH5 = Entry(frame1, font = controller.Arial12)
        self.DH5.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH6 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH6 = Entry(frame1, font = controller.Arial12)
        self.DH6.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH7 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH7 = Entry(frame1, font = controller.Arial12)
        self.DH7.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH8 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH8 = Entry(frame1, font = controller.Arial12)
        self.DH8.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH9 :', font = controller.Arial12).pack(side = TOP, padx = 5, pady = 2)
        self.DH9 = Entry(frame1, font = controller.Arial12)
        self.DH9.pack(side = TOP, padx = 5, pady = 2)        
        btn_go = Button(frame1, text = 'GO', font = controller.Arial12, width = 10, height = 1, borderwidth = 2, cursor = 'hand1', command = lambda: self.GO(self.dof.get(), self.DH1.get(), self.DH2.get(), self.DH3.get(), self.DH4.get(), self.DH5.get(), self.DH6.get(), self.DH7.get(), self.DH8.get(), self.DH9.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 5)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial12, width = 10, height = 1, borderwidth = 2, cursor = 'hand1')
        btn_reset.pack(side = TOP, padx = 5, pady = 5)
        btn_back = Button(frame1, text = "Back", font = controller.Arial12, width = 10, height = 1, borderwidth = 2, cursor = "hand1", command = lambda: controller.show_frame("main"))
        btn_back.pack(side = TOP, padx = 5, pady = 5)
        Label(self, text = 'RESULTS', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 50)        
        Label(self, text = 'Matrix', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        self.matrixDH_value = Label(self, text = '', font = controller.Arial20, fg = 'red')
        self.matrixDH_value.pack(side = TOP, padx = 10, pady = 10)
        Label(self, text = '', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 20)
        Label(self, text = 'Position of the manipulator', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 5)
        self.position_x = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.position_x.pack(side = TOP, padx = 10, pady = 2)
        self.position_y = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.position_y.pack(side = TOP, padx = 10, pady = 2)
        self.position_z = Label(self, text = '', font = controller.Arial16, fg = 'red')
        self.position_z.pack(side = TOP, padx = 10, pady = 2)

    def GO(self, dof, DH1, DH2, DH3, DH4, DH5, DH6, DH7, DH8, DH9):
        if dof == '':
            messagebox.showwarning('Warning', 'You must choose an option')
        if dof == '1':
            if DH1 == '':
                messagebox.showwarning('Warning', 'Check your DH1')
            if ((DH2 != '') or (DH3 != '') or (DH4 != '') or (DH5 != '') or (DH6 != '') or (DH7 != '') or (DH8 != '') or (DH9 != '')):
                messagebox.showerror('Error', 'You must leave the text boxes of DH2 to DH9')
            if ((DH1 != '') and (DH2 == '') and (DH3 == '') and (DH4 == '') and (DH5 == '') and (DH6 == '') and (DH7 == '') and (DH8 == '') and (DH9 == '')):
                DH1 = '['+'['+DH1+']'+']'
                dh1 = eval(DH1)
                dh1 = np.array(dh1)                
                try:
                    nrow1, ncolumn1 = dh1.shape
                    if ((nrow1 != 1) and (ncolumn1 != 4)):
                        messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')

                    if ((nrow1 == 1) and (ncolumn1 == 4)):
                        answer = messagebox.askquestion('Important to answer', 'Are you entering the angles in degrees?')
                        dh1_ai = dh1[0,0]
                        dh1_alphai = dh1[0,1]
                        dh1_di = dh1[0,2]
                        dh1_thetai = dh1[0,3]
                        if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                            DH = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai, True)                            
                        if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                            DH = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai)
                        px = DH[0,3]
                        py = DH[1,3]
                        pz = DH[2,3]
                        px = round(px, 5)
                        py = round(py, 5)
                        pz = round(pz, 5)
                        px = str(px)
                        py = str(py)
                        pz = str(pz)
                        DH = str(DH)
                        DH = DH[1:-1]
                        self.matrixDH_value.configure(text = ' '+DH)
                        self.position_x.configure(text = 'X: '+px)
                        self.position_y.configure(text = 'Y: '+py)
                        self.position_z.configure(text = 'Z: '+pz)
                except:
                    messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')

        if dof == '2':
            if ((DH1 == '') and (DH2 == '')):
                messagebox.showwarning('Warning', 'Check your DH1 and DH2')
            if ((DH3 != '') or (DH4 != '') or (DH5 != '') or (DH6 != '') or (DH7 != '') or (DH8 != '') or (DH9 != '')):
                messagebox.showerror('Error', 'You must leave the text boxes of DH3 to DH9')
            if ((DH1 != '') and (DH2 != '') and (DH3 == '') and (DH4 == '') and (DH5 == '') and (DH6 == '') and (DH7 == '') and (DH8 == '') and (DH9 == '')):
                DH1 = '['+'['+DH1+']'+']'
                dh1 = eval(DH1)
                dh1 = np.array(dh1)
                DH2 = '['+'['+DH2+']'+']'
                dh2 = eval(DH2)
                dh2 = np.array(dh2)
                try:
                    nrow1, ncolumn1 = dh1.shape
                    nrow2, ncolumn2 = dh2.shape
                    if ((nrow1 != 1) and (nrow2 != 1) and (ncolumn1 != 4) and (ncolumn2 != 4)):
                        messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')
                    if ((nrow1 == 1) and (nrow2 == 1) and (ncolumn1 == 4) and (ncolumn2 == 4)):
                        answer = messagebox.askquestion('Important to answer', 'Are you entering the angles in degrees?')
                        dh1_ai = dh1[0,0]
                        dh1_alphai = dh1[0,1]
                        dh1_di = dh1[0,2]
                        dh1_thetai = dh1[0,3]
                        dh2_ai = dh2[0,0]
                        dh2_alphai = dh2[0,1]
                        dh2_di = dh2[0,2]
                        dh2_thetai = dh2[0,3]
                        if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                            DH1 = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai,True)
                            DH2 = htmDH(dh2_ai, dh2_alphai, dh2_di, dh2_thetai, True)
                            DH = m_mult(DH1, DH2)
                        if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                            DH1 = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai)
                            DH2 = htmDH(dh2_ai, dh2_alphai, dh2_di, dh2_thetai)
                            DH = m_mult(DH1, DH2)
                        px = DH[0,3]
                        py = DH[1,3]
                        pz = DH[2,3]
                        px = round(px, 5)
                        py = round(py, 5)
                        pz = round(pz, 5)
                        px = str(px)
                        py = str(py)
                        pz = str(pz)
                        DH = str(DH)
                        DH = DH[1:-1]
                        self.matrixDH_value.configure(text = ' '+DH)
                        self.position_x.configure(text = 'X: '+px)
                        self.position_y.configure(text = 'Y: '+py)
                        self.position_z.configure(text = 'Z: '+pz)
                except:
                    messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')
        if dof == '3':
            if ((DH1 == '') and (DH2 == '') and (DH3 == '')):
                messagebox.showwarning('Warning', 'Check your DH1 and DH2')
            if ((DH4 != '') or (DH5 != '') or (DH6 != '') or (DH7 != '') or (DH8 != '') or (DH9 != '')):
                messagebox.showerror('Error', 'You must leave the text boxes of DH4 to DH9')
            if ((DH1 != '') and (DH2 != '') and (DH3 != '') and (DH4 == '') and (DH5 == '') and (DH6 == '') and (DH7 == '') and (DH8 == '') and (DH9 == '')):
                DH1 = '['+'['+DH1+']'+']'
                dh1 = eval(DH1)
                dh1 = np.array(dh1)
                DH2 = '['+'['+DH2+']'+']'
                dh2 = eval(DH2)
                dh2 = np.array(dh2)
                DH3 = '['+'['+DH3+']'+']'
                dh3 = eval(DH3)
                dh3 = np.array(dh3)
                try:
                    nrow1, ncolumn1 = dh1.shape
                    nrow2, ncolumn2 = dh2.shape
                    nrow3, ncolumn3 = dh3.shape
                    if ((nrow1 != 1) and (nrow2 != 1) and (nrow3 != 1) and (ncolumn1 != 4) and (ncolumn2 != 4) and (ncolumn3 != 4)):
                        messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')
                    if ((nrow1 == 1) and (nrow2 == 1) and (nrow3 == 1) and (ncolumn1 == 4) and (ncolumn2 == 4) and (ncolumn3 == 4)):
                        answer = messagebox.askquestion('Important to answer', 'Are you entering the angles in degrees?')
                        dh1_ai = dh1[0,0]
                        dh1_alphai = dh1[0,1]
                        dh1_di = dh1[0,2]
                        dh1_thetai = dh1[0,3]
                        dh2_ai = dh2[0,0]
                        dh2_alphai = dh2[0,1]
                        dh2_di = dh2[0,2]
                        dh2_thetai = dh2[0,3]
                        dh3_ai = dh3[0,0]
                        dh3_alphai = dh3[0,1]
                        dh3_di = dh3[0,2]
                        dh3_thetai = dh3[0,3]
                        if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                            DH1 = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai,True)
                            DH2 = htmDH(dh2_ai, dh2_alphai, dh2_di, dh2_thetai, True)
                            DH3 = htmDH(dh3_ai, dh3_alphai, dh3_di, dh3_thetai, True)
                            DH = m_mult(DH1, DH2, DH3)
                        if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                            DH1 = htmDH(dh1_ai, dh1_alphai, dh1_di, dh1_thetai)
                            DH2 = htmDH(dh2_ai, dh2_alphai, dh2_di, dh2_thetai)
                            DH3 = htmDH(dh3_ai, dh3_alphai, dh3_di, dh3_thetai)
                            DH = m_mult(DH1, DH2, DH3)
                        px = DH[0,3]
                        py = DH[1,3]
                        pz = DH[2,3]
                        px = round(px, 5)
                        py = round(py, 5)
                        pz = round(pz, 5)
                        px = str(px)
                        py = str(py)
                        pz = str(pz)
                        DH = str(DH)
                        DH = DH[1:-1]
                        self.matrixDH_value.configure(text = ' '+DH)
                        self.position_x.configure(text = 'X: '+px)
                        self.position_y.configure(text = 'Y: '+py)
                        self.position_z.configure(text = 'Z: '+pz)
                except:
                    messagebox.showerror('Error', 'There must be 4 parameters of DH, you are entering more or less')
        if dof == '9':
            DH1 = '['+'['+DH1+']'+']'
            dh1 = eval(DH1)
            dh1 = np.array(dh1)
            DH2 = '['+'['+DH2+']'+']'
            dh2 = eval(DH2)
            dh2 = np.array(dh2)
            DH3 = '['+'['+DH3+']'+']'
            dh3 = eval(DH3)
            dh3 = np.array(dh3)
            DH4 = '['+'['+DH4+']'+']'
            dh4 = eval(DH4)
            dh4 = np.array(dh4)
            DH5 = '['+'['+DH5+']'+']'
            dh5 = eval(DH5)
            dh5 = np.array(dh5)
            DH6 = '['+'['+DH6+']'+']'
            dh6 = eval(DH6)
            dh6 = np.array(dh6)
            DH7 = '['+'['+DH7+']'+']'
            dh7 = eval(DH7)
            dh7 = np.array(dh7)
            DH8 = '['+'['+DH8+']'+']'
            dh8 = eval(DH8)
            dh8 = np.array(dh8)
            DH9 = '['+'['+DH9+']'+']'
            dh9 = eval(DH9)
            dh9 = np.array(dh9)


if __name__ == "__main__":
    app = GUI()
    app.mainloop()