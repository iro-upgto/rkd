from tkinter import *
from tkinter import font
from tkinter import messagebox
from tkinter import ttk
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
import numpy as np
from numpy import *
from scipy.optimize import *
import webbrowser as wb
import os
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
        self.OS = os.name
        if self.OS == 'posix': #Operating systems Linux y Mac OS
            img = PhotoImage(file = "img/robot.png")
        if self.OS == 'nt': #Operating system Windows
            img = PhotoImage(file = "img\robot.png")
        self.call("wm", "iconphoto", self._w, img)
        self.geometry("1200x650+300+10")        
        barMenu = Menu(self)
        menuInfo = Menu(barMenu)
        menuhelp = Menu(barMenu)
        menufile = Menu(barMenu)
        menufile.add_command(label = 'Exit', command = quit)
        menuInfo.add_command(label = "Open Message", command = self.info)
        menuhelp.add_command(label = "Open Manual in Spanish", command = self.manual_spanish)
        barMenu.add_cascade(label = 'File', menu = menufile)
        barMenu.add_cascade(label = "Information", menu = menuInfo)
        barMenu.add_cascade(label = "Help", menu = menuhelp)
        self.config(menu = barMenu)
        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (main, transformations, forward_kinematics, inverse_kinematics, differential_kinematics, newton_raphson, mixed_root, rotations,parameterization, axis_angle, matrixDH):
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

    def manual_spanish(self):
        if self.OS == 'posix':#Operating systems Linux y Mac OS
            wb.open_new(r"Software management manuals/Manual del manejo del software en ESPAÑOL.pdf")
        if self.OS == 'nt':#Operating system Windows
            wb.open_new(r"Software management manuals\Manual del manejo del software en ESPAÑOL.pdf")

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
        btn_invkinematics = Button(self, text = "Inverse Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command =lambda: controller.show_frame('inverse_kinematics'))
        btn_invkinematics.pack(padx = 10, pady = 10)
        btn_dkinematics = Button(self, text = "Differential Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame('differential_kinematics'))
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
        ax.legend(loc = 0)
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
            if ((nrow != 3) or (ncolumn != 3)):
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
            matrix = '['+matrix+']'  
            H = eval(matrix)
            matrix = np.array(H)
            try:
                nrow, ncolumn = matrix.shape
                if ((nrow != 3) or (ncolumn != 3)):
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
        Label(frame1, text = 'DH1 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH1 = Entry(frame1, font = controller.Arial16)
        self.DH1.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH2 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH2 = Entry(frame1, font = controller.Arial16)
        self.DH2.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH3 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH3 = Entry(frame1, font = controller.Arial16)
        self.DH3.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH4 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH4 = Entry(frame1, font = controller.Arial16)
        self.DH4.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH5 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH5 = Entry(frame1, font = controller.Arial16)
        self.DH5.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH6 :', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.DH6 = Entry(frame1, font = controller.Arial16)
        self.DH6.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        btn_go = Button(frame1, text = 'GO', font = controller.Arial16, width = 25, height = 1, borderwidth = 2, cursor = 'hand1', command = lambda: self.GO(self.DH1.get(), self.DH2.get(), self.DH3.get(), self.DH4.get(), self.DH5.get(), self.DH6.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 5)
        btn_kinematics_diagram = Button(frame1, text = 'Kinematics diagram', font = controller.Arial16, width = 25, height = 1, borderwidth = 2, cursor = 'hand1', command = lambda: self.kinematics_diagram())
        btn_kinematics_diagram.pack(side = TOP, padx = 5, pady = 5)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial16, width = 25, height = 1, borderwidth = 2, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 5)
        btn_back = Button(frame1, text = "Back", font = controller.Arial16, width = 25, height = 1, borderwidth = 2, cursor = "hand1", command = lambda: controller.show_frame("main"))
        btn_back.pack(side = TOP, padx = 5, pady = 5)
        Label(self, text = 'RESULTS', font = controller.Arial20).pack(side = TOP, padx = 5, pady = 50)        
        Label(self, text = 'Matrix', font = controller.Arial20).pack(side = TOP, padx = 5, pady = 10)
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

    def GO(self, DH1, DH2, DH3, DH4, DH5, DH6):
        try:
            DH = []
            for dh in [DH1, DH2, DH3, DH4, DH5,DH6]:
                if dh != '':
                    DH.append(eval(dh))
            self.Ts = []
            for k in DH:
                if len(k)>4:
                    self.Ts.append(htmDH(k[0],k[1],k[2],k[3],k[4]))
                else:
                    self.Ts.append(htmDH(k[0],k[1],k[2],k[3]))
                    
            DH = m_mult(*self.Ts)
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
            messagebox.showerror('Error', 'Check your entered data')
            messagebox.showinfo('Information', 'Tip: Probably your mistake is that you are missing or you have too many values in your parameters')
    
    def kinematics_diagram(self):
        self.window_kinematics_diagram = Toplevel()
        self.window_kinematics_diagram.title('Kinematics diagram')        
        f = Figure(figsize = (7,7), dpi = 100)
        self.a = f.add_subplot(111, projection='3d')
        a = self.a
        a.clear()
        self.canvas = FigureCanvasTkAgg(f, self.window_kinematics_diagram)  # A tk.DrawingArea.        
        self.canvas.get_tk_widget().pack(side = BOTTOM, fill=BOTH, expand=True)
        canvas = self.canvas
        Ts = self.Ts
        if self.DH1.get() != '':
            T1_0 = Ts[0]
            A = np.array([0,0,0])
            B = T1_0[:3,3]
            self.xmin = min([A[0],B[0]])
            self.xmax = max([A[0],B[0]])
            self.ymin = min([A[1],B[1]])
            self.ymax = max([A[1],B[1]])
            self.zmin = min([A[2],B[2]])
            self.zmax = max([A[2],B[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0]], 
                   [A[1],B[1]], 
                   [A[2],B[2]], '-o')
            self.draw_uvw(np.eye(4), a)
            self.draw_uvw1(np.eye(4), a)
            self.draw_uvw(T1_0, a)
        
        if ((self.DH1.get() != '') and (self.DH2.get() != '')):
            T1_0 = Ts[0]
            T2_0 = m_mult(Ts[0],Ts[1])

            A = np.array([0,0,0])
            B = T1_0[:3,3]
            C = T2_0[:3,3]            
            self.xmin = min([A[0],B[0],C[0]])
            self.xmax = max([A[0],B[0],C[0]])
            self.ymin = min([A[1],B[1],C[1]])
            self.ymax = max([A[1],B[1],C[1]])
            self.zmin = min([A[2],B[2],C[2]])
            self.zmax = max([A[2],B[2],C[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0],C[0]], 
                   [A[1],B[1],C[1]], 
                   [A[2],B[2],C[2]], '-o')
            self.draw_uvw(np.eye(4), a)
            self.draw_uvw(T1_0, a)
            self.draw_uvw(T2_0, a)
        
        if ((self.DH1.get() != '') and (self.DH2.get() != '') and (self.DH3.get() != '')):
            T1_0 = Ts[0]
            T2_0 = m_mult(Ts[0],Ts[1])
            T3_0 = m_mult(T2_0,Ts[2])

            A = np.array([0,0,0])
            B = T1_0[:3,3]
            C = T2_0[:3,3]
            D = T3_0[:3,3]
            self.xmin = min([A[0],B[0],C[0],D[0]])
            self.xmax = max([A[0],B[0],C[0],D[0]])
            self.ymin = min([A[1],B[1],C[1],D[1]])
            self.ymax = max([A[1],B[1],C[1],D[1]])
            self.zmin = min([A[2],B[2],C[2],D[2]])
            self.zmax = max([A[2],B[2],C[2],D[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0],C[0],D[0]], 
                   [A[1],B[1],C[1],D[1]], 
                   [A[2],B[2],C[2],D[2]], '-o')
            self.draw_uvw(np.eye(4), a)
            self.draw_uvw(T1_0, a)
            self.draw_uvw(T2_0, a)
            self.draw_uvw(T3_0, a)

        if ((self.DH1.get() != '') and (self.DH2.get() != '') and (self.DH3.get() != '') and (self.DH4.get() != '')):
            T1_0 = Ts[0]
            T2_0 = m_mult(Ts[0],Ts[1])
            T3_0 = m_mult(T2_0,Ts[2])
            T4_0 = m_mult(T3_0,Ts[3])

            A = np.array([0,0,0])
            B = T1_0[:3,3]
            C = T2_0[:3,3]
            D = T3_0[:3,3]
            E = T4_0[:3,3]
            self.xmin = min([A[0],B[0],C[0],D[0],E[0]])
            self.xmax = max([A[0],B[0],C[0],D[0],E[0]])
            self.ymin = min([A[1],B[1],C[1],D[1],E[1]])
            self.ymax = max([A[1],B[1],C[1],D[1],E[1]])
            self.zmin = min([A[2],B[2],C[2],D[2],E[2]])
            self.zmax = max([A[2],B[2],C[2],D[2],E[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0],C[0],D[0],E[0]], 
                   [A[1],B[1],C[1],D[1],E[1]], 
                   [A[2],B[2],C[2],D[2],E[2]], '-o')
            self.draw_uvw(np.eye(4), a)
            self.draw_uvw(T1_0, a)
            self.draw_uvw(T2_0, a)
            self.draw_uvw(T3_0, a)
            self.draw_uvw(T4_0, a)

        if ((self.DH1.get() != '') and (self.DH2.get() != '') and (self.DH3.get() != '') and (self.DH4.get() != '') and (self.DH5.get() != '')):
            T1_0 = Ts[0]
            T2_0 = m_mult(Ts[0],Ts[1])
            T3_0 = m_mult(T2_0,Ts[2])
            T4_0 = m_mult(T3_0,Ts[3])
            T5_0 = m_mult(T4_0,Ts[4])

            A = np.array([0,0,0])
            B = T1_0[:3,3]
            C = T2_0[:3,3]
            D = T3_0[:3,3]
            E = T4_0[:3,3]
            F = T5_0[:3,3]
            self.xmin = min([A[0],B[0],C[0],D[0],E[0],F[0]])
            self.xmax = max([A[0],B[0],C[0],D[0],E[0],F[0]])
            self.ymin = min([A[1],B[1],C[1],D[1],E[1],F[1]])
            self.ymax = max([A[1],B[1],C[1],D[1],E[1],F[1]])
            self.zmin = min([A[2],B[2],C[2],D[2],E[2],F[2]])
            self.zmax = max([A[2],B[2],C[2],D[2],E[2],F[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0],C[0],D[0],E[0],F[0]], 
                   [A[1],B[1],C[1],D[1],E[1],F[1]], 
                   [A[2],B[2],C[2],D[2],E[2],F[2]], '-o')
            self.draw_uvw(np.eye(4), a)
            self.draw_uvw(T1_0, a)
            self.draw_uvw(T2_0, a)
            self.draw_uvw(T3_0, a)
            self.draw_uvw(T4_0, a)
            self.draw_uvw(T5_0, a)

        if ((self.DH1.get() != '') and (self.DH2.get() != '') and (self.DH3.get() != '') and (self.DH4.get() != '') and (self.DH5.get() != '') and (self.DH6.get() != '')):
            T1_0 = Ts[0]
            T2_0 = m_mult(Ts[0],Ts[1])
            T3_0 = m_mult(T2_0,Ts[2])
            T4_0 = m_mult(T3_0,Ts[3])
            T5_0 = m_mult(T4_0,Ts[4])
            T6_0 = m_mult(T5_0,Ts[5])

            A = np.array([0,0,0])
            B = T1_0[:3,3]
            C = T2_0[:3,3]
            D = T3_0[:3,3]
            E = T4_0[:3,3]
            F = T5_0[:3,3]
            G = T6_0[:3,3]
            self.xmin = min([A[0],B[0],C[0],D[0],E[0],F[0],G[0]])
            self.xmax = max([A[0],B[0],C[0],D[0],E[0],F[0],G[0]])
            self.ymin = min([A[1],B[1],C[1],D[1],E[1],F[1],G[1]])
            self.ymax = max([A[1],B[1],C[1],D[1],E[1],F[1],G[1]])
            self.zmin = min([A[2],B[2],C[2],D[2],E[2],F[2],G[2]])
            self.zmax = max([A[2],B[2],C[2],D[2],E[2],F[2],G[2]])
            xmin = self.xmin - 0.1*(self.xmax - self.xmin)
            xmax = self.xmax + 0.1*(self.xmax - self.xmin)
            ymin = self.ymin - 0.1*(self.ymax - self.ymin)
            ymax = self.ymax + 0.1*(self.ymax - self.ymin)
            zmin = self.zmin - 0.1*(self.zmax - self.zmin)
            zmax = self.zmax + 0.1*(self.zmax - self.zmin)
            a.plot([A[0],B[0],C[0],D[0],E[0],F[0],G[0]], 
                   [A[1],B[1],C[1],D[1],E[1],F[1],G[1]], 
                   [A[2],B[2],C[2],D[2],E[2],F[2],G[2]], '-o')
            self.draw_uvw(np.eye(4), a)            
            self.draw_uvw(T1_0, a)
            self.draw_uvw(T2_0, a)
            self.draw_uvw(T3_0, a)
            self.draw_uvw(T4_0, a)
            self.draw_uvw(T5_0, a)
            self.draw_uvw(T6_0, a)
        axis_max = max(xmax,ymax,zmax)
        axis_min = min(xmin,ymin,zmin)
        a.set_xlim(axis_min,axis_max)
        a.set_ylim(axis_min,axis_max)
        a.set_zlim(axis_min,axis_max)
        canvas.draw()
        a.set_xlabel('X axis')
        a.set_ylabel('Y axis')
        a.set_zlabel('Z axis')
        a.set_aspect("equal")
        a.mouse_init()
        self.wait_window(self.window_kinematics_diagram)
    
    def draw_uvw(self,H,ax):
        u = H[:3,0]
        v = H[:3,1]
        w = H[:3,2]
        o = H[:3,3]
        xmin = self.xmin - 0.1*(self.xmax - self.xmin)
        xmax = self.xmax + 0.1*(self.xmax - self.xmin)
        ymin = self.ymin - 0.1*(self.ymax - self.ymin)
        ymax = self.ymax + 0.1*(self.ymax - self.ymin)
        zmin = self.zmin - 0.1*(self.zmax - self.zmin)
        zmax = self.zmax + 0.1*(self.zmax - self.zmin)      
        axis_max = max(xmax,ymax,zmax)
        axis_min = min(xmin,ymin,zmin)
        quiver_size = 0.1*(np.absolute(axis_max - axis_min))
        ax.quiver(o[0],o[1],o[2],u[0],u[1],u[2],color="r", length=quiver_size)
        ax.quiver(o[0],o[1],o[2],v[0],v[1],v[2],color="g", length=quiver_size)
        ax.quiver(o[0],o[1],o[2],w[0],w[1],w[2],color="b", length=quiver_size)
        ax.set_aspect('equal')

    def draw_uvw1(self,H,ax):
        u = H[:3,0]
        v = H[:3,1]
        w = H[:3,2]
        o = H[:3,3]
        xmin = self.xmin - 0.1*(self.xmax - self.xmin)
        xmax = self.xmax + 0.1*(self.xmax - self.xmin)
        ymin = self.ymin - 0.1*(self.ymax - self.ymin)
        ymax = self.ymax + 0.1*(self.ymax - self.ymin)
        zmin = self.zmin - 0.1*(self.zmax - self.zmin)
        zmax = self.zmax + 0.1*(self.zmax - self.zmin)      
        axis_max = max(xmax,ymax,zmax)
        axis_min = min(xmin,ymin,zmin)
        quiver_size = 0.1*(np.absolute(axis_max - axis_min))
        ax.quiver(o[0],o[1],o[2],u[0],u[1],u[2],color="r", length=quiver_size, label = "$X$")
        ax.quiver(o[0],o[1],o[2],v[0],v[1],v[2],color="g", length=quiver_size, label = "$Y$")
        ax.quiver(o[0],o[1],o[2],w[0],w[1],w[2],color="b", length=quiver_size, label = "$Z$")
        ax.legend()
        ax.set_aspect('equal')

    def reset(self):
        self.DH1.delete(0, 'end')
        self.DH2.delete(0, 'end')
        self.DH3.delete(0, 'end')
        self.DH4.delete(0, 'end')
        self.DH5.delete(0, 'end')
        self.DH6.delete(0, 'end')
        self.matrixDH_value.configure(text = '')
        self.position_x.configure(text = '')
        self.position_y.configure(text = '')
        self.position_z.configure(text = '')
        self.a.clear()

class inverse_kinematics(Frame):
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Inverse Kinematics', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        btn_newton_raphson = Button(self, text = 'Inverse Kinematics (N - R)', font = controller.Arial16, width = 35, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('newton_raphson'))
        btn_newton_raphson.pack(side = TOP, padx = 5, pady = 10)
        btn_mixed = Button(self, text = 'Inverse Kinematics (Mixed)', font = controller.Arial16, width = 35, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('mixed_root'))
        btn_mixed.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(self, text = 'Back', font = controller.Arial16, width = 35, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('main'))
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        
class newton_raphson(Frame):
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Inverse Kinematics for NEWTON - RAPHSON', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)        
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = 'J (Matrix J):', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.matrixJ = Entry(frame1, font = controller.Arial14)
        self.matrixJ.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 5)
        Label(frame1, text = 'b (Matrx F):', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.matrixF = Entry(frame1, font = controller.Arial14)
        self.matrixF.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 5)
        Label(frame1, text = 'Number of variables', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.number_var = ttk.Combobox(frame1, font = controller.Arial14, width = 3)
        self.number_var.pack(side = TOP, padx = 5, pady = 2)
        self.number_var['values'] = ['', '1', '2', '3', '4', '5', '6']
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 5)
        Label(frame1, text = 'Initials Values:', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 2)
        self.x0 = Entry(frame1, font = controller.Arial14)
        self.x0.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial14).pack(side = TOP, padx = 5, pady = 10)
        btn_go = Button(frame1, text = 'GO', font = controller.Arial14, width = 15, height = 1, borderwidth = 5, cursor = 'hand1', command = lambda: self.GO(self.matrixJ.get(), self.matrixF.get(), self.number_var.get(), self.x0.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial14, width = 15, height = 1, borderwidth = 5, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(frame1, text = 'Back', font = controller.Arial14, width = 15, height = 1, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('inverse_kinematics'))
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'RESULTS', font = controller.Arial20).pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'Approximate values:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.results_variable1 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable1.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable2 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable2.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable3 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable3.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable4 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable4.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable5 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable5.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable6 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable6.pack(side = TOP, padx = 5, pady = 2)
        Label(self, text = 'Error range by variable:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.error_variable1 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable1.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable2 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable2.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable3 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable3.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable4 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable4.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable5 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable5.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable6 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable6.pack(side = TOP, padx = 5, pady = 2)
        Label(self, text = 'Number of iterations:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.iterations = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.iterations.pack(side = TOP, padx = 5, pady = 2)

    def GO(self, matrixJ, matrixF, number_var, x0):
        if matrixJ == '':
            messagebox.showwarning('Warning', 'Matrix J is empty')
        if matrixF == '':
            messagebox.showwarning('Warning', 'Matrix F is empty')
        if number_var == '':
            messagebox.showwarning('Warning', 'Number of variables is empty')
        if x0 == '':
            messagebox.showwarning('Warning', 'Initials Values is empty')        
        
        nvar = number_var
        mJ = matrixJ
        mF = matrixF

        if ((mJ != '') and (mF != '') and (x0 != '') and (nvar != '')):
            x_0 = '['+'['+x0+']'+']'
            x_0 = eval(x_0)
            x_0 = np.array(x_0)
            x0 = eval(x0)
            x0 = np.array(x0)
            try:
                nrowx0, ncolumnx0 = x_0.shape
                if nvar == '1':
                    if nrowx0 == 1:
                        if ncolumnx0 != 1:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 1:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                error1 = x[0]
                                result1 = float(result1)
                                error1 = float(error1)
                                result1 = round(result1, 5)
                                error1 = round(error1, 5)
                                result1 = str(result1)
                                error1 = str(error1)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.error_variable1.configure(text = 'Error 1 = '+error1)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                error1 = x[0]
                                result1 = float(result1)
                                error1 = float(error1)
                                result1 = round(result1, 5)
                                error1 = round(error1, 5)
                                result1 = str(result1)
                                error1 = str(error1)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.error_variable1.configure(text = 'Error 1 = '+error1)
                                self.iterations.configure(text = k)
                if nvar == '2':
                    if nrowx0 == 1:
                        if ncolumnx0 != 2:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 2:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):                            
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)                            
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                error1 = x[0]
                                error2 = x[1]
                                result1 = float(result1)
                                result2 = float(result2)
                                error1 = float(error1)
                                error2 = float(error2)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                error1 = str(error1)
                                error2 = str(error2)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)                            
                                result1 = X0[0,0]
                                result2 = X0[0,1]
                                error1 = x[0,0]
                                error2 = x[0,1]
                                result1 = str(result1)
                                result2 = str(result2)
                                result1 = float(result1)
                                result2 = float(result2)
                                error1 = float(error1)
                                error2 = float(error2)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error1 = str(error1)
                                error2 = str(error2)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.iterations.configure(text = k)
                if nvar == '3':
                    if nrowx0 == 1:
                        if ncolumnx0 != 3:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 3:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):                            
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.iterations.configure(text = k)
                if nvar == '4':
                    if nrowx0 == 1:
                        if ncolumnx0 != 4:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 4:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):                            
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.iterations.configure(text = k)
                if nvar == '5':
                    if nrowx0 == 1:
                        if ncolumnx0 != 5:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 5:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):                            
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                result5 = x0[4]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                error5 = x[4]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                result5 = float(result5)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                error5 = float(error5)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                result5 = round(result5, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                error5 = round(error5, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                result5 = str(result5)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                error5 = str(error5)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.results_variable5.configure(text = 'Variable 5 = '+result5)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.error_variable5.configure(text = 'Variable 5 = '+error5)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                result5 = x0[4]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                error5 = x[4]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                result5 = float(result5)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                error5 = float(error5)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                result5 = round(result5, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                error5 = round(error5, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                result5 = str(result5)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                error5 = str(error5)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.results_variable5.configure(text = 'Variable 5 = '+result5)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.error_variable5.configure(text = 'Variable 5 = '+error5)
                                self.iterations.configure(text = k)
                if nvar == '6':
                    if nrowx0 == 1:
                        if ncolumnx0 != 6:
                            messagebox.showerror('Error', 'Check your initial values, probably do not match the number of variables')
                        if ncolumnx0 == 6:
                            answer = messagebox.askquestion('Important to answer', 'If you have entered angles as variables, do you want your results to return in degrees?')
                            if ((answer == 'si') or (answer == 'sí') or (answer == 'Si') or (answer == 'Sí') or (answer == 'SI') or (answer == 'SÍ') or (answer == 'yes') or (answer == 'Yes') or (answer == 'YES')):                            
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF, True)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                result5 = x0[4]
                                result6 = x0[5]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                error5 = x[4]
                                error6 = x[5]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                result5 = float(result5)
                                result6 = float(result6)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                error5 = float(error5)
                                error6 = float(error6)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                result5 = round(result5, 5)
                                result6 = round(result6, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                error5 = round(error5, 5)
                                error6 = round(error6, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                result5 = str(result5)
                                result6 = str(result6)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                error5 = str(error5)
                                error6 = str(error6)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.results_variable5.configure(text = 'Variable 5 = '+result5)
                                self.results_variable6.configure(text = 'Variable 6 = '+result6)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.error_variable5.configure(text = 'Variable 5 = '+error5)
                                self.error_variable6.configure(text = 'Variable 6 = '+error6)
                                self.iterations.configure(text = k)
                            if ((answer == 'no') or (answer == 'No') or (answer == 'NO')):
                                X0, x, k = self.inverse_kinematics(self.j1, self.b1, x0, nvar, mJ, mF)
                                X0 = np.array(X0)
                                x = np.array(x)
                                result1 = X0[0]
                                result2 = X0[1]
                                result3 = x0[2]
                                result4 = x0[3]
                                result5 = x0[4]
                                result6 = x0[5]
                                error1 = x[0]
                                error2 = x[1]
                                error3 = x[2]
                                error4 = x[3]
                                error5 = x[4]
                                error6 = x[5]
                                result1 = float(result1)
                                result2 = float(result2)
                                result3 = float(result3)
                                result4 = float(result4)
                                result5 = float(result5)
                                result6 = float(result6)
                                error1 = float(error1)
                                error2 = float(error2)
                                error3 = float(error3)
                                error4 = float(error4)
                                error5 = float(error5)
                                error6 = float(error6)
                                result1 = round(result1, 5)
                                result2 = round(result2, 5)
                                result3 = round(result3, 5)
                                result4 = round(result4, 5)
                                result5 = round(result5, 5)
                                result6 = round(result6, 5)
                                error1 = round(error1, 5)
                                error2 = round(error2, 5)
                                error3 = round(error3, 5)
                                error4 = round(error4, 5)
                                error5 = round(error5, 5)
                                error6 = round(error6, 5)
                                result1 = str(result1)
                                result2 = str(result2)
                                result3 = str(result3)
                                result4 = str(result4)
                                result5 = str(result5)
                                result6 = str(result6)
                                error1 = str(error1)
                                error2 = str(error2)
                                error3 = str(error3)
                                error4 = str(error4)
                                error5 = str(error5)
                                error6 = str(error6)
                                self.results_variable1.configure(text = 'Variable 1 = '+result1)
                                self.results_variable2.configure(text = 'Variable 2 = '+result2)
                                self.results_variable3.configure(text = 'Variable 3 = '+result3)
                                self.results_variable4.configure(text = 'Variable 4 = '+result4)
                                self.results_variable5.configure(text = 'Variable 5 = '+result5)
                                self.results_variable6.configure(text = 'Variable 6 = '+result6)
                                self.error_variable1.configure(text = 'Variable 1 = '+error1)
                                self.error_variable2.configure(text = 'Variable 2 = '+error2)
                                self.error_variable3.configure(text = 'Variable 3 = '+error3)
                                self.error_variable4.configure(text = 'Variable 4 = '+error4)
                                self.error_variable5.configure(text = 'Variable 5 = '+error5)
                                self.error_variable6.configure(text = 'Variable 6 = '+error6)
                                self.iterations.configure(text = k)                

            except:
                messagebox.showerror('Error', 'It caused an error check your data')

    def inverse_kinematics(self, J, b, x0, number_of_variables, matrixJ, matrixF, deg = False,eps = 1e-6):
        """
        Calculates the inverse kinematics from a numerical method

        ** Solved from the numerical method 'Newton - Raphson' **

        *deg* : bool
            ¿Is theta given in degrees?

        IMPORT: You must create your two functions of J (Jacobian), b (initial values ​​evaluated)
        """
        k = 1 #iterations
        num_var = number_of_variables
        while True:
            x = np.linalg.solve(J(matrixJ, x0, num_var), -b(matrixF, x0, num_var))
            if norm(x) < eps: break
            x0 += x
            k += 1 #Increase of iterations counter
        if deg:
            return rad2deg(x0), x, k
        return x0, x, k

    def j1(self, J, x0, number_of_variables):
        """
        Calculates the Jacobian matrix evaluated with the initial values
        """
        var = ['var1', 'var2', 'var3', 'var4', 'var5', 'var6', 'var7', 'var8', 'var9']
        num_var = number_of_variables

        if num_var == '1':
            var1 = var[0]
            if var1 == 'var1':
                var1 = x0            
        if num_var == '2':
            var1 = var[0]
            var2 = var[1]
            if ((var1 == 'var1') and (var2 == 'var2')):
                var1, var2 = x0
        if num_var == '3':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3')):
                var1, var2, var3 = x0
        if num_var == '4':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            var4 = var[3]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4')):
                var1, var2, var3, var4 = x0
        if num_var == '5':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            var4 = var[3]
            var5 = var[4]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5')):
                var1, var2, var3, var4, var5 = x0
        if num_var == '6':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            var4 = var[3]
            var5 = var[4]
            var6 = var[5]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6')):
                var1, var2, var3, var4, var5, var6 = x0
        J = '['+J+']'
        J = eval(J)
        J = np.array(J)        
        J = J
        return J

    def b1(self, F, x0, number_of_variables):
        """
        calculates the approximation of vector 'bi' on the method 'Newton - Rapshon'
        """
        var = ['var1', 'var2', 'var3', 'var4', 'var5', 'var6', 'var7', 'var8', 'var9']
        num_var = number_of_variables

        if num_var == '1':
            var1 = var[0]
            if var1 == 'var1':
                var1 = x0            
        if num_var == '2':
            var1 = var[0]
            var2 = var[1]
            if ((var1 == 'var1') and (var2 == 'var2')):
                var1, var2 = x0
        if num_var == '3':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3')):
                var1, var2, var3 = x0
        if num_var == '4':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            var4 = var[3]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4')):
                var1, var2, var3, var4 = x0
        if num_var == '5':
            var1 = var[0]
            var2 = var[1]
            var3 = var[2]
            var4 = var[3]
            var5 = var[4]
            if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5')):
                var1, var2, var3, var4, var5 = x0
        F = eval(F)
        F = np.array(F)
        F = F
        return F

    def reset(self):
        self.matrixJ.delete(0, 'end')
        self.matrixF.delete(0, 'end')
        self.number_var.delete(0, 'end')
        self.x0.delete(0, 'end')
        self.results_variable1.configure(text = '')
        self.results_variable2.configure(text = '')
        self.results_variable3.configure(text = '')
        self.results_variable4.configure(text = '')
        self.results_variable5.configure(text = '')
        self.results_variable6.configure(text = '')
        self.error_variable1.configure(text = '')
        self.error_variable2.configure(text = '')
        self.error_variable3.configure(text = '')
        self.error_variable4.configure(text = '')
        self.error_variable5.configure(text = '')
        self.error_variable6.configure(text = '')
        self.iterations.configure(text = '')

class mixed_root(Frame):
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Inverse Kinematics (mixed)', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = 'Equations:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.equations = Entry(frame1, font = controller.Arial16)
        self.equations.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        Label(frame1, text = 'Initials values', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.x0 = Entry(frame1, font = controller.Arial16)
        self.x0.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = '', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        btn_go = Button(frame1, text = 'GO', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.GO(self.x0.get(), self.equations.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(frame1, text = 'Back', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('inverse_kinematics'))
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'RESULTS', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'Approximate values:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.results_variable1 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable1.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable2 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable2.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable3 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable3.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable4 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable4.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable5 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable5.pack(side = TOP, padx = 5, pady = 2)
        self.results_variable6 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.results_variable6.pack(side = TOP, padx = 5, pady = 2)
        Label(self, text = 'Error range by variable:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.error_variable1 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable1.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable2 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable2.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable3 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable3.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable4 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable4.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable5 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable5.pack(side = TOP, padx = 5, pady = 2)
        self.error_variable6 = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.error_variable6.pack(side = TOP, padx = 5, pady = 2)
        Label(self, text = 'Number of iterations:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.iterations = Label(self, text = '', font = controller.Arial14, fg = 'red')
        self.iterations.pack(side = TOP, padx = 5, pady = 2)
    
    def GO(self, x0, equations):
        if equations == '':
            messagebox.showwarning('Warning', 'Equations is empty')
        if x0 == '':
            messagebox.showwarning('Warning', 'Initials values is empty')
        if ((equations != '') and (x0 != '')):
            x0 = '['+'['+x0+']'+']'
            x0 = eval(x0)
            x0 = np.array(x0)
            try:
                nrowx0, ncolumnx0 = x0.shape
                if nrowx0 != 1:
                    messagebox.showerror('Error', 'Check your Initials values')
                if nrowx0 == 1:
                    results = root(self.equ, x0)
                    if ncolumnx0 == 1:
                        results1 = results.x[0]
                        error1 = results.fun[0]
                        iterations = results.nfev
                        results1 = float(results1)
                        results1 = round(results1, 5)
                        error1 = float(error1)
                        error1 = round(error1, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.iterations.configure(text = iterations)
                    if ncolumnx0 == 2:
                        results1 = results.x[0]
                        results2 = results.x[1]
                        error1 = results.fun[0]
                        error2 = results.fun[1]
                        iterations = results.nfev
                        results1 = float(results1)
                        results2 = float(results2)
                        error1 = float(error1)
                        error2 = float(error2)
                        results1 = round(results1, 5)
                        results2 = round(results2, 5)
                        error1 = round(error1, 5)
                        error2 = round(error2, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.results_variable2.configure(text = 'Variable 2 = '+str(results2))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.error_variable2.configure(text = 'Variable 2 = '+str(error2))
                        self.iterations.configure(text = iterations)
                    if ncolumnx0 == 3:
                        results1 = results.x[0]
                        results2 = results.x[1]
                        results3 = results.x[2]
                        error1 = results.fun[0]
                        error2 = results.fun[1]
                        error3 = results.fun[2]
                        iterations = results.nfev
                        results1 = float(results1)
                        results2 = float(results2)
                        results3 = float(results3)
                        error1 = float(error1)
                        error2 = float(error2)
                        error3 = float(error3)
                        results1 = round(results1, 5)
                        results2 = round(results2, 5)
                        results3 = round(results3, 5)
                        error1 = round(error1, 5)
                        error2 = round(error2, 5)
                        error3 = round(error3, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.results_variable2.configure(text = 'Variable 2 = '+str(results2))
                        self.results_variable3.configure(text = 'Variable 3 = '+str(results3))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.error_variable2.configure(text = 'Variable 2 = '+str(error2))
                        self.error_variable3.configure(text = 'Variable 3 = '+str(error3))
                        self.iterations.configure(text = iterations)
                    if ncolumnx0 == 4:
                        results1 = results.x[0]
                        results2 = results.x[1]
                        results3 = results.x[2]
                        results4 = results.x[3]
                        error1 = results.fun[0]
                        error2 = results.fun[1]
                        error3 = results.fun[2]
                        error4 = results.fun[3]
                        iterations = results.nfev
                        results1 = float(results1)
                        results2 = float(results2)
                        results3 = float(results3)
                        results4 = float(results4)
                        error1 = float(error1)
                        error2 = float(error2)
                        error3 = float(error3)
                        error4 = float(error4)
                        results1 = round(results1, 5)
                        results2 = round(results2, 5)
                        results3 = round(results3, 5)
                        results4 = round(results4, 5)
                        error1 = round(error1, 5)
                        error2 = round(error2, 5)
                        error3 = round(error3, 5)
                        error4 = round(error4, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.results_variable2.configure(text = 'Variable 2 = '+str(results2))
                        self.results_variable3.configure(text = 'Variable 3 = '+str(results3))
                        self.results_variable4.configure(text = 'Variable 4 = '+str(results4))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.error_variable2.configure(text = 'Variable 2 = '+str(error2))
                        self.error_variable3.configure(text = 'Variable 3 = '+str(error3))
                        self.error_variable4.configure(text = 'Variable 4 = '+str(error4))
                        self.iterations.configure(text = iterations)
                    if ncolumnx0 == 5:
                        results1 = results.x[0]
                        results2 = results.x[1]
                        results3 = results.x[2]
                        results4 = results.x[3]
                        results5 = results.x[4]
                        error1 = results.fun[0]
                        error2 = results.fun[1]
                        error3 = results.fun[2]
                        error4 = results.fun[3]
                        error5 = results.fun[4]
                        iterations = results.nfev
                        results1 = float(results1)
                        results2 = float(results2)
                        results3 = float(results3)
                        results4 = float(results4)
                        results5 = float(results5)
                        error1 = float(error1)
                        error2 = float(error2)
                        error3 = float(error3)
                        error4 = float(error4)
                        error5 = float(error5)
                        results1 = round(results1, 5)
                        results2 = round(results2, 5)
                        results3 = round(results3, 5)
                        results4 = round(results4, 5)
                        results5 = round(results5, 5)
                        error1 = round(error1, 5)
                        error2 = round(error2, 5)
                        error3 = round(error3, 5)
                        error4 = round(error4, 5)
                        error5 = round(error5, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.results_variable2.configure(text = 'Variable 2 = '+str(results2))
                        self.results_variable3.configure(text = 'Variable 3 = '+str(results3))
                        self.results_variable4.configure(text = 'Variable 4 = '+str(results4))
                        self.results_variable5.configure(text = 'Variable 5 = '+str(results5))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.error_variable2.configure(text = 'Variable 2 = '+str(error2))
                        self.error_variable3.configure(text = 'Variable 3 = '+str(error3))
                        self.error_variable4.configure(text = 'Variable 4 = '+str(error4))
                        self.error_variable5.configure(text = 'Variable 5 = '+str(error5))
                        self.iterations.configure(text = iterations)
                    if ncolumnx0 == 6:
                        results1 = results.x[0]
                        results2 = results.x[1]
                        results3 = results.x[2]
                        results4 = results.x[3]
                        results5 = results.x[4]
                        results6 = results.x[5]
                        error1 = results.fun[0]
                        error2 = results.fun[1]
                        error3 = results.fun[2]
                        error4 = results.fun[3]
                        error5 = results.fun[4]
                        error6 = results.fun[5]
                        iterations = results.nfev
                        results1 = float(results1)
                        results2 = float(results2)
                        results3 = float(results3)
                        results4 = float(results4)
                        results5 = float(results5)
                        results6 = float(results6)
                        error1 = float(error1)
                        error2 = float(error2)
                        error3 = float(error3)
                        error4 = float(error4)
                        error5 = float(error5)
                        error6 = float(error6)
                        results1 = round(results1, 5)
                        results2 = round(results2, 5)
                        results3 = round(results3, 5)
                        results4 = round(results4, 5)
                        results5 = round(results5, 5)
                        results6 = round(results6, 5)
                        error1 = round(error1, 5)
                        error2 = round(error2, 5)
                        error3 = round(error3, 5)
                        error4 = round(error4, 5)
                        error5 = round(error5, 5)
                        error6 = round(error6, 5)
                        self.results_variable1.configure(text = 'Variable 1 = '+str(results1))
                        self.results_variable2.configure(text = 'Variable 2 = '+str(results2))
                        self.results_variable3.configure(text = 'Variable 3 = '+str(results3))
                        self.results_variable4.configure(text = 'Variable 4 = '+str(results4))
                        self.results_variable5.configure(text = 'Variable 5 = '+str(results5))
                        self.results_variable6.configure(text = 'Variable 6 = '+str(results6))
                        self.error_variable1.configure(text = 'Variable 1 = '+str(error1))
                        self.error_variable2.configure(text = 'Variable 2 = '+str(error2))
                        self.error_variable3.configure(text = 'Variable 3 = '+str(error3))
                        self.error_variable4.configure(text = 'Variable 4 = '+str(error4))
                        self.error_variable5.configure(text = 'Variable 5 = '+str(error5))
                        self.error_variable6.configure(text = 'Variable 6 = '+str(error6))
                        self.iterations.configure(text = iterations)
            except:
                messagebox.showerror('Error', 'It caused an error, check your data')
    def equ(self, x):
        equations = eval(self.equations.get())
        return np.array(equations)

    def reset(self):
        self.equations.delete(0, 'end')
        self.x0.delete(0, 'end')
        self.results_variable1.configure(text = '')
        self.results_variable2.configure(text = '')
        self.results_variable3.configure(text = '')
        self.results_variable4.configure(text = '')
        self.results_variable5.configure(text = '')
        self.results_variable6.configure(text = '')
        self.error_variable1.configure(text = '')
        self.error_variable2.configure(text = '')
        self.error_variable3.configure(text = '')
        self.error_variable4.configure(text = '')
        self.error_variable5.configure(text = '')
        self.error_variable6.configure(text = '')

class differential_kinematics(Frame):
    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller
        Label(self, text = 'Differential Kinematics', font = controller.title_font).pack(side = TOP, padx = 5, pady = 10)
        frame1 = Frame(self, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")
        Label(frame1, text = 'DH1:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh1 = Entry(frame1, font = controller.Arial16)
        self.dh1.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH2:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh2 = Entry(frame1, font = controller.Arial16)
        self.dh2.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH3:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh3 = Entry(frame1, font = controller.Arial16)
        self.dh3.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH4:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh4 = Entry(frame1, font = controller.Arial16)
        self.dh4.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH5:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh5 = Entry(frame1, font = controller.Arial16)
        self.dh5.pack(side = TOP, padx = 5, pady = 2)
        Label(frame1, text = 'DH6:', font = controller.Arial16).pack(side = TOP, padx = 5, pady = 2)
        self.dh6 = Entry(frame1, font = controller.Arial16)
        self.dh6.pack(side = TOP, padx = 5, pady = 2)
        btn_go = Button(frame1, text = 'GO', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.GO(self.dh1.get(), self.dh2.get(), self.dh3.get(), self.dh4.get(), self.dh5.get(), self.dh6.get()))
        btn_go.pack(side = TOP, padx = 5, pady = 10)
        btn_reset = Button(frame1, text = 'Reset', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: self.reset())
        btn_reset.pack(side = TOP, padx = 5, pady = 10)
        btn_back = Button(frame1, text = 'Back', font = controller.Arial16, width = 15, height = 2, borderwidth = 5, cursor = 'hand1', command = lambda: controller.show_frame('main'))
        btn_back.pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'RESULTS', font = controller.Arial20).pack(side = TOP, padx = 5, pady = 10)
        Label(self, text = 'Jacobian Matrix', font = controller.Arial20).pack(side = TOP, padx = 5, pady = 10)
        self.result_matrix = Label(self, text = '', font = controller.Arial20, fg = 'red')
        self.result_matrix.pack(side = TOP, padx = 5, pady = 10)

    def GO(self, DH1, DH2, DH3, DH4, DH5, DH6):
        try:
            DH = []
            for dh in [DH1, DH2, DH3, DH4, DH5,]:
                if dh != '':
                    DH.append(eval(dh))                

            J = jacobian(*DH)
            J = str(J)
            J = J[1:-1]
            self.result_matrix.configure(text = ' '+J)
        except:
            messagebox.showerror('Error', 'Check your entered data')
            messagebox.showinfo('Information', 'Tip: Probably your mistake is that you are missing or you have too many values in your parameters')

    def reset(self):
        self.dh1.delete(0, 'end')
        self.dh2.delete(0, 'end')
        self.dh3.delete(0, 'end')
        self.dh4.delete(0, 'end')
        self.dh5.delete(0, 'end')
        self.dh6.delete(0, 'end')
        self.result_matrix.configure(text = '')

if __name__ == "__main__":
    app = GUI()
    app.mainloop()