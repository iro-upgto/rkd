#GUI of the library 'rkd'

from tkinter import *
from tkinter import ttk
from tkinter import font
from tkinter import messagebox
from rkd.transformations import *
from rkd.kinematics import *

class GUI():
    """
    """

    def __init__(self):
        self.main = Tk()
        self.main.title("Robot Kinematics")
        self.main.geometry("1200x480+50+100")
        self.main.minsize(1100, 480)
        self.main.maxsize(1200, 500)
        img = PhotoImage(file="img/robot.png") #icon of the main
        self.main.call("wm","iconphoto", self.main._w, img) #icon of the main        
        frame1 = Frame(self.main, width = 650, height = 500)
        frame1.pack(side = "left", anchor = "n")                
        Arial16 = font.Font(family = "Arial", size = 16, weight = "bold")
        logo = PhotoImage(file = "img/upg(1).png")
        Label(self.main, image = logo, height = 150).place(x = 900, y = 20)
        btn_transformations = Button(frame1, text = "Transformations", font = Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = self.m_transformations)
        btn_transformations.grid(row = 0, column = 1, padx = 10, pady = 20)
        btn_kinematics = Button(frame1, text = "Forward Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = self.m_fwd_kinematics)
        btn_kinematics.grid(row = 1, column = 1, padx = 10, pady = 10)
        btn_rkinematics = Button(frame1, text = "Inverse Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = self.m_inverse_kinematics)
        btn_rkinematics.grid(row = 2, column = 1, padx = 10, pady = 10)
        btn_dkinematics = Button(frame1, text = "Differential Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = self.m_differential_kinematics)
        btn_dkinematics.grid(row = 3, column = 1, padx = 10, pady = 10)
        barMenu1 = Menu(self.main)
        menuInfo = Menu(barMenu1)
        menuhelp1 = Menu(barMenu1)
        menuInfo.add_command(label = "Open Message", command = self.info)
        menuhelp1.add_command(label = "Open Message", command = self.help1)
        barMenu1.add_cascade(label = "Information", menu = menuInfo)
        barMenu1.add_cascade(label = "Help", menu = menuhelp1)
        self.main.config(menu = barMenu1)
        self.main.mainloop()

    #Functions for the BarMenu

    def info(self):
        messagebox.showinfo("Information","Version: 1.0\n\nMIT License\n\nCopyright (c) 2018 IRO\n\nPermission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the Software), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\n\nThe above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.\n\nTHE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.")

    def help1(self):
        messagebox.showinfo("Help", "Hola")


#second level of windows

    def m_transformations(self):
        self.ven_transformations = Toplevel()   
        self.ven_transformations.title("Transformations")
        self.ven_transformations.geometry("800x650+300+10")
        img = PhotoImage(file="img/robot.png") #icon of the main
        self.main.call("wm","iconphoto", self.ven_transformations._w, img) #icon of the main
        frame2 = Frame(self.ven_transformations, width = 800, height = 650)
        frame2.pack()        
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_rotations = Button(frame2, text = "Rotations", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = self.m_rotations)
        btn_rotations.grid(row = 0, column = 1, padx = 10, pady = 20)
        btn_parameterization = Button(frame2, text = "Parameterization\nof\nrotations", font = Arial14, width = 20, height = 5, borderwidth = 5, cursor = "hand1")
        btn_parameterization.grid(row = 1, column = 1, padx = 10, pady = 20)
        btn_axis_angle = Button(frame2, text = "Axis / Angle", font = Arial14, width = 20, height = 5, borderwidth = 5, cursor = "hand1")
        btn_axis_angle.grid(row = 2, column = 1, padx = 10, pady = 20)
        btn_htmDH = Button(frame2, text = "Matrix DH", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1")
        btn_htmDH.grid(row = 3, column = 1, padx = 10, pady = 10)        
        btn_back = Button(frame2, text = "Back", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = self.ven_transformations.destroy) 
        btn_back.grid(row = 4, column = 1, padx = 10, pady = 10)
        barMenu2 = Menu(self.ven_transformations)        
        menuhelp2 = Menu(barMenu2)        
        menuhelp2.add_command(label = "Open Message", command = self.help2)        
        barMenu2.add_cascade(label = "Help", menu = menuhelp2)
        self.ven_transformations.config(menu = barMenu2)
        self.main.wait_window(self.ven_transformations)

    #Functions for the BarMenu

    def help2(self):
        messagebox.showinfo("Help", "Hola")

    def m_fwd_kinematics(self):
        self.ven_kinematics = Toplevel()
        self.ven_kinematics.title("Kinematics")
        self.ven_kinematics.geometry("800x650+300+10")
        img = PhotoImage(file="img/robot.png") #icon of the main
        self.main.call("wm","iconphoto", self.ven_kinematics._w, img) #icon of the main
        frame3 = Frame(self.ven_kinematics, width = 800, height = 650)
        frame3.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_back = Button(frame3, text = "Back", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = self.ven_kinematics.destroy) 
        btn_back.grid(row = 3, column = 3, padx = 10, pady = 10)
        barMenu3 = Menu(self.ven_kinematics)
        menuhelp3 = Menu(barMenu3)
        menuhelp3.add_command(label = "Open Message", command = self.help3)
        barMenu3.add_cascade(label = "Help", menu = menuhelp3)
        self.ven_kinematics.config(menu = barMenu3)
        self.main.wait_window(self.ven_kinematics)

    #Funtions for the BarMenu

    def help3(self):
        messagebox.showinfo("Help", "Hola")

    def m_inverse_kinematics(self):
        self.ven_inv_kinematics = Toplevel()
        self.ven_inv_kinematics.title("Inverse Kinematics")
        self.ven_inv_kinematics.geometry("800x650+300+10")
        img = PhotoImage(file="img/robot.png") #icon of the main
        self.main.call("wm","iconphoto", self.ven_inv_kinematics._w, img) #icon of the main
        frame4 = Frame(self.ven_inv_kinematics, width = 800, height = 650)
        frame4.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_back = Button(frame4, text = "Back", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = self.ven_inv_kinematics.destroy) 
        btn_back.grid(row = 3, column = 3, padx = 10, pady = 10)
        barMenu4 = Menu(self.ven_inv_kinematics)
        menuhelp4 = Menu(barMenu4)
        menuhelp4.add_command(label = "Open Message", command = self.help4)
        barMenu4.add_cascade(label = "Help", menu = menuhelp4)
        self.ven_inv_kinematics.config(menu = barMenu4)
        self.main.wait_window(self.ven_inv_kinematics)

    #Funtions for the BarMenu

    def help4(self):
        messagebox.showinfo("Help", "Hola")

    def m_differential_kinematics(self):
        self.ven_dif_kinematics = Toplevel()
        self.ven_dif_kinematics.title("Differential Kinematics")
        self.ven_dif_kinematics.geometry("800x650+300+10")
        img = PhotoImage(file="img/robot.png") #icon of the main
        self.main.call("wm","iconphoto", self.ven_dif_kinematics._w, img) #icon of the main
        frame5 = Frame(self.ven_dif_kinematics, width = 800, height = 650)
        frame5.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_back = Button(frame5, text = "Back", font = Arial14, width = 20, height = 3, borderwidth = 5, cursor = "hand1", command = self.ven_dif_kinematics.destroy) 
        btn_back.grid(row = 3, column = 3, padx = 10, pady = 10)
        barMenu5 = Menu(self.ven_dif_kinematics)
        menuhelp5 = Menu(barMenu5)
        menuhelp5.add_command(label = "Open Message", command = self.help5)
        barMenu5.add_cascade(label = "Help", menu = menuhelp5)
        self.ven_dif_kinematics.config(menu = barMenu5)
        self.main.wait_window(self.ven_dif_kinematics)

    #Functions for the BarMenu

    def help5(self):
        messagebox.showinfo("Help", "Hola")
    
#Third level of windows
    def m_rotations(self):
        self.ven_rotations = Toplevel()
        self.ven_rotations.title("Rotations")
        self.ven_rotations.geometry("800x750+300+10")
        img = PhotoImage(file="img/rotations.gif") #icon of the main
        self.main.call("wm","iconphoto", self.ven_rotations._w, img) #icon of the main
        frame6 = Frame(self.ven_rotations, width = 800, height = 650)
        frame6.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        txt1 = StringVar()
        txt2 = StringVar()
        Label(frame6, text = "Axis:", font = Arial14).grid(row = 0, column = 0, padx = 10, pady = 20)        
        txt_axis = Entry(frame6, font = Arial14)
        txt_axis.grid(row = 0, column = 1, padx = 10, pady = 20)
        Label(frame6, text = "Angle:", font = Arial14).grid(row = 0, column = 2, padx = 10, pady = 20)
        txt_angle = Entry(frame6, font = Arial14)
        txt_angle.grid(row = 0, column = 3, padx = 10, pady = 20)        
        btn_go = Button(frame6, text = "GO", font = Arial14, width = 15, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: self.go_pulse(txt_axis.get(), txt_angle.get()))
        btn_go.grid(row = 1, column = 1, padx = 10, pady = 10)
        btn_back = Button(frame6, text = "Back", font = Arial14, width = 15, height = 3, borderwidth = 5, cursor = "hand1", command = self.ven_rotations.destroy) 
        btn_back.grid(row = 1, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_rotations)

    #Functions for buttons
    
    def go_pulse(self, axis, angle):
        print(axis, "\n",angle)
        #if ((axis == "") and (angle == "")):
        #    messagebox.showwarning("Warning", "Check the TextBox")

        #if ((axis != "") and (angle != "")):
        #    answer = messagebox.askquestion("Important to answer", "Are you entering the angles in degrees?")


def mainp():
    my_GUI = GUI()
    return (0)

if __name__ == '__main__':
    mainp()