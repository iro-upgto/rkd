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
        self.main.title("Fundamentals of Robotics")
        self.main.geometry("650x450+400+60")
        self.main.minsize(600, 480)
        self.main.maxsize(650, 500)
        frame1 = Frame(self.main, width = 650, height = 500)
        frame1.pack()
        Arial16 = font.Font(family = "Arial", size = 16, weight = "bold")
        btn_transformations = Button(frame1, text = "Transformations", font = Arial16, width = 40, height = 3, borderwidth = 5, command = self.m_transformations)
        btn_transformations.grid(row = 0, column = 1, padx = 10, pady = 20)
        btn_kinematics = Button(frame1, text = "Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, command = self.m_kinematics)
        btn_kinematics.grid(row = 1, column = 1, padx = 10, pady = 10)
        btn_rkinematics = Button(frame1, text = "Inverse Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, command = self.m_inverse_kinematics)
        btn_rkinematics.grid(row = 2, column = 1, padx = 10, pady = 10)
        btn_dkinematics = Button(frame1, text = "Differential Kinematics", font = Arial16, width = 40, height = 3, borderwidth = 5, command = self.m_differential_kinematics)
        btn_dkinematics.grid(row = 3, column = 1, padx = 10, pady = 10)
        barMenu = Menu(self.main)
        menuInfo = Menu(barMenu)
        menuInfo.add_command(label = "Open Message",command = self.info)
        barMenu.add_cascade(label = "Information", menu = menuInfo)
        self.main.config(menu = barMenu)

        self.main.mainloop()

    def info(self):
        messagebox.showinfo("Information","Hola")

    #Funtions for the Buttons

    #second level of windows

    def m_transformations(self):
        self.ven_transformations = Toplevel()   
        self.ven_transformations.title("Transformations")
        self.ven_transformations.geometry("800x700+50+10")        
        frame2 = Frame(self.ven_transformations, width = 800, height = 700)
        frame2.pack()        
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_rotations = Button(frame2, text = "Rotations", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.m_rotations)
        btn_rotations.grid(row = 0, column = 1, padx = 10, pady = 20)
        btn_conversions = Button(frame2, text = "Conversions\nfrom matrices\nto angle", font = Arial14, width = 20, height = 5, borderwidth = 5)
        btn_conversions.grid(row = 1, column = 1, padx = 10, pady = 20)
        btn_htmDH = Button(frame2, text = "Matrix DH", font = Arial14, width = 20, height = 3, borderwidth = 5)
        btn_htmDH.grid(row = 2, column = 1, padx = 10, pady = 10)        
        btn_exit = Button(frame2, text = "Exit", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.ven_transformations.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_transformations)

    def m_kinematics(self):
        self.ven_kinematics = Toplevel()
        self.ven_kinematics.title("Kinematics")
        self.ven_kinematics.geometry("800x700+50+10")
        frame3 = Frame(self.ven_kinematics, width = 800, height = 700)
        frame3.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_exit = Button(frame3, text = "Exit", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.ven_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_kinematics)

    def m_inverse_kinematics(self):
        self.ven_inv_kinematics = Toplevel()
        self.ven_inv_kinematics.title("Inverse Kinematics")
        self.ven_inv_kinematics.geometry("800x700+50+10")
        frame4 = Frame(self.ven_inv_kinematics, width = 800, height = 700)
        frame4.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_exit = Button(frame4, text = "Exit", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.ven_inv_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_inv_kinematics)

    def m_differential_kinematics(self):
        self.ven_dif_kinematics = Toplevel()
        self.ven_dif_kinematics.title("Differential Kinematics")
        self.ven_dif_kinematics.geometry("800x700+50+10")
        frame5 = Frame(self.ven_dif_kinematics, width = 800, height = 700)
        frame5.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_exit = Button(frame5, text = "Exit", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.ven_dif_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_dif_kinematics)
    
    #Third level of windows
    def m_rotations(self):
        self.ven_rotations = Toplevel()
        self.ven_rotations.title("Rotations")
        self.ven_rotations.geometry("800x700+50+10")
        frame6 = Frame(self.ven_rotations, width = 800, height = 700)
        frame6.pack()
        Arial14 = font.Font(family = "Arial", size = 14, weight = "bold")
        btn_exit = Button(frame6, text = "Exit", font = Arial14, width = 20, height = 3, borderwidth = 5, command = self.ven_rotations.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_rotations)
            

def mainp():
    my_GUI = GUI()
    return (0)

if __name__ == '__main__':
    mainp()