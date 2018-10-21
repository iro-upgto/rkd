#GUI of the library 'rkd'

from tkinter import *
from tkinter import ttk
from tkinter import font

class GUI():
    """
    """

    def __init__(self):
        self.main = Tk()
        self.main.title("Fundamentals of Robotics")
        self.main.geometry("650x450+400+50")
        self.main.minsize(600, 430)
        self.main.maxsize(650, 450)
        frame1 = Frame(self.main, width = 650, height = 450)
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

        self.main.mainloop()



    def m_transformations(self):
        self.ven_transformations = Toplevel()   
        self.ven_transformations.title("Transformations")
        self.ven_transformations.geometry("800x700+50+10")
        frame2 = Frame(self.ven_transformations, width = 800, height = 700)
        frame2.pack()
        Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")
        btn_exit = Button(frame2, text = "Exit", font = Arial12, width = 20, height = 3, borderwidth = 5, command = self.ven_transformations.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_transformations)

    def m_kinematics(self):
        self.ven_kinematics = Toplevel()
        self.ven_kinematics.title("Kinematics")
        self.ven_kinematics.geometry("800x700+50+10")
        frame3 = Frame(self.ven_kinematics, width = 800, height = 700)
        frame3.pack()
        Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")
        btn_exit = Button(frame3, text = "Exit", font = Arial12, width = 20, height = 3, borderwidth = 5, command = self.ven_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_kinematics)

    def m_inverse_kinematics(self):
        self.ven_inv_kinematics = Toplevel()
        self.ven_inv_kinematics.title("Inverse Kinematics")
        self.ven_inv_kinematics.geometry("800x700+50+10")
        frame4 = Frame(self.ven_inv_kinematics, width = 800, height = 700)
        frame4.pack()
        Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")
        btn_exit = Button(frame4, text = "Exit", font = Arial12, width = 20, height = 3, borderwidth = 5, command = self.ven_inv_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_inv_kinematics)

    def m_differential_kinematics(self):
        self.ven_dif_kinematics = Toplevel()
        self.ven_dif_kinematics.title("Differential Kinematics")
        self.ven_dif_kinematics.geometry("800x700+50+10")
        frame5 = Frame(self.ven_dif_kinematics, width = 800, height = 700)
        frame5.pack()
        Arial12 = font.Font(family = "Arial", size = 12, weight = "bold")
        btn_exit = Button(frame5, text = "Exit", font = Arial12, width = 20, height = 3, borderwidth = 5, command = self.ven_dif_kinematics.destroy) 
        btn_exit.grid(row = 3, column = 3, padx = 10, pady = 10)
        self.main.wait_window(self.ven_dif_kinematics)

def mainp():
    my_GUI = GUI()
    return (0)

if __name__ == '__main__':
    mainp()