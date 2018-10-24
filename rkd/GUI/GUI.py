from tkinter import *
from tkinter import font
from tkinter import messagebox

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
        self.minsize(1150, 600)
        self.maxsize(1200, 650)

        barMenu = Menu(self)
        menuInfo = Menu(barMenu)
        menuhelp = Menu(barMenu)        
        menuInfo.add_command(label = "Open Message", command = self.info)
        menuhelp.add_command(label = "Open Message")# command = self.help1)        
        barMenu.add_cascade(label = "Information", menu = menuInfo)
        barMenu.add_cascade(label = "Help", menu = menuhelp)        
        self.config(menu = barMenu)


        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (main, transformations, rotations):
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

class main(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller        
        label = Label(self, text="Robot Kinematics", font=controller.title_font)
        label.pack(side="top", fill="x", pady=10)
        btn_transformations = Button(self, text = "Transformations", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations"))
        btn_transformations.pack(padx = 10, pady = 20)
        btn_kinematics = Button(self, text = "Forward Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1")# command = self.m_fwd_kinematics)
        btn_kinematics.pack(padx = 10, pady = 10)
        btn_rkinematics = Button(self, text = "Inverse Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1")# command = self.m_inverse_kinematics)
        btn_rkinematics.pack(padx = 10, pady = 10)
        btn_dkinematics = Button(self, text = "Differential Kinematics", font = controller.Arial16, width = 40, height = 3, borderwidth = 5, cursor = "hand1")# command = self.m_differential_kinematics)
        btn_dkinematics.pack(padx = 10, pady = 10)      


class transformations(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller

        label = Label(self, text="Transformations", font=controller.title_font)
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


class rotations(Frame):

    def __init__(self, parent, controller):
        Frame.__init__(self, parent)
        self.controller = controller

        txt1 = StringVar()
        txt2 = StringVar()
        Label(self, text = "Axis:", font = controller.Arial14).pack(side = "left", anchor = "n", padx = 10, pady = 20)        
        txt_axis = Entry(self, font = controller.Arial14)
        txt_axis.pack(side = "left", anchor = "n",padx = 10, pady = 20)
        Label(self, text = "Angle:", font = controller.Arial14).pack(side = "left", anchor = "n", padx = 10, pady = 20)
        txt_angle = Entry(self, font = controller.Arial14)
        txt_angle.pack(side = "left", anchor = "n", padx = 10, pady = 20)
        btn_go = Button(self, text = "GO", font = controller.Arial14, width = 15, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: self.go_pulse(txt_axis.get(), txt_angle.get()))
        btn_go.pack(side = "left", anchor = "n", padx = 100, pady = 10)
        btn_back = Button(self, text = "Back", font = controller.Arial14, width = 15, height = 3, borderwidth = 5, cursor = "hand1", command = lambda: controller.show_frame("transformations")) 
        btn_back.pack(side = "left", anchor = "n", padx = 10, pady = 10)

    #Functions for buttons
    
    def go_pulse(self, axis, angle):        
        if ((axis == "") or (angle == "")):
            messagebox.showwarning("Warning", "Check the TextBox")

        if ((axis != "") or (angle != "")):
            answer = messagebox.askquestion("Important to answer", "Are you entering the angles in degrees?")


if __name__ == "__main__":
    app = GUI()
    app.mainloop()