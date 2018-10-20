#GUI of the library 'rkd'

from tkinter import *
from tkinter import ttk
from tkinter import font

main = Tk()
main.title("Fundamentals of Robotics")
main.geometry("650x400")
main.minsize(600, 350)
main.maxsize(650, 400)
frame1 = Frame(main, width=900, height=450)
frame1.pack()

#style = ttk.Style()
#style.map("C.TButton",
#		foreground=[('pressed','red'),('active','blue')],
#		background=[('pressed','!disabled','black'),('active','white')]
#		)
Arial12 = font.Font(family = "Arial",size = 12,weight = "bold")
btn_transformations = Button(frame1, text = "Transformations", font = Arial12, width = 50, height = 3)
btn_transformations.grid(row = 0, column = 1, padx = 10, pady = 20)
btn_kinematics = Button(frame1, text = "Kinematics", font = Arial12, width = 50, height = 3)
btn_kinematics.grid(row = 1, column = 1, padx = 10, pady = 10)
btn_rkinematics = Button(frame1, text = "Inverse Kinematics", font = Arial12, width = 50, height = 3)
btn_rkinematics.grid(row = 2, column = 1, padx = 10, pady = 10)
btn_dkinematics = Button(frame1, text = "Differential Kinematics", font = Arial12, width = 50, height = 3)
btn_dkinematics.grid(row = 3, column = 1, padx = 10, pady = 10)
main.mainloop()