import  tkinter as tk 
import cv2 
from PIL import Image, ImageTk 

width, height = 800, 600 
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height) 

root = tk.Tk() 
root.bind('<Escape>', lambda e: root.quit()) 
lmain = tk.Label(root) 
lmain.pack() 

def show_frame(): 
    _, frame = cap.read() 
    frame = cv2.flip(frame, 1) 
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA) 
    img = Image.fromarray(cv2image) 
    imgtk = ImageTk.PhotoImage(image=img) 
    lmain.imgtk = imgtk 
    lmain.configure(image=imgtk) 
    lmain.after(10, show_frame) 

show_frame() 
root.mainloop() 

#[0,-0.8660254,0.5],[0.5,-0.4330127,-0.75],[0.8660254,0.25,0.4330127]