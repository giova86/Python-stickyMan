# libraries
import cv2
import time
import mediapipe as mp
import numpy as np
import os
from utils import mediapipe_detection, draw_landmarks, draw_landmarks_custom, draw_limit_rh, draw_limit_lh, check_detection

import pyvirtualcam
from pyvirtualcam import PixelFormat

import tkinter as tk
from tkinter import messagebox, PhotoImage
from ttkthemes import ThemedTk

from PIL import Image, ImageTk

import objc
from AVFoundation import AVCaptureDevice

import customtkinter
customtkinter.set_appearance_mode("System")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

def get_webcam_names():
    webcam_list = []
    devices = AVCaptureDevice.devicesWithMediaType_("vide")

    for device in devices:
        name = device.localizedName()
        webcam_list.append(name)

    return webcam_list


mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils



def select_webcam_and_open():
    def open_webcam(webcam_id):
        width = 1920
        height = 1080
        cap = cv2.VideoCapture(webcam_id)
        with mp_holistic.Holistic(min_detection_confidence=0.5,
                                  min_tracking_confidence=0.5) as holistic:
            with pyvirtualcam.Camera(width, height, 30, fmt=PixelFormat.BGR) as cam:
                print()
                print('Virtual camera device: ' + cam.device)
                print()
                while cap.isOpened():
                    ret, frame = cap.read()

                    # make detection
                    image, results = mediapipe_detection(frame, holistic)

                    # draw_limit_rh(frame, results)
                    # draw_limit_lh(frame, results)

                    img = np.zeros([frame.shape[0],frame.shape[1],3],dtype=np.uint8)
                    draw_landmarks_custom(img, results)

                    # check_detection(frame, results)

                    cv2.imshow('Sticky Man', img)
                    cam.send(img)
                    cam.sleep_until_next_frame()

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                cap.release()
                cv2.destroyAllWindows()

    devices = get_webcam_names()

    popup = customtkinter.CTk()
    popup.title("Webcam Selection")
    popup.resizable(False, False)

    total_webcams = cv2.getBuildInformation().count("Video I/O")
    empty_space = (popup.winfo_height() - (len(devices))) / 10

    # Aggiungiamo un'immagine sopra i bottoni
    try:
        img = PhotoImage(file='/Users/gbocchi/GitHub/dev_pers/Python-stickyMan/title.png')
        image_label = tk.Label(popup, image=img)
        image_label.pack(pady=(30,20),padx=70)
    except tk.TclError:
        messagebox.showerror("Image Error", "Failed to load the image.")

    for webcam_id in range(len(devices)):
        button = customtkinter.CTkButton(popup, text=f"{devices[webcam_id]}", command=lambda id=webcam_id: open_webcam(id))#, height=2, width=35)
        # button.pack(pady=5)
        button.pack(side=tk.TOP, padx=30,pady=(0,20))

    if total_webcams == 0:
        messagebox.showwarning("No Webcams", "No webcams found on this system.")

    popup.update_idletasks()
    width = popup.winfo_width()
    height = popup.winfo_height()
    x = (popup.winfo_screenwidth() // 2) - (width // 2)
    y = (popup.winfo_screenheight() // 2) - (height // 2)
    popup.geometry('{}x{}+{}+{}'.format(width, height, x, y))

    popup.mainloop()

if __name__ == "__main__":
    select_webcam_and_open()
