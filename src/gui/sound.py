#!/usr/bin/env python
# -- coding: utf-8 --

import customtkinter as ctk
import pygame 

pygame.mixer.init()
click_sound = pygame.mixer.Sound("/home/seojin/catkin_ws/src/soomac/src/gui/click_sound.mp3")

def play_click_sound():
    click_sound.play()

def on_button_click():
    play_click_sound()

if __name__ == '__main__':
    root = ctk.CTk()

    button = ctk.CTkButton(root, text="Click Me", command=on_button_click)
    button.pack(pady=20)

    root.mainloop()
