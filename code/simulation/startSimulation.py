import tkinter as tk
import subprocess
from Sensing_cv.predict import run
NUM = 30



run(NUM)

root = tk.Tk()

# 创建Button1和Button2
button1 = tk.Button(root, text="Step1 - Push Garbages", command=lambda: subprocess.call(["python", "script1.py"]))
button2 = tk.Button(root, text="Step2 - Grab Garbages", command=lambda: subprocess.call(["python", "single_testing_ppo.py"]))



button1.pack()
button2.pack()


root.mainloop()
