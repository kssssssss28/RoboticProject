import tkinter as tk
import subprocess
import os
from Sensing_cv.predict import run
NUM = 15



run(NUM)

root = tk.Tk()
print(os.getcwd())
# 创建Button1和Button2
button1 = tk.Button(root, text="Step1 - Push Red Garbages", command=lambda: subprocess.call(["python", "run_two_arm_env_red.py"]))
button2 = tk.Button(root, text="Step1 - Push Blue Garbages", command=lambda: subprocess.call(["python", "run_two_arm_env_blue.py"]))
button3 = tk.Button(root, text="Step2 - Grab Garbages", command=lambda: subprocess.call(["python", "single_testing_ppo.py"]))



button1.pack()
button2.pack()
button3.pack()


root.mainloop()
