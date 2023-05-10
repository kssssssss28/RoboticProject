import tkinter as tk
import subprocess

# 创建主窗口
root = tk.Tk()
root.title("Garbage sorting type")
root.geometry("600x400")
image = tk.PhotoImage(file="bg.jpeg")

# 将图片添加到标签
label = tk.Label(root, image=image)
label.pack()
# 定义按钮1的点击事件
def on_button1_click():
    subprocess.run(["python", "single_testing_ppo.py"])

# 定义按钮2的点击事件
def on_button2_click():
    subprocess.run(["python", "script2.py"])

# 创建按钮1
button1 = tk.Button(root, text="Grab the garbage", command=on_button1_click)
button1.pack(pady=50, padx=150, anchor=tk.CENTER)

# 创建按钮2
button2 = tk.Button(root, text="Push the garbage", command=on_button2_click)
button2.pack(pady=50, padx=150, anchor=tk.CENTER)

# 进入主循环
root.mainloop()
