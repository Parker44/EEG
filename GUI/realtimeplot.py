from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *
import numpy as np
import serial

# ------Global variables-------
data = np.array([])
plot_thread = None
cond = False


def plot_data():
    global cond, data
    if cond:
        data_bytes = s.read(2)
        data_int = int.from_bytes(data_bytes, "big") * (5/65535)

        if len(data) < 100:
            data = np.append(data, data_int)
        else:
            data[0:99] = data[1:100]
            data[99] = data_int
        lines.set_xdata(np.arange(0, len(data)))
        lines.set_ydata(data)

        canvas.draw()

    root.after(1, plot_data)


def plot_start():
    global cond
    cond = True
    s.reset_input_buffer()


def plot_stop():
    global cond
    cond = False


# -----Create GUI------
root = Tk()
root.title('EEG Data Visualizer')
root.configure(background='white')
root.geometry('650x500')

# -------Create plot--------
fig = Figure()
ax = fig.add_subplot(111)
ax.set_title('Electrode Signal')
ax.set_xlabel('Sample')
ax.set_ylabel('Voltage')
ax.set_xlim(0, 100)
ax.set_ylim(-0.5, 5)
lines = ax.plot([], [])[0]

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().place(x=10, y=10, width=600, height=400)
canvas.draw()

# ------Create button------
root.update()
start = Button(root, text='Start', font=('calibri', 12), command=plot_start)
start.place(x=100, y=450)

root.update()
stop = Button(root, text='Stop', font=('calibri', 12), command=plot_stop)
stop.place(x=start.winfo_x() + start.winfo_reqwidth() + 20, y=450)

# ----Start serial-----
s = serial.Serial(
    port='COM6',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
s.reset_input_buffer()

root.after(1, plot_data)
root.mainloop()
