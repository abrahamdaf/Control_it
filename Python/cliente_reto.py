import numpy as np
import matplotlib.pyplot as plt
import socket
import tkinter as tk
import time
import threading


def tkLoop():
    UDP_IP = '192.168.43.125'
    UDP_SEND_PORT = 6666
    root = tk.Tk()
    entry = tk.Entry(root)

    def submit(event):
        while True:
            try:
                sockS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                UDP_PAYLOAD = str(int(entry.get()))
                sockS.sendto(bytes(UDP_PAYLOAD, "utf-8"),
                             (UDP_IP, UDP_SEND_PORT))
                sockS.close()
                time.sleep(0.1)
                break
            except:
                print("FAILED")
                pass

    entry.bind("<Return>", submit)
    entry.pack()
    root.mainloop()


def udpLoop():
    UDP_PORT = 1234

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT))

    figure, ax = plt.subplots(2)
    y1 = np.array([0])
    x1 = np.array([0])
    x2 = np.array([0])
    y2 = np.array([0])

    plt.ion()
    plot1, = ax[0].plot(x1, y1)
    plot2, = ax[1].plot(x2, y2)
    ax[1].set_title("Right RPM")
    ax[0].set_title("Left RPM")

    numR = 0
    numL = 0

    plt.show()
    while True:
        data, addr = sock.recvfrom(1024)
        string = data.decode('utf-8')
        if string.find("Right RPM"):
            numR = float(string[string.find('M')+1:-1])
        elif string.find("Left RPM"):
            numL = float(string[string.find('M')+1:-1])

        if numR:
            x1 = np.append(x1, x1[-1]+0.6)
            y1 = np.append(y1, numR)

            plot1.set_xdata(x1)
            plot1.set_ydata(y1)

            ax[0].set_ylim(np.min(y1), np.max(y1)+20)
            ax[0].set_xlim(np.min(x1), np.max(x1))

            figure.canvas.draw()
            figure.canvas.flush_events()
        if numL:
            x2 = np.append(x2, x2[-1]+0.6)
            y2 = np.append(y2, numL)

            plot2.set_xdata(x2)
            plot2.set_ydata(y2)

            ax[1].set_ylim(np.min(y2), np.max(y2)+20)
            ax[1].set_xlim(np.min(x2), np.max(x2))

            figure.canvas.draw()
            figure.canvas.flush_events()


t1 = threading.Thread(target=udpLoop)
t1.start()

t2 = threading.Thread(target=tkLoop)
t2.start()
