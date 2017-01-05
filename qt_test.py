#!/usr/bin/env python
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import Queue
from threading import Thread
import time
from numpy.random import *

class QtImage:
    
    def __init__(self, windowTitle="QtImage"):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.win = pg.GraphicsWindow(windowTitle)
        self.view = self.win.addViewBox()
        self.imageItem = pg.ImageItem()
        self.view.addItem(self.imageItem)
        self.view.setAspectLocked(True)
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.q = Queue.Queue()

    def getPort(self):
        return self.q

    def update(self):
        try:
            data = self.q.get(block=False)
#            print("got data")
            self.imageItem.setImage(data, autoLevels=False)
            logImage(self.imageItem)
        except Queue.Empty:
            #                print("exited")
            pass




class QtPlotter:

    def __init__(self, windowTitle="QtPlotter"):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.win = pg.GraphicsWindow(windowTitle)
        self.ax = self.win.addPlot()
        self.ax.enableAutoRange('xy', False)
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.ax.setAspectLocked(True)

    def getPort(self):
        q = Queue.Queue()
        plt = self.ax.plot()

        self.ports.append((q, plt))
        return q

    def update(self):
      #      print("update")
        for q, plt in self.ports:
            try:
                #                with q.mutex:
                #                    print("one q")
                params = q.get(block=False)
                data = params[0]
                if (len(params) > 1):
                    color = params[1]
                else:
                    color = "FFFFFF"
                if (len(params) > 2):
                    pen = params[2]
                else:
                    pen = None

                plt.setData(
                    np.asarray(data[0, :]).flatten(),
                    np.asarray(data[1, :]).flatten(), pen=pen, symbol="o",
                    symbolPen=pg.mkPen({'color': color, 'width': 2}),
                    symbolSize=1
                )

            except Queue.Empty:
                #                print("exited")
                pass
 #       print("return from update")


def qtLoop():
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        print("starting qtLoop")
        QtGui.QApplication.instance().exec_()

    else:
        print("refusing to start qtLoop")


def example():
    plotter = QtPlotter()
    curve = plotter.getPort()

    def producer():
        while True:
            curve.put((np.random.random(size=(2, 100)), "#00FFFF"))
            #curve. plot(np.random.random(100)), "#00FFFF")
            time.sleep(0.1)

    p = Thread(target=producer)
    p.daemon = True
    p.start()

    qtLoop()

def update():
    imageItem.setImage(np.random.normal(size=(100,100)) , autoLevels=False)


hexByName = {"black" 	    : "#000000",
             "mediumgreen" 	: "#47b73b",
             "lightgreen" 	: "#7CCF6F",
             "darkblue" 	: "#5D4EFF",
             "lightblue" 	: "#8072FF",
             "darkred" 	    : "#B66247",
             "cyan"         : "#5DC8ED",
             "mediumred" 	: "#D76B48",
             "lightred" 	: "#FB8F6C",
             "darkyellow" 	: "#C3CD41",
             "lightyellow" 	: "#D3DA76",
             "darkgreen" 	: "#3E9F2F",
             "magenta" 	    : "#B664C7",
             "gray"         : "#cccccc",
             "white" 	    : "#ffffff"}

colorAssoc = {
             0: "gray",
             1: "darkred",
             2: "lightblue",
    }

rgbByName = {key: np.array((int(value[1:3], 16), int(value[3:5], 16), int(value[5:7], 16)))
             for key, value in hexByName.items()}





ports = []
timer = pg.QtCore.QTimer()
win = pg.GraphicsWindow("MyTest")
view = win.addViewBox()
imageItem = pg.ImageItem()
view.addItem(imageItem)
view.setAspectLocked(True)
timer.timeout.connect(update)
timer.start(0)
lut = np.zeros((7, 3), dtype=np.ubyte)
for key, val in colorAssoc.items():
    lut[key, :] = rgbByName[val]
imageItem.setLookupTable(lut)



if __name__ == "__main__":
    qtLoop()
    pass