#!/usr/bin/env python
from python_qt_binding.QtCore import Slot, Qt, QTimer, qWarning, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from python_qt_binding import QT_BINDING_VERSION
from distutils.version import LooseVersion
from matplotlib.figure import Figure
import operator
import numpy as np
import copy
import threading
import rospy
from trajectory_msgs.msg import JointTrajectory
import time
if LooseVersion(QT_BINDING_VERSION) >= LooseVersion('5.0.0'):
    from matplotlib.backends.backend_qt5agg \
        import FigureCanvasQTAgg as FigureCanvas
else:
    from matplotlib.backends.backend_qt4agg \
        import FigureCanvasQTAgg as FigureCanvas


class PlotCanvas(FigureCanvas):
    def __init__(self):
        super(PlotCanvas, self).__init__(Figure())
        self.axes = self.figure.add_subplot(111)
        self.axes.grid(True, color='gray')
        self.figure.tight_layout()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.updateGeometry()


class PlotWidget(QWidget):
    def __init__(self, parent=None):
        super(PlotWidget, self).__init__(parent)
        # create widgets
        self.canvas = PlotCanvas()
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        self.setLayout(vbox)

    def draw_curves(self, _desired_plots, curve_names, data):
        desired_plots = {key: value for key,
                         value in _desired_plots.items() if len(value) > 0}
        if (not desired_plots):
            return
        rows = max(len(value) for value in desired_plots.values())
        cols = len(desired_plots)
        self.canvas.figure.clear()

        t0 = time.time()

        traj_names = ['position', 'velocity', 'acceleration', 'effort']
        for col, (jointName, itemsToPlot) in enumerate(desired_plots.items()):
            for key, curve in itemsToPlot.items():
                ax = self.canvas.figure.add_subplot(
                    rows,
                    cols, cols*traj_names.index(key)
                    + col + 1)
                ax.grid()
                ax.plot(curve[0], curve[1])
        t1 = time.time()

        print('\n --- ------------------\n', t1 - t0)
        # self.canvas.axes.clear()
        # self.canvas.axes.grid(True, color='gray')
        # for name in curve_names:
        #     xdata, ydata = data[name]
        #     self.canvas.axes.plot(xdata, ydata, 'o-', label=name)[0]
        # self.update_legend()
        # print('\n --- desired plots\n')
        # print(desired_plots)
        # print('\n --- ------------------\n')
        self.canvas.draw()

    def update_legend(self):
        handles, labels = self.canvas.axes.get_legend_handles_labels()
        self.canvas.axes.legend(handles, labels, loc='upper left')
