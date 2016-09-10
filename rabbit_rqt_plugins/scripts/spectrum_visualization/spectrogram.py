#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy

from numpy import *  # noqa

from qt_gui.plugin import Plugin
from python_qt_binding.QtGui import QApplication, QWidget
from python_qt_binding.QtGui import QVBoxLayout, QHBoxLayout
from python_qt_binding.QtGui import QLineEdit, QPushButton
from python_qt_binding import QtCore

import seaborn  # noqa
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from rabbit_msgs.msg import Spectrum


class SpectrogramPlugin(Plugin):
    update_signal = QtCore.pyqtSignal()
    subscriber_signal = QtCore.pyqtSignal(str)

    def __init__(self, context):
        super(SpectrogramPlugin, self).__init__(context)
        self.setObjectName('Spectrogram')

        self._widget = QWidget()
        layout = QVBoxLayout()
        self._widget.setLayout(layout)

        layout_ = QHBoxLayout()
        self.line_edit = QLineEdit()
        layout_.addWidget(self.line_edit)
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self.apply_clicked)
        layout_.addWidget(self.apply_btn)
        layout.addLayout(layout_)

        self.fig = Figure((5, 4), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(111)
        self.fig.tight_layout()

        layout.addWidget(self.canvas)

        context.add_widget(self._widget)

        self.update_signal.connect(self.update_spectrogram)
        self.subscriber_signal.connect(self.update_subscriber)

        self.subscriber_signal.emit('spec')

    def spectrum_callback(self, data):
        nch = data.nch
        len = data.nfreq

        if self.spectrogram is None:
            self.spectrogram = zeros([len, 1000])

        s = array(data.data).reshape([nch, len, 2])[-1]
        s = linalg.norm(s, axis=1)
        s += 1e-8
        log(s, s)

        self.spectrogram = roll(self.spectrogram, -1, 1)
        self.spectrogram[:, -1] = s

        if data.header.seq % 100 == 0:
            self.update_signal.emit()

    def apply_clicked(self):
        self.update_subscriber(self.line_edit.displayText())

    def update_spectrogram(self):
        if self.spectrogram is not None:
            self.axes.clear()

            self.axes.imshow(self.spectrogram, aspect="auto", origin="lower", cmap="hot")
            self.axes.grid(None)

            self.axes.set_ylabel("Freq. [bin]")
            self.axes.set_xlabel("Time [frame]")

            self.fig.tight_layout()

            self.canvas.draw()

        QApplication.processEvents()

    def update_subscriber(self, topic_name):
        self.topic_name = topic_name
        self.line_edit.setText(self.topic_name)

        if hasattr(self, 'sub'):
            self.sub.unregister()
        self.spectrogram = None
        self.sub = rospy.Subscriber(topic_name,
                                    Spectrum, self.spectrum_callback,
                                    queue_size=500)

    def restore_settings(self, plugin_settings, instance_settings):
        topic_name = instance_settings.value('topic_name')
        self.subscriber_signal.emit(topic_name)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic_name', self.topic_name)

    def shutdown_plugin(self):
        pass
