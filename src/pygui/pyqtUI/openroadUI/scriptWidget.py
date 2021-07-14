'''
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, OpenROAD
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
'''

from PyQt5.QtCore import QDate, QFile, Qt, QTextStream, QSettings, pyqtSignal, QObject, QEvent, pyqtSignal

from PyQt5.QtGui import (QFont, QIcon, QKeySequence, QTextCharFormat,
                         QTextCursor, QTextTableFormat, QColor)

from PyQt5.QtWidgets import (QAction, QApplication,
                             QDialog, QFileDialog, QMainWindow, QMessageBox,
                             QTextEdit, QLineEdit, QLabel, QListWidget, QDockWidget,
                             QPushButton, QRadioButton, QToolButton,
                             QStackedLayout, QGridLayout, QHBoxLayout, QVBoxLayout,
                             QWidget)

from PyQt5.Qt import (QEventLoop,
                      QIcon, QMenu, Qt,
                      QSize, QPainter, QRect,
                      QCursor, QKeySequence, QTimer)

from . import openroadGlobals as org
import openroadpy as orp


class ScriptWidgetComm(QObject):
    commandExecuted = pyqtSignal()


class ScriptWidget(QDockWidget):
    history_limit = 100

    def __init__(self, title="Transcript", parent=None):
        super().__init__(title, parent)
        self.output_ = QTextEdit()
        self.input_ = QLineEdit()
        self.pauser_ = QPushButton()

        self.outputBuffer_ = []
        self.history_ = []
        self.historyPosition_ = 0
        self.paused_ = False

        self.buildWidgets()

        self.comm_ = ScriptWidgetComm()

        self.installEventFilter(self)

    def __del__(self):
        pass

    def eventFilter(self, obj, xEvent):
        if xEvent == None:
            return False

        if xEvent.type() == QEvent.Enter:
            self.setFocus()
            self.setFocusPolicy(Qt.StrongFocus)
            xEvent.accept()
            return True
        elif xEvent.type() == QEvent.Leave:
            org.OpenRoadGlobals.get_main_window().centralWidget().setFocus()
            org.OpenRoadGlobals.get_main_window().centralWidget().setFocusPolicy(Qt.StrongFocus)
        elif xEvent.type() == QEvent.KeyPress:
            key = xEvent.key()
            if key == Qt.Key_Down:
                if self.historyPosition_ < len(self.history_) - 1:
                    self.historyPosition_ += 1
                    self.input_.setText(self.history_[self.historyPosition_])
                elif self.historyPosition_ == len(self.history_) - 1:
                    self.historyPosition_ += 1
                    self.input_.clear()
                xEvent.accept()
            elif key == Qt.Key_Up:
                if self.historyPosition_ > 0:
                    self.historyPosition_ -= 1
                    self.input_.setText(self.history_[self.historyPosition_])
                    xEvent.accept()
            return True
        return super(ScriptWidget, self).eventFilter(obj, xEvent)

    def buildWidgets(self):
        self.output_.setReadOnly(True)
        self.pauser_.setEnabled(False)

        self.inner_layout = QHBoxLayout()
        self.inner_layout.addWidget(self.pauser_)
        self.inner_layout.addWidget(self.input_, stretch=1)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.output_, stretch=1)
        self.layout.addLayout(self.inner_layout)

        self.container = QWidget()
        self.container.setLayout(self.layout)

        self.input_.returnPressed.connect(self.executeCommand)
        self.pauser_.clicked.connect(self.pauserClicked)

        self.setWidget(self.container)

        self.pauser_.setText("OpenROAD > ")
        self.pauser_.setStyleSheet("")

        tclResObj = orp.openRoadTclRes()
        tclResObj.tclRes = "Success"
        tclResObj.tclRetVal = True
        self.updateOutput(tclResObj, channelOut=False)

    def initOnce(self):
        self.pauser_.setText("Initializing")
        self.pauser_.setStyleSheet("background-color: red")
        self.input_.clear()

        # Make changes visible while command runs
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)
        org.OpenRoadGlobals.get_openroad_intf().initOnce()

        self.pauser_.setText("OpenROAD > ")
        self.pauser_.setStyleSheet("")
        self.comm_.commandExecuted.emit()

    # @pyqtSlot()
    def executeCommand(self):
        self.pauser_.setText("Running")
        self.pauser_.setStyleSheet("background-color: red")
        command = self.input_.text()
        self.input_.clear()

        # Show the command that we executed
        self.output_.setTextColor(Qt.black)
        self.output_.append("> " + command)
        print("CMD: " + command)

        # Make changes visible while command runs
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)

        ret_val = ""
        if command.startswith("exec_python "):
            commandArgs = command.split(" ")
            if len(commandArgs) < 2 or commandArgs[1] == "-h" or commandArgs[1] == "--help":
                ret_val = "Usage: exec_python <python_file>"
            else:
                if os.path.exists(commandArgs[1]):
                   execfile(commandArgs[1])
                else:
                   ret_val = "File " + commandArgs[1] + " not found."
        else:
            tcl_ret_val = org.OpenRoadGlobals.get_tcl_bridge().evalTclCommand(command)
            ret_val = tcl_ret_val

        # Show its output
        # Not from Tcl Channel
        self.updateOutput(ret_val, channelOut=False)

        # Update history; ignore repeated commands and keep last 100
        if len(self.history_) == 0 or self.history_[-1] != command:
            if len(self.history_) == ScriptWidget.history_limit:
                self.history_ = self.history_[1:]
            self.history_.append(command)

        self.historyPosition_ = len(self.history_)
        self.pauser_.setText("OpenROAD > ")
        self.pauser_.setStyleSheet("")
        self.comm_.commandExecuted.emit()

    def updateOutput(self, tcl_ret_val, channelOut):
        # Show whatever we captured from the output channel in grey
        self.output_.setTextColor(QColor(0x30, 0x30, 0x30))
        for out in self.outputBuffer_:
            if len(out) > 0:
                self.output_.append(out)
        self.outputBuffer_ = []

        # Show the return value color-coded by true/false.
        result = tcl_ret_val.tclRes
        return_code = tcl_ret_val.tclRetVal
        if len(result) != 0:
            if channelOut == True:
                self.output_.setTextColor(Qt.black)
            else:
                self.output_.setTextColor(
                    Qt.blue if return_code == True else Qt.red)
            self.output_.append(result)

    def readSettings(self, settings):
        settings.beginGroup("scripting")
        self.history_ = settings.value("history").toStringList()
        self.historyPosition_ = len(self.history_)
        self.settings.endGroup()

    def writeSettings(self, settings):
        settings.beginGroup("scripting")
        settings.setValue("history", self.history_)
        settings.endGroup()

    # @pyqtSlot()
    def pause(self):
        prior_text = self.pauser_.text()
        prior_enable = self.pauser_.isEnabled()
        prior_style = self.pauser_.styleSheet()

        self.pauser_.setText("Continue")
        self.pauser_.setStyleSheet("background-color: yellow")
        self.pauser_.setEnabled(True)
        self.paused_ = True

        # Keep processing events until the user continues
        while self.paused_:
            QApplication.processEvents()

        self.pauser_.setText(prior_text)
        self.pauser_.setStyleSheet(prior_style)
        self.pauser_.setEnabled(prior_enable)

    # @pyqtSlot()
    def pauserClicked(self):
        self.paused_ = False
