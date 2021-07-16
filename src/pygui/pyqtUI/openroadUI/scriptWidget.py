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

from PyQt5.QtCore import QDate, QFile, Qt, QTextStream, QSettings, pyqtSignal, QObject, QEvent, pyqtSignal, QPoint

from PyQt5.QtGui import (QFont, QIcon, QKeySequence, QTextCharFormat, QMouseEvent,
                         QTextCursor, QTextTableFormat, QColor)

from PyQt5.QtWidgets import (QAction, QApplication,
                             QDialog, QFileDialog, QMainWindow, QMessageBox,
                             QTextEdit, QLineEdit, QLabel, QListWidget, QDockWidget,
                             QPushButton, QRadioButton, QToolButton, QLabel, QMenu,
                             QStackedLayout, QGridLayout, QHBoxLayout, QVBoxLayout,
                             QWidget)

from PyQt5.Qt import (QEventLoop,
                      QIcon, QMenu, Qt,
                      QSize, QPainter, QRect,
                      QCursor, QKeySequence, QTimer)

from . import openroadGlobals as org
import openroadpy as orp
import time
import sys
import os

from enum import Enum
from io import StringIO
from contextlib import redirect_stdout, redirect_stderr

class ShellLanguage(Enum):
    TCL = 1
    PYTHON = 2

class ChoiceButton(QPushButton):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.clicked_near_arrow = None
        # set icon by letter
        self.label_icon = QLabel("  ▼  ")
        self.label_icon.setAttribute(Qt.WA_TranslucentBackground)
        self.label_icon.setAttribute(Qt.WA_TransparentForMouseEvents)
        icon_size = QSize(19, 19)
        # set icon by picture
        self.pixmap_default = QIcon("default_button.png").pixmap(icon_size) # prepare images if necessary
        self.pixmap_presssed = QIcon("pressed_button.png").pixmap(icon_size) # prepare images if necessary
        self.pic_icon = QLabel()
        self.pic_icon.setAttribute(Qt.WA_TranslucentBackground)
        self.pic_icon.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.pic_icon.setPixmap(self.pixmap_default)
        # layout
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 6, 3)
        lay.setSpacing(0)
        lay.addStretch(1)
        lay.addWidget(self.pic_icon)
        lay.addWidget(self.label_icon)
        self.label_icon.hide()
        self.label_icon.show()
    def set_icon(self, pressed):
        if pressed:
            self.label_icon.setStyleSheet("QLabel{color:white}")
            self.pic_icon.setPixmap(self.pixmap_presssed)
        else:
            self.label_icon.setStyleSheet("QLabel{color:black}")
            self.pic_icon.setPixmap(self.pixmap_default)
    def mousePressEvent(self, event):
        if event.type() == QEvent.MouseButtonPress:
            self.set_icon(pressed=True)
            # figure out press location
            topRight = self.rect().topRight()
            bottomRight = self.rect().bottomRight()
            # get the rect from QStyle instead of hardcode numbers here
            arrowTopLeft = QPoint(topRight.x()-19, topRight.y())
            arrowRect = QRect(arrowTopLeft, bottomRight)
            if arrowRect.contains(event.pos()):
                self.clicked_near_arrow = True
                self.blockSignals(True)
                QPushButton.mousePressEvent(self, event)
                self.blockSignals(False)
                print('clicked near arrow')
                self.open_context_menu()
            else:
                self.clicked_near_arrow = False
                QPushButton.mousePressEvent(self, event)
    def mouseMoveEvent(self, event):
        if self.rect().contains(event.pos()):
            self.set_icon(pressed=True)
        else:
            self.set_icon(pressed=False)
        QPushButton.mouseMoveEvent(self, event)
    def mouseReleaseEvent(self, event):
        self.set_icon(pressed=False)
        if self.clicked_near_arrow:
            self.blockSignals(True)
            QPushButton.mouseReleaseEvent(self, event)
            self.blockSignals(False)
        else:
            QPushButton.mouseReleaseEvent(self, event)
    def setMenu(self, menu):
        self.menu = menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_context_menu)
    # ContextMenueのlauncher
    def open_context_menu(self, point=None):
        point = QPoint(7, 23)
        self.menu.exec_(self.mapToGlobal(point))
        event = QMouseEvent(QEvent.MouseButtonRelease, QPoint(10, 10), Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
        self.mouseReleaseEvent(event)

class ScriptPyLog(object):
    def __init__(self, logArr):
        self.terminal = sys.stdout
        self.logger = logArr
        self.textLog = ""

    def write(self, message):
        self.terminal.write(message)
        self.textLog += message
        if "\n" in message:
           self.logger.append(self.textLog.rstrip())
           self.textLog = ""

    def messages(self):
        return self.textLog

    def flush(self):
        pass


class ScriptWidgetComm(QObject):
    commandExecuted = pyqtSignal()


class ScriptWidget(QDockWidget):
    history_limit = 100

    def __init__(self, title="Transcript", parent=None):
        super().__init__(title, parent)
        self.output_ = QTextEdit()
        self.input_ = QLineEdit()

        self.pauser_ = QPushButton("OR >")
        self.langBtn_ = QPushButton("Tcl")
        self.shell_ = ShellLanguage.TCL

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

        self.pauserMenu_ = QMenu()

        self.inner_layout = QHBoxLayout()
        #self.inner_layout.addWidget(self.langBtn_)
        self.inner_layout.addWidget(self.pauser_)
        self.inner_layout.addWidget(self.input_, stretch=1)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.output_, stretch=1)
        self.layout.addLayout(self.inner_layout)

        self.container = QWidget()
        self.container.setLayout(self.layout)

        self.input_.returnPressed.connect(self.executeCommand)
        self.pauser_.clicked.connect(self.pauserClicked)
        self.langBtn_.clicked.connect(self.toggleLanguage)

        self.setWidget(self.container)

        self.pauser_.setStyleSheet("")

        tclResObj = orp.openRoadTclRes()
        tclResObj.tclRes = "Success"
        tclResObj.tclRetVal = True
        self.updateOutput(tclResObj, channelOut=False)

    def toggleLanguage(self):
        if self.shell_ == ShellLanguage.TCL :
            self.shell_ = ShellLanguage.PYTHON
            self.langBtn_.setText("Python")
        else :
            self.shell_ = ShellLanguage.TCL
            self.langBtn_.setText("Tcl")

    def initOnce(self):
        self.pauser_.setText("Initializing")
        self.pauser_.setStyleSheet("background-color: red")
        self.input_.clear()

        # Make changes visible while command runs
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)
        org.OpenRoadGlobals.get_openroad_intf().initOnce()

        self.pauser_.setText("OR > ")
        self.pauser_.setStyleSheet("")
        self.comm_.commandExecuted.emit()

    def executePythonCmd(self, cmdStr):
        commandArgs = cmdStr.split(" ")
        hasError = False
        origstdout = sys.stdout
        sys.stdout = ScriptPyLog(self.output_)
        if len(commandArgs) < 2 or commandArgs[1] == "-h" or commandArgs[1] == "--help":
            ret_val = "Usage: exec_python <python_file>"
        else:
            if os.path.exists(commandArgs[1]):
               try:
                  exec(open(commandArgs[1]).read())
                  ret_val = "Executed file successfully"
               except Exception as e:
                  print(e)
                  hasError = True
                  ret_val = "Error executing the file"
            else:
               hasError = True
               ret_val = "File " + commandArgs[1] + " not found."

        sys.stdout = origstdout

        return (not hasError), ret_val

    # @pyqtSlot()
    def executeCommand(self):
        self.pauser_.setText("Running")
        self.pauser_.setStyleSheet("background-color: red")
        command = self.input_.text()
        self.input_.clear()

        # Show the command that we executed
        self.output_.setTextColor(Qt.black)
        self.output_.append("> " + command)

        # Make changes visible while command runs
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)

        ret_val = ""
        if self.shell_ == ShellLanguage.PYTHON :
            cmdOut = StringIO()
            cmdErr = StringIO()

            with redirect_stdout(cmdOut):
                with redirect_stderr(cmdErr) :
                    org.OpenRoadGlobals.getPyInterp().runsource(command)
            if len(cmdErr.getvalue()) > 0:
                self.output_.setTextColor(Qt.red)
                cmdOut = cmdErr.getvalue()
            else:
                cmdOut = cmdOut.getvalue()
                self.output_.setTextColor(Qt.blue)
            self.output_.append(cmdOut)
        else:
            tcl_ret_val = org.OpenRoadGlobals.get_tcl_bridge().evalTclCommand(command)
            ret_val = tcl_ret_val

            # Show its output
            # Not from Tcl Channel
            self.updateOutput(ret_val, channelOut=False)

            # Show its output
            # Not from Tcl Channel
            self.updateOutput(tcl_ret_val, channelOut=False)

            # Update history; ignore repeated commands and keep last 100
            if len(self.history_) == 0 or self.history_[-1] != command:
                if len(self.history_) == ScriptWidget.history_limit:
                    self.history_ = self.history_[1:]
                self.history_.append(command)

                self.historyPosition_ = len(self.history_)
        self.pauser_.setText("OR > ")
        self.pauser_.setStyleSheet("")
        if self.shell_ == ShellLanguage.TCL:
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
        self.prior_text = self.pauser_.text()
        self.prior_enable = self.pauser_.isEnabled()
        self.prior_style = self.pauser_.styleSheet()

        self.pauser_.setText("Continue")
        self.pauser_.setStyleSheet("background-color: yellow")
        self.pauser_.setEnabled(True)
        self.paused_ = True

        # Keep processing events until the user continues
        while self.paused_:
            # time.sleep(0.001)  # Sleep for 1 milli seconds
            QApplication.processEvents()

        self.pauser_.setText("Running")
        self.pauser_.setStyleSheet("background-color: red")
        self.pauser_.setEnabled(False)
        self.pauser_.repaint()
        print("Returning from Pause")
        sys.stdout.flush()
        return

    # @pyqtSlot()
    def pauserClicked(self):
        print("Pauser Clicked...")
        sys.stdout.flush()
        self.paused_ = False
        self.pauser_.setText("Running")
        self.pauser_.setStyleSheet("background-color: red")
        self.pauser_.setEnabled(False)
        self.pauser_.repaint()
