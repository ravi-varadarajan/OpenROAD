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

import math
from PyQt5.QtCore import QDate, QFile, QIODevice, Qt, QTextStream, QTimer, QEvent, pyqtSignal, Qt
from PyQt5.QtGui import (QFont, QIcon, QKeySequence, QTextCharFormat,
                         QTextCursor, QTextTableFormat)
#from PyQt5.QtPrintSupport import QPrintDialog, QPrinter
from PyQt5.QtWidgets import (QAction, QApplication, QDialog, QDockWidget,
                             QFileDialog, QListWidget, QMainWindow, QMessageBox, QTextEdit,
                             QOpenGLWidget, QTreeView, QFrame, QLabel, QScrollArea, QSplitter)

from qtconsole.rich_jupyter_widget import RichJupyterWidget
from qtconsole.inprocess import QtInProcessKernelManager

from PyQt5.QtGui import *
from PyQt5.QtCore import *

import OpenGL.GL as gl
from OpenGL.GL import *
from OpenGL.GLU import *

from . import openroad_rc
from . import openroadGlobals as org
from . import openRoadTclBridge as ortcl

from . import orpyInit

from . import scriptWidget as sw
from . import hierBrowser as hb
#from . import layerViewer as ltv
from . import layerViewerNew as ltv

from . import tab

import os
import sys
import opendbpy as odb
import openroadpy as orp

from . import layoutViewer as lv
import inspect

import code
from pathlib import Path

#app = None

SHOW_SMILEY = 'SHOW_SMILEY' in os.environ

class ConsoleWidget(RichJupyterWidget):
    def __init__(self, customBanner=None, *args, **kwargs):
        super(ConsoleWidget, self).__init__(*args, **kwargs)

        if customBanner is not None:
            self.banner = customBanner

        self.font_size = 6
        self.kernel_manager = kernel_manager = QtInProcessKernelManager()
        kernel_manager.start_kernel(show_banner=False)
        kernel_manager.kernel.gui = 'qt'
        self.kernel_client = kernel_client = self._kernel_manager.client()
        kernel_client.start_channels()

        def stop():
            kernel_client.stop_channels()
            kernel_manager.shutdown_kernel()
            #guisupport.get_app_qt().exit()

        #self.exit_requested.connect(stop)

    def push_vars(self, variableDict):
        """
        Given a dictionary containing name / value pairs, push those variables
        to the Jupyter console widget
        """
        self.kernel_manager.kernel.shell.push(variableDict)

    def clear(self):
        """
        Clears the terminal
        """
        self._control.clear()

        # self.kernel_manager

    def print_text(self, text):
        """
        Prints some plain text to the console
        """
        self._append_plain_text(text)

    def execute_command(self, command):
        """
        Execute a command in the frame of the console widget
        """
        self._execute(command, False)

class VLine(QFrame):
    # a simple VLine, like the one you get from designer
    def __init__(self):
        super(VLine, self).__init__()
        self.setFrameShape(self.VLine | self.Sunken)


class MainWindow(QMainWindow):
    mouseButtonReleased = pyqtSignal(int, int)

    def __init__(self):
        super(MainWindow, self).__init__()     

        #org.OpenRoadGlobals.openRoadIntf = PythonOpenRoadIntf()
        #org.OpenRoadGlobals.openRoadTclBridge = ortcl.TclInterpBridge()

        orIntf = org.OpenRoadGlobals.get_openroad_intf()

        self.layoutWidget = lv.LayoutWindow(self)
        self.setCentralWidget(self.layoutWidget)

        self.createActions()
        self.createMenus()
        # self.createToolBars()
        self.createStatusBar()
        self.createDockWindows()

        self.setWindowTitle("OpenROAD")

        self.layoutWidget.getLayoutViewer().canvasConnected.connect(self.updateViews)

        self.beginRun()

        org.OpenRoadGlobals.openRoadMainWin = self
        org.OpenRoadGlobals.openRoadLayoutViewer = self.layoutWidget.getLayoutViewer()
        #QTimer.singleShot(500, self.processInitOnce)
        self.processInitOnce()

        if SHOW_SMILEY:
            self.topLayer = orp.GLLayer.createDummyLayerTree()
            canvas = orp.GLCanvas.getDummyCanvas()
            self.layoutWidget.getLayoutViewer().attachCanvasToLayoutView(canvas)
        else:
            #print("SHOWING EMPTY CANVAS")
            self.topLayer = orp.GLLayer.createDummyLayerTree(True)
            canvas = orp.GLCanvas.getCanvas("OR")
            if canvas is None:
                canvas = orp.GLCanvas.getDummyCanvas(True)
            self.layoutWidget.getLayoutViewer().attachCanvasToLayoutView(canvas)
            QTimer.singleShot(150, self.showWorldView)
            # self.layoutWidget.showWorldView()
        self.resize(1000, 800)

        org.OpenRoadGlobals.incrementX()

        pyInterp = code.InteractiveInterpreter(locals=locals())
        org.OpenRoadGlobals.shellPythonInterp = pyInterp
        org.OpenRoadGlobals.getPyInterp().runsource("import sys, os")
        org.OpenRoadGlobals.getPyInterp().runsource("sys.path.append(os.getcwd())")
        org.OpenRoadGlobals.getPyInterp().runsource("sys.path.append(Path.home()")   

        #self.resize(1500, 1000)

        # self.installEventFilter(self)

    def showWorldView(self):
        #print("Came to Show World View as timer fired...")
        self.layoutWidget.showWorldView()
        self.layoutWidget.worldView_.hide()

    def updateViews(self, connectedCanvas):
        self.layersTree.populateLayerModel(connectedCanvas.getTopLayerNode())
        self.layersTree.repaint()

    def eventFilter(self, obj, xEvent):
        if xEvent.type() == QEvent.MouseButtonRelease or xEvent.type() == QEvent.NonClientAreaMouseButtonRelease:
            self.mouseButtonDown = False
            self.windowDragResizeStarted = False
        if xEvent.type() == QEvent.Resize:
            #print("Executing Mainwindow Mouse Resize...")
            self.windowDragResizeStarted = True
            xEvent.accept()
            return True
        return super(MainWindow, self).eventFilter(obj, xEvent)

    def processInitOnce(self):
        #print("Came to Process Init Once in Python")
        self.scriptWidget.initOnce()

    def beginRun(self):
        pass
        # self.textEdit.clear()

    def about(self):
        QMessageBox.about(self, "About OpenRoad EDA",
                          "<b>DEMOCRATIZING HARDWARE DESIGN</b><br>"
                          "The OpenRoad Project attacks the barriers of Cost, Expertise and Uncertainity<br>."
                          "(i..e, Risk) that block the feasibility of hardware design in advanced technologies.")

    def createActions(self):
        self.newDesignAct = QAction(QIcon(':/images/new.png'), "&New Run",
                                    self, shortcut=QKeySequence.New,
                                    statusTip="Start a new Design Run", triggered=self.beginRun)

        self.quitAct = QAction("&Quit", self, shortcut="Ctrl+Q",
                               statusTip="Quit the application", triggered=self.close)

        self.aboutAct = QAction("&About", self,
                                statusTip="Show the application's About box",
                                triggered=self.about)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu("&File")
        self.fileMenu.addAction(self.quitAct)

        self.viewMenu = self.menuBar().addMenu("&Windows")

        self.menuBar().addSeparator()

        self.helpMenu = self.menuBar().addMenu("&Help")
        self.helpMenu.addAction(self.aboutAct)

    def createToolBars(self):
        self.fileToolBar = self.addToolBar("File")
        self.fileToolBar.addAction(self.newDesignAct)

    def createStatusBar(self):
        self.selObjInfo = QLabel("")
        self.locLabel = QLabel("Loc : ")

        self.statusBar().showMessage("Ready")
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(self.locLabel)

    def showStatusMessage(self, msg):
        self.statusBar().showMessage(msg)

    def resetStatusMessage(self):
        self.statusBar().showMessage("Ready")

    def createDockWindows(self):
        dock = QDockWidget("DesignBrowser", self)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea |
                             Qt.RightDockWidgetArea | Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        self.items = []

        self.hierBrowser = QTreeView(dock)
        self.designTreeModel = hb.DesignTreeModel(self.items)

        self.hierBrowser.setModel(self.designTreeModel)
        dock.setWidget(self.hierBrowser)

        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self.viewMenu.addAction(dock.toggleViewAction())

        self.techDock = QDockWidget("Tech Layers", self)
        self.layersTree = ltv.LayerViewer(self)
        self.techDock.setWidget(self.layersTree)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.techDock)
        self.viewMenu.addAction(self.techDock.toggleViewAction())

        dock = sw.ScriptWidget("OpenROAD Transcript", self)
        dock.setFloating(False)

        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        self.viewMenu.addAction(dock.toggleViewAction())

        self.scriptWidget = dock
        self.scriptWidget.setAllowedAreas(Qt.AllDockWidgetAreas)

        self.ipythonDock = QDockWidget("IPython Shell", self)
        self.ipythonConsole = ConsoleWidget(parent=self)
        self.ipythonConsole.push_vars({'orGlobals' : org.OpenRoadGlobals})
        self.ipythonDock.setWidget(self.ipythonConsole)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.ipythonDock)
        self.viewMenu.addAction(self.ipythonDock.toggleViewAction())
        self.ipythonDock.setFloating(False)

        self.tabledock = QDockWidget("Table Viewer", self)
        self.tabledockwidget = tab.TabViewer()
        self.tabledock.setWidget(self.tabledockwidget)
        self.addDockWidget(Qt.TopDockWidgetArea, self.tabledock)
        self.viewMenu.addAction(self.tabledock.toggleViewAction())
        self.tabledock.setVisible(False)

        self.layersTree.layerUpdated.connect(self.layerChanged)

    def getScriptWidget(self):
        return self.scriptWidget

    def populateHierTree(self):
        org.printDebugMsg(f"Came to PopulateHierTree")
        self.designTreeModel.buildModel()

    def layerChanged(self, orLayer, changeInLayer):
        self.layoutWidget.repaintView()


def main():
    orpyInit.init()
    app = QApplication([])
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())
