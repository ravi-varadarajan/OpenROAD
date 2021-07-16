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
from PyQt5.QtCore import (QDate, QFile, QIODevice, Qt, QTextStream, QTimer,
                          QEvent, pyqtSignal, QSize, QPoint, QRect, QRectF, QSignalMapper)
from PyQt5.QtGui import (QFont, QIcon, QKeySequence, QTextCharFormat, QPixmap,
                         QTextCursor, QTextTableFormat, QPalette, QBrush, QColor, QIcon, QPen)
from PyQt5.QtWidgets import (QAction, QActionGroup, QApplication, QDialog, QDockWidget,
                             QFileDialog, QListWidget, QMainWindow, QMessageBox, QTextEdit,
                             QOpenGLWidget, QTreeView, QGraphicsColorizeEffect, QLabel, QSplitter, QToolButton,
                             QScrollArea, QWidget, QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QToolBar, QLineEdit, QShortcut, QMenu)

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtOpenGL

from PyQt5 import QtCore, QtGui, QtWidgets

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from . import openroad_rc
import openroadpy as orp
from . import openroadGlobals as org

import os
import sys
import traceback

from enum import Enum

from functools import partial

DRAW_IN_PYTHON = 'DRAW_IN_PYTHON' in os.environ
SHOW_SMILEY = 'SHOW_SMILEY' in os.environ


class WorldViewGLWidget(QOpenGLWidget):
    modifyVisibleArea = pyqtSignal(orp.GLRectangle)

    def __init__(self, parent=None):
        super(WorldViewGLWidget, self).__init__(parent)
        self.setMouseTracking(True)
        self.aspect = 1
        self.width_ = None
        self.height_ = None

        self.canvas = None
        self.topLayer = None

        self.mouseClickCoord = None
        self.curMouseCoord = None
        self.motionStarted = False
        self.mouseButton = None

        self.onceOnly = False
        self.visAreaShp = None
        self.worldVisLayer = None
        self.initiatedFromWorldView = False

        self.glView = orp.GLView.getView("WorldView")

    def attachCanvas(self, cnv, visArea=None):
        self.canvas = cnv
        self.topLayer = self.canvas.getTopLayerNode()
        self.glView.setTopCanvas(self.canvas)
        if visArea is not None:
            self.glView.clearViewShapes(orp.SELECT_VIEW_SHAPE)
            if self.visAreaShp is not None:
                orp.GLShape.destroyShape(self.visAreaShp)

        drawLayers = self.topLayer.getWorldViewDrawLayers()
        selLayers = self.topLayer.getWorldViewSelectLayers()
        self.worldVisLayer = selLayers[0]
        self.visAreaShp = orp.GLRectShape.getGLRectShape(visArea)
        self.glView.addShapeOnView(self.visAreaShp, self.worldVisLayer)
        self.glView.setWorldViewParams(drawLayers, selLayers)

    def updateVisibleArea(self, visRect):
        self.makeCurrent()
        if self.canvas == None:
            return
        self.glView.clearViewShapes(orp.SELECT_VIEW_SHAPE)
        if self.visAreaShp is not None:
            orp.GLShape.destroyShape(self.visAreaShp)
        self.visAreaShp = orp.GLRectShape.getGLRectShape(visRect)
        self.glView.addShapeOnView(self.visAreaShp, self.worldVisLayer)
        self.glView.resetPixmaps(False, True)
        if self.initiatedFromWorldView == False:
            self.repaint()

    def mouseMoveEvent(self, evt):
        self.makeCurrent()
        self.curMouseCoord = orp.GLPoint2D(evt.x(), evt.y())
        if self.mouseClickCoord != None and self.mouseButton == Qt.LeftButton:
            curCoord = orp.GLPoint2D(evt.x(), evt.y())
            if curCoord == self.mouseClickCoord:
                return
            elif self.motionStarted == False:
                self.motionStarted = True
                self.glView.stopAnimation()
                #self.glView.startAnimationObject(orp.MOTION_OR_SHAPE, self.mouseClickCoord)
                self.glView.startAnimationObject(
                    self.visAreaShp, self.mouseClickCoord, self.worldVisLayer)
                if self.glView.getMotionObjectCount() == 0:
                    self.motionStarted = False
                    self.glView.stopAnimation()
            else:
                self.repaint()

    def mousePressEvent(self, evt):
        self.makeCurrent()
        self.setFocus()
        self.setFocusPolicy(Qt.StrongFocus)
        x = evt.x()
        y = evt.y()
        self.mouseClickCoord = orp.GLPoint2D(x, y)
        self.glView.setMouseClickedCoord(self.mouseClickCoord)
        self.motionStarted = False
        self.mouseButton = evt.button()
        evt.accept()

    def mouseReleaseEvent(self, evt):
        self.makeCurrent()
        self.setFocus()
        self.setFocusPolicy(Qt.StrongFocus)
        self.mouseButton = None
        mouseReleaseCoord = orp.GLPoint2D(evt.x(), evt.y())
        self.initiatedFromWorldView = True
        if evt.button() == Qt.LeftButton and self.mouseClickCoord != mouseReleaseCoord:
            mouseClickStart = self.glView.getWorldCoord(self.mouseClickCoord)
            mouseClickEnd = self.glView.getWorldCoord(mouseReleaseCoord)
            transX = mouseClickStart.x() - mouseClickEnd.x()
            transY = mouseClickStart.y() - mouseClickEnd.y()
            if self.visAreaShp is not None:
                visArea = self.visAreaShp.getBoundingBox()
                newVisArea = visArea.translate(-transX, -transY)
            self.glView.stopAnimation()
            if self.visAreaShp is not None:
                self.modifyVisibleArea.emit(newVisArea)
            self.motionStarted = False
            self.mouseClickCoord = None
        self.repaint()
        self.initiatedFromWorldView = False
        evt.accept()

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)

        glShadeModel(GL_FLAT)
        glClearDepth(1.0)
        glDepthFunc(GL_ALWAYS)
        glEnable(GL_DEPTH_TEST)

        glReadBuffer(GL_BACK)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)

    def resizeGL(self, width, height):
        if height == 0:
            self.height_ = 1
        self.width_ = width
        self.height_ = height
        self.aspect = width/height
        glViewport(0, 0, width, height)
        self.glView.setViewPortDimensions(self.width_, self.height_)

    def paintGL(self):
        self.makeCurrent()
        if self.motionStarted == False:
            self.glView.setViewPortDimensions(self.width_, self.height_)
        try:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        except BaseException as e:
            org.printDebugMsg(f"Encountered Exception line 124 {str(e)}")

        if self.motionStarted == False:
            self.glView.draw()
        else:
            self.glView.drawMotionObjs(self.curMouseCoord)
        return


class WorldViewer(QDialog):
    modifyVisibleArea = pyqtSignal(orp.GLRectangle)
    layoutVisAreaChanged = pyqtSignal(orp.GLRectangle)

    def __init__(self, parent=None):
        super(WorldViewer, self).__init__(parent)

        self.showGraphicsScene = False

        self.gridLayout = QGridLayout(self)
        self.frame = QFrame(self)
        self.frame.setFrameShape(QFrame.WinPanel)
        self.frame.setFrameShadow(QFrame.Raised)

        self.gridLayout_2 = QGridLayout(self.frame)
        self.worlViewWidget = WorldViewGLWidget(self.frame)
        self.gridLayout_2.addWidget(self.worlViewWidget, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.frame, 0, 0, 1, 1)

        if self.showGraphicsScene:
            self.scene = QGraphicsScene()
            self.graphicView = QGraphicsView(self.scene, self)

            self.cnvBrush = QBrush(Qt.gray)
            self.cnvBrush.setStyle(Qt.Dense6Pattern)

            self.viewBrush = QBrush(Qt.yellow)
            self.viewBrush.setStyle(Qt.DiagCrossPattern)

            self.cnvPen = QPen(Qt.red)
            self.viewPen = QPen(Qt.green)
        self.setFixedSize(300, 300)
        self.setWindowTitle("World View")

        self.worlViewWidget.modifyVisibleArea.connect(
            lambda newVisRect: self.modifyVisibleArea.emit(newVisRect))

    def showWorldCanvas(self, cnv, visArea):
        self.worlViewWidget.attachCanvas(cnv, visArea)

    def showEvent(self, evt):
        self.worlViewWidget.glView.resetPixmaps()
        self.worlViewWidget.repaint()
        evt.accept()

    def buildScene(self, cnvRect, visArea):
        self.scene.clear()
        self.graphicView.setGeometry(0, 0, cnvRect.width(), cnvRect.height())
        self.visRect = self.scene.addRect(
            visArea, self.viewPen, self.viewBrush)
        self.cnvRect = self.scene.addRect(cnvRect, self.cnvPen, self.cnvBrush)
        self.visRect.setFlag(QGraphicsItem.ItemIsMovable)

    def updateVisArea(self, visRect):
        self.worlViewWidget.updateVisibleArea(visRect)


class LayoutWindow(QWidget):
    layoutVisibleAreaChanged = pyqtSignal(orp.GLRectangle)

    def __init__(self, parent):
        super(LayoutWindow, self).__init__(parent)

        self.winFrame = QFrame(self)
        self.setWindowTitle("OpenRoad Layout")
        self.layoutViewer = LayoutViewer(self)

        self.vLayout = QVBoxLayout(self)
        self.vLayout.addWidget(self.winFrame)
        self.vLayout.addWidget(self.layoutViewer)

        self.setLayout(self.vLayout)
        self.setupToolBar(self.winFrame)
        self.winFrame.hide()

        self.shortcut_find = QShortcut(QKeySequence('Ctrl+T'), self)
        self.shortcut_find.activated.connect(self.showToolBar)

        self.createFilterActions()

        self.actionfindObject.triggered.connect(self.findObjectInLayout)
        self.actionClose.triggered.connect(lambda: self.winFrame.hide())

        self.zoomInBtn.clicked.connect(
            lambda: self.layoutViewer.applyViewOp(self.layoutViewer.glView.zoomIn))
        self.zoomOutBtn.clicked.connect(
            lambda: self.layoutViewer.applyViewOp(self.layoutViewer.glView.zoomOut))
        self.fitBtn.clicked.connect(
            lambda: self.layoutViewer.applyViewOp(self.layoutViewer.glView.fit))

        self.objTypeToFind = orp.DB_INSTANCE
        self.objViewOp = orp.SELECT_OBJECT

        self.objTypeStrs = ["ft-Instance", "ft-Net", "ft-Pin"]
        self.viewTypeStr = ["ft-Select", "ft-Highlight", "ft-Locate"]
        self.findMapParams = dict(zip(self.objTypeStrs + self.viewTypeStr,
                                      [orp.DB_INSTANCE, orp.DB_NET, orp.DB_PIN,
                                       orp.SELECT_OBJECT, orp.HIGHLIGHT_OBJECT, orp.LOCATE_OBJECT]))

        self.shortcut_worldview = QShortcut(QKeySequence('Ctrl+W'), self)
        self.shortcut_worldview.activated.connect(self.showWorldView)

        self.worldView_ = WorldViewer(self)

        self.worldView_.modifyVisibleArea.connect(self.updateVisibleArea)
        self.layoutVisibleAreaChanged.connect(
            lambda visRect: self.worldView_.updateVisArea(visRect))

    def updateVisibleArea(self, newVisArea):
        self.layoutViewer.glView.setVisibleArea(newVisArea)
        self.layoutViewer.repaint()
        updatedVisArea = self.layoutViewer.glView.getVisibleArea()
        self.layoutVisibleAreaChanged.emit(updatedVisArea)

    def showWorldView(self):
        if self.layoutViewer.canvas == None:
            return
        self.worldView_.showWorldCanvas(
            self.layoutViewer.canvas, self.layoutViewer.getVisibleArea())
        self.worldView_.show()

    def getLayoutPixmap(self):
        return self.layoutViewer.grab()

    def resizeEvent(self, evt):
        self.worldView_.hide()

    def repaintView(self):
        self.layoutViewer.applyViewOp(
            partial(self.layoutViewer.glView.refresh, True))

    def createFilterActions(self):
        self.findCtrlBtn.setPopupMode(QToolButton.InstantPopup)
        self.findPopupMenu = QMenu(parent=self.findCtrlBtn)
        self.findCtrlBtn.setMenu(self.findPopupMenu)

        self.sigMapper = QSignalMapper()

        act1 = QAction(text="Instance", parent=self)
        act1.triggered.connect(self.sigMapper.map)
        act1.setToolTip("Find Instance")
        act1.setCheckable(True)
        act1.setChecked(True)
        self.sigMapper.setMapping(act1, "ft-Instance")

        act2 = QAction(text="Net", parent=self)
        act2.triggered.connect(self.sigMapper.map)
        act2.setToolTip("Find Net")
        act2.setCheckable(True)
        self.sigMapper.setMapping(act2, "ft-Net")

        act3 = QAction(text="Pin", parent=self)
        act3.triggered.connect(self.sigMapper.map)
        act3.setToolTip("Find Pin")
        act3.setCheckable(True)
        self.sigMapper.setMapping(act3, "ft-Pin")

        self.actGroup1 = QActionGroup(self)
        self.actGroup1.addAction(act1)
        self.actGroup1.addAction(act2)
        self.actGroup1.addAction(act3)

        self.findPopupMenu.addAction(act1)
        self.findPopupMenu.addAction(act2)
        self.findPopupMenu.addAction(act3)

        self.findPopupMenu.addSeparator()

        act4 = QAction(text="Select", parent=self)
        act4.triggered.connect(self.sigMapper.map)
        act4.setToolTip("Select Found Object")
        act4.setCheckable(True)
        act4.setChecked(True)
        self.sigMapper.setMapping(act4, "ft-Select")

        act5 = QAction(text="Highlight", parent=self)
        act5.triggered.connect(self.sigMapper.map)
        act5.setToolTip("Highlight Found Object")
        act5.setCheckable(True)
        act5.setChecked(False)
        self.sigMapper.setMapping(act5, "ft-Highlight")

        act6 = QAction(text="Locate", parent=self)
        act6.triggered.connect(self.sigMapper.map)
        act6.setToolTip("Locate Found Object")
        act6.setCheckable(True)
        act6.setChecked(False)
        self.sigMapper.setMapping(act6, "ft-Locate")

        self.actGroup2 = QActionGroup(self)
        self.actGroup2.addAction(act4)
        self.actGroup2.addAction(act5)
        self.actGroup2.addAction(act6)

        self.actGroup2.setExclusive(True)

        self.findPopupMenu.addAction(act4)
        self.findPopupMenu.addAction(act5)
        self.findPopupMenu.addAction(act6)

        self.sigMapper.mapped[str].connect(self.applyFilterOption)

    def showToolBar(self):
        if self.layoutViewer.canvas is not None:
            self.winFrame.show()

    def getLayoutViewer(self):
        return self.layoutViewer

    def findObjectInLayout(self):
        if self.findObjEdit == "":
            return
        objToFind = f"{self.findObjEdit.text()}"
        orIntf = org.OpenRoadGlobals.get_openroad_intf()
        retVal = orIntf.showObjectInView(
            objToFind, self.objTypeToFind, self.objViewOp, self.layoutViewer.glView, True)
        if retVal:
            self.layoutViewer.repaint()
        self.layoutViewer.showSelectedShapeInfo()

    def applyFilterOption(self, mapStr):
        if mapStr in self.objTypeStrs:
            self.objTypeToFind = self.findMapParams[mapStr]
        else:
            self.objViewOp = self.findMapParams[mapStr]

    def setupToolBar(self, Form):
        Form.setObjectName("Form")
        Form.resize(779, 51)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.findObjEdit = QtWidgets.QLineEdit(Form)
        self.findObjEdit.setMinimumSize(QtCore.QSize(100, 0))
        self.findObjEdit.setObjectName("findObjEdit")
        self.horizontalLayout.addWidget(self.findObjEdit)
        self.findCtrlBtn = QtWidgets.QToolButton(Form)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/tool.svg"),
                       QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.findCtrlBtn.setIcon(icon)
        self.findCtrlBtn.setIconSize(QtCore.QSize(24, 24))
        self.findCtrlBtn.setObjectName("findCtrlBtn")
        self.horizontalLayout.addWidget(self.findCtrlBtn)
        self.line = QtWidgets.QFrame(Form)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout.addWidget(self.line)
        self.zoomInBtn = QtWidgets.QToolButton(Form)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/images/zoom-in.svg"),
                        QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.zoomInBtn.setIcon(icon1)
        self.zoomInBtn.setIconSize(QtCore.QSize(24, 24))
        self.zoomInBtn.setObjectName("zoomInBtn")
        self.horizontalLayout.addWidget(self.zoomInBtn)
        self.zoomOutBtn = QtWidgets.QToolButton(Form)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/images/zoom-out.svg"),
                        QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.zoomOutBtn.setIcon(icon2)
        self.zoomOutBtn.setIconSize(QtCore.QSize(24, 24))
        self.zoomOutBtn.setObjectName("zoomOutBtn")
        self.horizontalLayout.addWidget(self.zoomOutBtn)
        self.fitBtn = QtWidgets.QToolButton(Form)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/images/maximize.svg"),
                        QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.fitBtn.setIcon(icon3)
        self.fitBtn.setIconSize(QtCore.QSize(24, 24))
        self.fitBtn.setObjectName("fitBtn")
        self.horizontalLayout.addWidget(self.fitBtn)
        self.line_2 = QtWidgets.QFrame(Form)
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.horizontalLayout.addWidget(self.line_2)
        self.closeBtn = QtWidgets.QToolButton(Form)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(":/images/x.svg"),
                        QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.closeBtn.setIcon(icon4)
        self.closeBtn.setIconSize(QtCore.QSize(24, 24))
        self.closeBtn.setObjectName("closeBtn")
        self.horizontalLayout.addWidget(self.closeBtn)
        self.gridLayout.addLayout(self.horizontalLayout, 0, 0, 1, 1)
        self.actionClose = QtWidgets.QAction(Form)
        self.actionClose.setObjectName("actionClose")
        self.actionfindObject = QtWidgets.QAction(Form)
        self.actionfindObject.setObjectName("actionfindObject")

        self.retranslateUi(Form)
        self.findObjEdit.returnPressed.connect(self.actionfindObject.trigger)
        self.closeBtn.clicked.connect(self.actionClose.trigger)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Find"))
        self.findObjEdit.setPlaceholderText(_translate("Form", "Object Name"))
        self.findCtrlBtn.setToolTip(_translate("Form", "Find Options"))
        self.findCtrlBtn.setText(_translate("Form", "..."))
        self.zoomInBtn.setText(_translate("Form", "..."))
        self.zoomOutBtn.setText(_translate("Form", "..."))
        self.fitBtn.setText(_translate("Form", "..."))
        self.closeBtn.setText(_translate("Form", "..."))
        self.actionClose.setText(_translate("Form", "Close"))
        self.actionClose.setToolTip(_translate("Form", "Close"))
        self.actionfindObject.setText(_translate("Form", "findObject"))
        self.actionfindObject.setToolTip(_translate("Form", "Find Object"))


class LayoutViewer(QtOpenGL.QGLWidget):
    canvasConnected = pyqtSignal(orp.GLCanvas)

    def __init__(self, parent=None):
        super(LayoutViewer, self).__init__(parent)
        #self.parent = parent
        self.setMouseTracking(True)
        self.width = self.width()
        self.height = self.height()
        self.aspect = 1

        self.canvas = None
        self.topLayer = None

        self.mouseClickCoord = None
        self.curMouseCoord = None
        self.motionStarted = False
        self.widgetResized = False

        self.ctrlKeyApplied = False

        self.paintTimerApplied = False
        self.curSelShpIndx = 0

        org.OpenRoadGlobals.openRoadLayoutViewer = self
        #self.canvas = None
        self.glView = orp.GLView.getView("layoutView")

        self.mouseMoveEvent = self.handleMouseMoveEvent
        self.mousePressEvent = self.handleMousePressEvent
        self.mouseReleaseEvent = self.handleMouseReleaseEvent
        self.keyPressEvent = self.handleKeyPressEvent
        self.keyReleaseEvent = self.handleKeyReleaseEvent

    def attachCanvasToLayoutView(self, cnv):
        self.canvas = cnv
        self.topLayer = self.canvas.getTopLayerNode()
        self.glView.setTopCanvas(self.canvas)
        self.parent().worldView_.hide()
        self.canvasConnected.emit(self.canvas)

    def showSmileyCanvas(self):
        self.topLayer = orp.GLLayer.createDummyLayerTree()
        self.canvas = orp.GLCanvas.getDummyCanvas()
        self.glView.setTopCanvas(self.canvas)
        self.canvasConnected.emit(self.canvas)

    def getVisibleArea(self):
        self.makeCurrent()
        visArea = self.glView.getVisibleArea()
        self.doneCurrent()
        return visArea

    def minimumSizeHint(self):
        return QSize(50, 50)

    def sizeHint(self):
        return QSize(400, 400)

    def handleMouseMoveEvent(self, evt):
        self.makeCurrent()
        self.curMouseCoord = orp.GLPoint2D(evt.x(), evt.y())

        if self.glView.getTopCanvas() != None:
            worldLoc = self.glView.getWorldCoord(self.curMouseCoord)
            worldLocStr = worldLoc.repr()
            org.OpenRoadGlobals.get_main_window(
            ).locLabel.setText(f"Loc : {worldLocStr}")

        if self.mouseClickCoord != None and (self.mouseButton == Qt.LeftButton or self.mouseButton == Qt.RightButton):
            curCoord = orp.GLPoint2D(evt.x(), evt.y())
            if curCoord == self.mouseClickCoord:
                return
            elif self.motionStarted == False:
                self.motionStarted = True
                self.glView.stopAnimation()
                self.glView.startAnimationObject(
                    orp.RUBBERBAND_RECT, self.mouseClickCoord)
                self.repaint()
            else:
                self.repaint()
        if self.mouseClickCoord != None and self.mouseButton == Qt.MiddleButton:
            curCoord = orp.GLPoint2D(evt.x(), evt.y())
            if curCoord == self.mouseClickCoord:
                return
            elif self.motionStarted == False:
                self.motionStarted = True
                self.glView.stopAnimation()
                self.glView.startAnimationObject(
                    orp.MOTION_OR_SHAPE, self.mouseClickCoord)
                if self.glView.getMotionObjectCount() == 0:
                    self.motionStarted = False
                    self.glView.stopAnimation()
            else:
                self.repaint()
        org.printDebugMsg('Mouse move {}: [{},{}]'.format(
            evt.button(), evt.x(), evt.y()))

    def handleMousePressEvent(self, evt):
        self.makeCurrent()
        self.setFocus()
        self.setFocusPolicy(Qt.StrongFocus)
        x = evt.x()
        y = evt.y()
        self.mouseClickCoord = orp.GLPoint2D(x, y)
        self.glView.setMouseClickedCoord(self.mouseClickCoord)
        self.motionStarted = False
        self.mouseButton = evt.button()

        org.printDebugMsg('Mouse press {}: [{},{}]'.format(
            evt.button(), evt.x(), evt.y()))

    def showSelectedShapeInfo(self):
        selShpCount = self.glView.getSelectedShapesCount()
        if selShpCount == 0:
            org.OpenRoadGlobals.get_main_window().resetStatusMessage()
            return
        if self.curSelShpIndx >= selShpCount:
            self.curSelShpIndx = 0
        if self.curSelShpIndx < 0:
            self.curSelShpIndx = selShpCount - 1
        shpObj = self.glView.getSelectedShapeAt(self.curSelShpIndx)
        dbId = org.OpenRoadGlobals.get_openroad_intf().dbId
        shpInfo = f"({self.curSelShpIndx + 1}/{selShpCount}) : {shpObj.second.getShapeInfo(shpObj.first, dbId)}"
        org.OpenRoadGlobals.get_main_window().showStatusMessage(shpInfo)

    def handleMouseReleaseEvent(self, evt):
        self.makeCurrent()
        self.mouseButton = None
        mouseReleaseCoord = orp.GLPoint2D(evt.x(), evt.y())
        if evt.button() == Qt.LeftButton:
            if self.mouseClickCoord == mouseReleaseCoord:
                self.glView.stopAnimation()
                self.motionStarted = False
                self.mouseClickCoord = None
                _ = self.glView.selectShapesAt(mouseReleaseCoord, True, True)
                self.curSelShpIndx = 0
                self.repaint()
                self.showSelectedShapeInfo()
                return
            self.glView.setMouseReleaseCoord(mouseReleaseCoord)
            self.glView.stopAnimation()
            self.motionStarted = False
            worldMotionStart = self.glView.getWorldCoord(self.mouseClickCoord)
            worldMotionEnd = self.glView.getWorldCoord(mouseReleaseCoord)
            rectToSelect = orp.GLRectangle(worldMotionStart, worldMotionEnd)
            rectToSelect = rectToSelect.getRectWithFixedOrientation()
            _ = self.glView.selectShapesInRegion(rectToSelect)
            self.mouseClickCoord = None
            self.repaint()
            self.showSelectedShapeInfo()
            self.parent().layoutVisibleAreaChanged.emit(self.glView.getVisibleArea())
            return
        elif evt.button() == Qt.RightButton:
            clickDist = self.mouseClickCoord.distance(mouseReleaseCoord)
            if clickDist < 10.0:
                self.glView.stopAnimation()
                self.motionStarted = False
                self.mouseClickCoord = None
                self.repaint()
                return
            self.glView.setMouseReleaseCoord(mouseReleaseCoord)
            self.glView.stopAnimation()
            self.motionStarted = False
            worldMotionStart = self.glView.getWorldCoord(self.mouseClickCoord)
            worldMotionEnd = self.glView.getWorldCoord(mouseReleaseCoord)
            rectToZoom = orp.GLRectangle(worldMotionStart, worldMotionEnd)
            rectToZoom = rectToZoom.getRectWithFixedOrientation()
            self.glView.zoomRect(rectToZoom)
            self.mouseClickCoord = None
            self.repaint()
            self.parent().layoutVisibleAreaChanged.emit(self.glView.getVisibleArea())
            # Handle Rubberband Rectangle zoomRect here
        elif evt.button() == Qt.MiddleButton:
            if self.mouseClickCoord == mouseReleaseCoord:
                worldMarkerCoord = self.glView.getWorldCoord(
                    self.mouseClickCoord)
                self.glView.addMarkerAt(worldMarkerCoord)
                self.mouseClickCoord = None
            else:
                self.glView.stopAnimation()
                self.motionStarted = False
                self.mouseClickCoord = None
            self.repaint()
        org.printDebugMsg('Mouse release {}: [{},{}]'.format(
            evt.button(), evt.x(), evt.y()))

    def applyViewOp(self, viewOp):
        self.makeCurrent()
        viewOp()
        self.repaint()
        self.parent().layoutVisibleAreaChanged.emit(self.glView.getVisibleArea())

    def handleKeyPressEvent(self, evt):
        if evt.key() == Qt.Key_Control:
            self.ctrlKeyApplied = True
        else:
            self.ctrlKeyApplied = False

    def handleKeyReleaseEvent(self, evt):
        self.ctrlKeyApplied = False
        if DRAW_IN_PYTHON:
            return
        if self.glView.getTopCanvas() == None:
            return
        if evt.key() == Qt.Key_Plus:
            self.applyViewOp(self.glView.zoomIn)
            org.printDebugMsg("Zoom In Operation Performed")
        elif evt.key() == Qt.Key_Minus:
            self.applyViewOp(self.glView.zoomOut)
            org.printDebugMsg("Zoom Out Operation")
        elif evt.key() == Qt.Key_F:
            self.applyViewOp(self.glView.fit)
            org.printDebugMsg("Fit Operation")
        elif evt.key() == Qt.Key_I:
            visArea = self.glView.getVisibleArea()
            print(f"Visible Area of the View = {visArea.repr()}")
        elif evt.key() == Qt.Key_N:
            self.curSelShpIndx += 1
            self.showSelectedShapeInfo()
        elif evt.key() == Qt.Key_P:
            self.curSelShpIndx -= 1
            self.showSelectedShapeInfo()
        elif evt.key() == Qt.Key_R:
            self.applyViewOp(partial(self.glView.refresh, True))
        elif evt.key() == Qt.Key_C:
            viewShapeFlags = 0
            viewShapeFlags = viewShapeFlags | orp.SELECT_VIEW_SHAPE | orp.HIGHLIGHT_VIEW_SHAPE | orp.MARKER_VIEW_SHAPE
            self.applyViewOp(
                partial(self.glView.clearViewShapes, viewShapeFlags))
        elif evt.key() == Qt.Key_Up:
            if self.glView.isViewFit() == False:
                self.applyViewOp(partial(self.glView.panKey, orp.KEY_UP))
        elif evt.key() == Qt.Key_Down:
            if self.glView.isViewFit() == False:
                self.applyViewOp(partial(self.glView.panKey, orp.KEY_DOWN))
        elif evt.key() == Qt.Key_Left:
            if self.glView.isViewFit() == False:
                self.applyViewOp(partial(self.glView.panKey, orp.KEY_LEFT))
        elif evt.key() == Qt.Key_Right:
            if self.glView.isViewFit() == False:
                self.applyViewOp(partial(self.glView.panKey, orp.KEY_RIGHT))
        elif evt.key() == Qt.Key_Less:
            retVal = self.glView.updateMaxViewDepth(False)
            if retVal == True:
                self.repaint()
        elif evt.key() == Qt.Key_Greater:
            retVal = self.glView.updateMaxViewDepth(True)
            if retVal == True:
                self.repaint()
        evt.accept()

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)

        glShadeModel(GL_FLAT)
        glClearDepth(1.0)
        glDepthFunc(GL_ALWAYS)
        glEnable(GL_DEPTH_TEST)

        glReadBuffer(GL_BACK)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)

    def singleShotResize(self):
        pass

    def resizeGL(self, width, height):
        if height == 0:
            self.height = 1
        self.width = width
        self.height = height
        self.aspect = width/height
        glViewport(0, 0, width, height)
        if DRAW_IN_PYTHON == False:
            if self.glView.getTopCanvas() != None:
                self.glView.setViewPortDimensions(width, height)
        if self.glView is not None:
            self.glView.resetPixmaps()
        self.widgetResized = True

    def paintGL(self):
        if DRAW_IN_PYTHON:
            self.render()
            return
        if self.widgetResized == True:
            # Unfortunately gets us into this function twice...
            self.widgetResized = False
            return
        if self.motionStarted == False:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            if self.glView.getTopCanvas() == None:
                return
            self.glView.draw()
            return
        else:
            self.glView.drawMotionObjs(self.curMouseCoord)
        return

    def render(self):
        try:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            # glClear(GL_DEPTH_BUFFER_BIT)
        except BaseException as e:
            org.printDebugMsg(f"Encountered Exception line 124 {str(e)}")
            pass

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.aspect, 0.1, 10.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 2.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)

        glPointSize(5.0)
        glLineWidth(5.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glEnd()
