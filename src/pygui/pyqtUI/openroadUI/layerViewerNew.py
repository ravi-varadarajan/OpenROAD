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
import sys
import os
import math
from collections import deque
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (QAbstractItemView, QStyle, QPushButton, QRadioButton, QToolButton, QCheckBox, 
                             QItemDelegate, QStyledItemDelegate, QAbstractItemDelegate,
                             QGroupBox, QDialogButtonBox, QHBoxLayout, QVBoxLayout, QGridLayout, QWidget, QDialog, QColorDialog, QTreeView)
from PyQt5.QtCore import (Qt, QModelIndex, pyqtSignal, QPointF, QRectF, QMargins, QSize, QAbstractItemModel)
from PyQt5.QtGui import (QPainter, QPolygonF, QColor, QBrush)

import openroadpy as orp
from . import openroadGlobals as org

class PatternButton(QRadioButton) :
    def __init__(self, patType, parent=None):
        super(PatternButton, self).__init__("      ", parent)
        self.patType_ = patType
        self.setFixedWidth(100)

    def paintEvent(self, pEvent) :
        super(PatternButton, self).paintEvent(pEvent)
        qp = QPainter(self)
        brush = QBrush(QColor("black"), self.patType_)
        qp.setBrush(brush)
        btnRect = self.rect()
        btnRect.adjust(18, 2, -1, -1)
        qp.drawRect(btnRect)
        qp.end()

class LayerColorDialog(QDialog):
    def __init__(self, layerColor, layerPat, parent=None):
        super(LayerColorDialog, self).__init__(parent)
        self.layerColor_   = layerColor
        self.layerPattern_ = layerPat

        self.colorDialog = QColorDialog()
        self.colorDialog.setOptions(QColorDialog.DontUseNativeDialog | QColorDialog.NoButtons)
        self.colorDialog.setWindowFlags(Qt.Widget)
        self.colorDialog.setCurrentColor(self.layerColor_)
        self.colorDialog.currentColorChanged.connect(self.storeChangedColor)
        
        self.patGroupBox = QGroupBox("Layer Pattern")
        gLayout  = QGridLayout()
        gLayout.setColumnStretch(1, 4)
        gLayout.setColumnStretch(2, 4)

        self.patterns = [[Qt.NoBrush, Qt.SolidPattern], 
                         [Qt.HorPattern, Qt.VerPattern], 
                         [Qt.CrossPattern, Qt.DiagCrossPattern], 
                         [Qt.FDiagPattern, Qt.BDiagPattern]]
        self.patButtons = []
        for rIdx, patGrp in enumerate(self.patterns) :
            for cIdx, pat in enumerate(patGrp) :
                pb = PatternButton(pat)
                self.patButtons.append((pb, (rIdx, cIdx)))
                gLayout.addWidget(pb, rIdx, cIdx)
                if pat == self.layerPattern_ :
                    pb.setChecked(True)
                else:
                    pb.setChecked(False)

        self.patGroupBox.setLayout(gLayout)

        btnBox = QDialogButtonBox(self)
        btnBox.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btnBox.accepted.connect(self.colorAccepted)
        btnBox.rejected.connect(self.closeDialog)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.patGroupBox)
        mainLayout.addWidget(self.colorDialog)
        mainLayout.addWidget(btnBox)

        self.setLayout(mainLayout)
        self.setWindowTitle("Layer Config")
        self.setFixedSize(600, self.width()-20)

    def storeChangedColor(self, col) :
        self.layerColor_ = col

    def selectedColor(self) :
        return self.layerColor_

    def selectedPattern(self) :
        patIdx = (0, 0)
        try :
            patIdx = next(v[1] for v in self.patButtons if v[0].isChecked())
        except :
            pass
        return self.patterns[patIdx[0]][patIdx[1]]

    def colorAccepted(self) :
        self.accept()

    def closeDialog(self):
        self.reject()

class RectDelegate(QStyledItemDelegate):
    def __init__(self, layerNode=None, parent=None) :
        super(RectDelegate, self).__init__(parent)
        self.layerNode_ = layerNode

    def paint(self, painter, option, index):
        layerItem = index.data()
        if layerItem.getChildCount() == 0 :
            layerItem.paint(painter, option.rect, option.palette)
        else :
            super(RectDelegate, self).paint(painter, option, index)

    def sizeHint(self, option, index):
        layerItem = index.data()
        if isinstance(layerItem, ORLayerItem) and layerItem.getChildCount() == 0:
            return layerItem.sizeHint()
        else:
            return super(RectDelegate, self).sizeHint(option, index)

    def createEditor(self, parent, option, index):
        layerItem = index.data()
        if layerItem.getChildCount() == 0 :
            return LayerColorDialog(layerItem.qtColor_, layerItem.qtBrushPat_)
        else :
            return None

    def setModelData(self, editor, model, index):
        layerItem = index.data()
        if layerItem.getChildCount() == 0 :
            selColor = editor.selectedColor()
            selPattern = editor.selectedPattern()
            layerItem.setColorPattern(selColor, selPattern)
        else :
            return super(RectDelegate, self).setModelData(editor, model, index)

class ORLayerItem(object) :
    PaintingRectScaleFactor = 20
    qtPatterns = [Qt.NoBrush, Qt.SolidPattern, Qt.HorPattern, Qt.VerPattern, 
                  Qt.CrossPattern, Qt.DiagCrossPattern, Qt.FDiagPattern, Qt.BDiagPattern]
    orPatterns = [orp.OR_FILL_NONE_PAT, orp.OR_FILL_SOLID_PAT, orp.OR_HORIZONTAL_PAT,
                  orp.OR_VERTICAL_PAT, orp.OR_CHECK_PAT, orp.OR_CROSS_PAT, 
                  orp.OR_RIGHT_DIAG_PAT, orp.OR_LEFT_DIAG_PAT]
    qtOrPatDict = dict(zip(qtPatterns, orPatterns))
    orQtPatDict = dict(zip(orPatterns, qtPatterns))
    def __init__(self, layerViewer, orLayer, rowNum, parent) :
        self.layerParent_   = parent
        self.openRoadLayer_ = orLayer
        self.rowNum_        = rowNum
        self.layerViewer_   = layerViewer

        layerPen = None
        if self.openRoadLayer_ :
            layerPen = self.openRoadLayer_.getLayerPen()

        r = 255
        g = 0
        b = 0
        a = 1

        self.qtBrushPat_ = ORLayerItem.qtPatterns[0]
        if layerPen is not None :
            cVals = layerPen.getPenColorVals()
            r = math.ceil(cVals.red()   * 255)
            g = math.ceil(cVals.green() * 255)
            b = math.ceil(cVals.blue()  * 255)
            a = math.ceil(cVals.alpha() * 255)
            if layerPen.getPenPattern() in ORLayerItem.orQtPatDict :
                self.qtBrushPat_ = ORLayerItem.orQtPatDict[layerPen.getPenPattern()]

        self.qtColor_    = QColor(r, g, b, a)

        self.childLayers_   = []

    def sizeHint(self):
        return self.PaintingRectScaleFactor * QSize(4, 1)

    def paint(self, painter, rect, palette):
        painter.save()

        brush = QBrush(self.qtColor_, self.qtBrushPat_)
        painter.setBrush(brush)
        rect.adjust(2, 2, -2, -2)
        painter.drawRect(rect)
        painter.restore()
        return 

    def setColorPattern(self, qtColor, qtPat) :
        self.qtColor_ = qtColor
        self.qtBrushPat_ = qtPat

        cParam = orp.ColorParams(qtColor.redF(), qtColor.greenF(), qtColor.blueF(), 1.0)
        orPat = ORLayerItem.qtOrPatDict[qtPat]

        self.openRoadLayer_.getLayerPen().setPenColorVals(cParam)
        self.openRoadLayer_.getLayerPen().setPenPattern(orPat)
        self.layerViewer_.layerChanged(self.openRoadLayer_, 1)

    def itemName(self) :
        if self.openRoadLayer_ is None :
            return "Empty Layer Node"
        return self.openRoadLayer_.getLayerName()

    def getLayerName(self) :
        return self.itemName()

    def node(self) :
        return self.openRoadLayer_

    def data(self, colIdx) :
        if colIdx == 0 :
            return self.openRoadLayer_.getLayerName()
        elif colIdx == 1 :
            return self.getLayerName()
        elif colIdx == 2 :
            return self.openRoadLayer_.isLayerVisible()
        return self.openRoadLayer_.isLayerSelectable()

    def parent(self) :
        return self.layerParent_

    def child(self, i) :
        return self.childLayers_[i]

    def getChildCount(self) :
        return len(self.childLayers_)

    def row(self) :
        return self.rowNum_

    def addChildLayerNodes(self) :
        if self.openRoadLayer_ is None :
            return
        numChildren = self.openRoadLayer_.getChildCount()
        for cIdx in range(numChildren) :
            childNode   = self.openRoadLayer_.getChildLayerAt(cIdx)
            layerItem = ORLayerItem(self.layerViewer_, childNode, cIdx, self)
            self.childLayers_.append(layerItem)
            layerItem.addChildLayerNodes()

class ORLayerModel(QAbstractItemModel):
    columnNames = ["Layer", "Color", "V", "S"]
    def __init__(self, layerViewer, parent=None) :
        super(ORLayerModel, self).__init__(parent)
        self.layerViewer_  = layerViewer

        self.topLayerNode_ = None
        self.rootNode_ = ORLayerItem(self.layerViewer_, None, 0, None)
        self.rootNode_.addChildLayerNodes()

    def getRootNode(self) :
        return self.rootNode_

    def setupModelData(self, topLayerNode) :
        self.topLayerNode_ = topLayerNode
        firstChild = ORLayerItem(self.layerViewer_, self.topLayerNode_, 0, self.rootNode_)
        firstChild.addChildLayerNodes()
        self.rootNode_.childLayers_ = []
        self.rootNode_.childLayers_.append(firstChild)

    def columnCount(self, parent) :
        return len(ORLayerModel.columnNames)

    def flags(self, index):
        if not index.isValid():
            return None

        itemFlags = Qt.ItemIsEnabled | Qt.ItemIsSelectable 
        if index.column() > 1:
            itemFlags = Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable | Qt.ItemIsEditable 
        elif index.column() == 1 :
            itemFlags = Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable 

        return itemFlags

    def setData(self, index, value, role) :
        if index.column() > 1 :
            if role == Qt.CheckStateRole :
                layerItem = index.internalPointer()
                if layerItem == None :
                    return False
                if index.column() == 2 :
                    layerItem.openRoadLayer_.setLayerVisible(True if value == Qt.Checked else False)
                    self.dataChanged.emit(self.index(0, 2, index.parent()), self.index(1, 2, index.parent()))
                    self.layerViewer_.layerChanged(layerItem.openRoadLayer_, index.column())
                    return True
                elif index.column() == 3 :
                    layerItem.openRoadLayer_.setLayerSelectable(True if value == Qt.Checked else False)
                    self.dataChanged.emit(self.index(0, 2, index.parent()), self.index(1, 2, index.parent()))
                    self.layerViewer_.layerChanged(layerItem.openRoadLayer_, index.column())
                    return True
        elif index.column() == 1 :
            layerItem = index.internalPointer()
            self.dataChanged.emit(index, index) #Delegate has changed the data in the openroad Layer
            self.layerViewer_.layerChanged(layerItem.openRoadLayer_, index.column())
            return True
        return super(ORLayerModel, self).setData(index, value, role)

    def data(self, index, role) :
        if not index.isValid() :
            return None

        if role == Qt.CheckStateRole and index.column() <= 1 :
            return None

        layerItem = index.internalPointer()

        if role == Qt.CheckStateRole and index.column() > 1 :
            return Qt.Checked if layerItem.data(index.column()) else Qt.Unchecked
        if role == Qt.DisplayRole and index.column() == 0:
            return layerItem.itemName()
        if role == Qt.DisplayRole and index.column() == 1 :
            return layerItem
        if role == Qt.TextAlignmentRole and index.column() == 0 :
            return Qt.AlignLeft
        #return layerItem.data(index.column())

    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            if section == 0:
                return "Layer"
            if section == 1:
                return "Color"
            if section == 2:
                return "V"
            if section == 3:
                return "S"
        return None

    def index(self, row, column, parent):
        if not self.hasIndex(row, column, parent) :
            return QModelIndex()

        if not parent.isValid():
            parentItem = self.rootNode_
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QModelIndex()

    def parent(self, child):
        if not child.isValid():
            return QModelIndex()

        childItem = child.internalPointer()
        parentItem = childItem.parent()

        if not parentItem or parentItem == self.rootNode_:
            return QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

    def rowCount(self, parent):
        if parent.column() > 0 :
            return 0
        parentItem = None
        if not parent.isValid():
            parentItem = self.rootNode_
        else:
            parentItem = parent.internalPointer()
        return parentItem.getChildCount()

class LayerViewer(QWidget) :
    layerUpdated = pyqtSignal(orp.GLLayer, org.LayerDataType)
    def __init__(self, parent) :
        super(LayerViewer, self).__init__(parent=parent)

        self.tree_ = QTreeView(self.parent())
        layout     = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.tree_)
        self.tree_.setAlternatingRowColors(True)
        self.layerModel_ = ORLayerModel(self, None)
        self.tree_.setModel(self.layerModel_)

        self.tree_.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.SelectedClicked)
        
        self.rectDelegate = RectDelegate(self.layerModel_.getRootNode(), self.tree_)
        self.tree_.setItemDelegateForColumn(1, self.rectDelegate)

    def populateLayerModel(self, topLayerNode) :
        self.layerModel_ = ORLayerModel(self, None)
        self.tree_.setModel(self.layerModel_)

        self.layerModel_.beginResetModel()
        self.layerModel_.setupModelData(topLayerNode)
        self.layerModel_.endResetModel()

        self.tree_.expandAll()
        for colIdx in range(4) :
            self.tree_.resizeColumnToContents(colIdx)

    def layerChanged(self, orLayer, colIdx) :
        if colIdx == 1 :
            self.layerUpdated.emit(orLayer, org.LayerDataType.COLOR_OR_PAT)
        elif colIdx == 2 :
            self.layerUpdated.emit(orLayer, org.LayerDataType.VISIBILITY)
        elif colIdx == 3 :
            self.layerUpdated.emit(orLayer, org.LayerDataType.SELECTABILITY)
        else :
            print(f"Column {colIdx} of LayerItem {orLayer.getLayerName()} is changed")
        #print("LayerViewer Received layerChanged Signal")


