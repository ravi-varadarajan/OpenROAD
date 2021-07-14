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

from PyQt5.QtCore import QAbstractItemModel, QFile, QIODevice, QModelIndex, Qt
from PyQt5.QtWidgets import QApplication, QTreeView

from . import openroadGlobals as org

import sys
import json
import os.path
from   os import path
import opendbpy as odb
import openroadpy as orp


def loadJsonData(jsonFile):
    if not path.exists(jsonFile):
       print("json file", jsonFile, "not found")
       return None

    print("Reading json file ", jsonFile)
    with open(jsonFile) as f:
         contents = f.read()
    f.close()
    content = ''
    for line in contents:
        content = content + line

    designHierData = None
    try:
        designHierData = json.loads(content)
    except:
        print('Error: Could not read json file')
        return None

    return designHierData


class DesignTreeNode(object):
    def __init__(self, data):
        self._data = data
        if type(data) == tuple:
            self._data = list(data)
        if type(data) is str or not hasattr(data, '__getitem__'):
            self._data = [data]

        self._columncount = len(self._data)
        self._children = []
        self._parent = None
        self._row = 0

    def data(self, column):
        if column >= 0 and column < len(self._data):
            return self._data[column]

    def columnCount(self):
        return self._columncount

    def childCount(self):
        return len(self._children)

    def child(self, row):
        if row >= 0 and row < self.childCount():
            return self._children[row]

    def parent(self):
        return self._parent

    def row(self):
        return self._row

    def addChild(self, child):
        child._parent = self
        child._row = len(self._children)
        self._children.append(child)
        self._columncount = max(child.columnCount(), self._columncount)


class DesignTreeModel(QAbstractItemModel):
    def __init__(self, nodes):
        QAbstractItemModel.__init__(self)
        self.headerData = ["Instance", "Module", "Flat Insts",
                           "Insts", "Flat Macros", "Macros", "Mod Inst"]
        self.buildModel(nodes)

    def populateFromJsonData(self, currParent, hd, level=0):

        for hier_idx in range(len(hd['module_instances'])):
            hier_data = hd['module_instances'][hier_idx]
            nodeData = [hier_data['local_instance_name'], hier_data['module']['module_name'],
                        hier_data['module']['total_insts'], hier_data['module']['local_insts'],
                        hier_data['module']['total_macros'], hier_data['module']['local_macros'],
                        len(hier_data['module']['module_instances'])]
            newNode = DesignTreeNode(nodeData)
            currParent.addChild(newNode)

            self.populateFromJsonData(newNode, hier_data['module'], level+1)

    def buildModel(self, nodes=None):
        ordb = org.OpenRoadGlobals.get_openroad_db()
        if ordb == None:
           self.beginResetModel()
           self._root = DesignTreeNode(None)
           self.endResetModel()
           return

        block = ordb.getChip().getBlock()
        blockName = block.getName()
        jsonFile = blockName + ".json"
        designHierData = loadJsonData(jsonFile)

        jsonOk = True
        
        if designHierData != None:
            try:
                keys = designHierData.keys()
                if not 'logical_hierarchy' in keys:
                    raise
                keys = designHierData['logical_hierarchy'].keys()
                if not 'module_name' in keys:
                    raise
                if not 'total_insts' in keys:
                    raise
                if not 'local_insts' in keys:
                    raise
                if not 'total_macros' in keys:
                    raise
                if not 'local_macros' in keys:
                    raise
                if not 'module_instances' in keys:
                    raise
            except:
                print("Error: Json format not compatible with current version")
                jsonOk = False
        else:
            jsonOk = False

        if jsonOk == False:
           return

        self.beginResetModel()
        self._root = DesignTreeNode(None)
        topLevelInfo = [designHierData['logical_hierarchy']['module_name'],
                        str(designHierData['logical_hierarchy']
                            ['module_name']),
                        str(designHierData['logical_hierarchy']
                            ['total_insts']),
                        str(designHierData['logical_hierarchy']
                            ['local_insts']),
                        str(designHierData['logical_hierarchy']
                            ['total_macros']),
                        str(designHierData['logical_hierarchy']
                            ['local_macros']),
                        str(len(designHierData['logical_hierarchy']['module_instances']))]

        topNode = DesignTreeNode(topLevelInfo)
        self._root.addChild(topNode)
        self.populateFromJsonData(topNode, designHierData['logical_hierarchy'])

        self.endResetModel()

    def rowCount(self, index):
        if index.isValid():
            return index.internalPointer().childCount()
        return self._root.childCount()

    def addChild(self, node, _parent):
        if not _parent or not _parent.isValid():
            parent = self._root
        else:
            parent = _parent.internalPointer()
        parent.addChild(node)

    def index(self, row, column, _parent=None):
        if not _parent or not _parent.isValid():
            parent = self._root
        else:
            parent = _parent.internalPointer()

        if not QAbstractItemModel.hasIndex(self, row, column, _parent):
            return QModelIndex()

        child = parent.child(row)
        if child:
            return QAbstractItemModel.createIndex(self, row, column, child)
        else:
            return QModelIndex()

    def parent(self, index):
        if index.isValid():
            p = index.internalPointer().parent()
            if p:
                return QAbstractItemModel.createIndex(self, p.row(), 0, p)
        return QModelIndex()

    def columnCount(self, index):
        if index.isValid():
            return index.internalPointer().columnCount()
        return self._root.columnCount()

    def data(self, index, role):
        if not index.isValid():
            return None
        node = index.internalPointer()
        if role == Qt.DisplayRole:
            return node.data(index.column())
        return None

    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.headerData[section]

        return None


class HierTreeItem(object):
    def __init__(self, data, parent=None):
        self.parentItem = parent
        self.itemData = data
        self.childItems = []

    def appendChild(self, item):
        self.childItems.append(item)

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return len(self.itemData)

    def data(self, column):
        try:
            return self.itemData[column]
        except IndexError:
            return None

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem:
            return self.parentItem.childItems.index(self)

        return 0


class HierTreeModel(QAbstractItemModel):
    def __init__(self, data, parent=None):
        super(HierTreeModel, self).__init__(parent)

        self.rootItem = HierTreeItem(("Title", "Summary"))
        self.setupModelData(data.split('\n'), self.rootItem)

    def columnCount(self, parent):
        if parent.isValid():
            return parent.internalPointer().columnCount()
        else:
            return self.rootItem.columnCount()

    def data(self, index, role):
        if not index.isValid():
            return None

        if role != Qt.DisplayRole:
            return None

        item = index.internalPointer()

        return item.data(index.column())

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags

        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.rootItem.data(section)

        return None

    def index(self, row, column, parent):
        if not self.hasIndex(row, column, parent):
            return QModelIndex()

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QModelIndex()

    def parent(self, index):
        if not index.isValid():
            return QModelIndex()

        childItem = index.internalPointer()
        parentItem = childItem.parent()

        if parentItem == self.rootItem:
            return QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

    def rowCount(self, parent):
        if parent.column() > 0:
            return 0

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()

    def setupModelData(self, lines, parent):
        parents = [parent]
        indentations = [0]

        number = 0

        while number < len(lines):
            position = 0
            while position < len(lines[number]):
                if lines[number][position] != ' ':
                    break
                position += 1

            lineData = lines[number][position:].trimmed()

            if lineData:
                # Read the column data from the rest of the line.
                columnData = [s for s in lineData.split('\t') if s]

                if position > indentations[-1]:
                    # The last child of the current parent is now the new
                    # parent unless the current parent has no children.

                    if parents[-1].childCount() > 0:
                        parents.append(
                            parents[-1].child(parents[-1].childCount() - 1))
                        indentations.append(position)

                else:
                    while position < indentations[-1] and len(parents) > 0:
                        parents.pop()
                        indentations.pop()

                # Append a new item to the current parent's list of children.
                parents[-1].appendChild(HierTreeItem(columnData, parents[-1]))

            number += 1
