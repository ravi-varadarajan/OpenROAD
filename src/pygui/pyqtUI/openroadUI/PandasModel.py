from PyQt5 import QtCore, QtGui

import pandas as pd

class PandasModel(QtCore.QAbstractTableModel): 
    def __init__(self, df = pd.DataFrame(), parent=None): 
        QtCore.QAbstractTableModel.__init__(self, parent=parent)
        self._df = df

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role != QtCore.Qt.DisplayRole:
            return QtCore.QVariant()

        if orientation == QtCore.Qt.Horizontal:
            try:
                return self._df.columns.tolist()[section]
            except (IndexError, ):
                return QtCore.QVariant()
        elif orientation == QtCore.Qt.Vertical:
            try:
                # return self.df.index.tolist()
                return self._df.index.tolist()[section]
            except (IndexError, ):
                return QtCore.QVariant()

    def data(self, index, role=QtCore.Qt.DisplayRole):

        if role == QtCore.Qt.TextAlignmentRole:
            return QtCore.Qt.AlignCenter

        COLORS = ['#053061', '#2166ac', '#4393c3', '#92c5de', '#d1e5f0', '#f7f7f7', '#fddbc7', '#f4a582', '#d6604d', '#b2182b', '#67001f']

        if role == QtCore.Qt.BackgroundRole:
            value = self._df.iloc[index.row(), index.column()]
            if (isinstance(value, int) or isinstance(value, float)):
                try:
                    value = int(value)
                except:
                    value = 0
                while(value > 10 or value < -10):
                    value//=10
                value = max(-5, value)
                value = min(5, value)
                value = value + 5
                return QtGui.QColor(COLORS[value])


        if role != QtCore.Qt.DisplayRole and role != QtCore.Qt.EditRole:
            return QtCore.QVariant()

        if not index.isValid():
            return QtCore.QVariant()

        return QtCore.QVariant(str(self._df.iloc[index.row(), index.column()]))

    def setData(self, index, value, role):

        if(role==QtCore.Qt.EditRole):

            row = self._df.index[index.row()]
            col = self._df.columns[index.column()]
            if hasattr(value, 'toPyObject'):
                # PyQt4 gets a QVariant
                value = value.toPyObject()
            else:
                # PySide gets an unicode
                dtype = self._df[col].dtype
                if dtype != object:
                    value = None if value == '' else dtype.type(value)
            self._df.set_value(row, col, value)
            return True
        else:
            return False

    def flags(self, index):
        return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

    def rowCount(self, parent=QtCore.QModelIndex()): 
        return len(self._df.index)

    def columnCount(self, parent=QtCore.QModelIndex()): 
        return len(self._df.columns)

    def sort(self, column, order):
        colname = self._df.columns.tolist()[column]
        self.layoutAboutToBeChanged.emit()
        self._df.sort_values(colname, ascending= order == QtCore.Qt.AscendingOrder, inplace=True)
        self._df.reset_index(inplace=True, drop=True)
        self.layoutChanged.emit()
