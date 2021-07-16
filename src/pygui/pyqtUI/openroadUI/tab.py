from PyQt5 import QtCore, QtGui, QtWidgets
import sys, os
from . import tablewidget
from . import PandasModel
from . import parser
import argparse
import pandas as pd

parse = argparse.ArgumentParser()

parse.add_argument('--ipath', type=str, default='report.txt')
parse.add_argument('--opath', type=str, default='out')

args = parse.parse_args()

class TableWidget1(tablewidget.TableWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.child_widget = tablewidget.TableWidget()
    
    def loadFile(self):
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open File", "", "Report Files (*.*)");
        try:
            os.mkdir(args.opath)
        except:
            pass
        parser.write_files(fileName, args.opath)
        fileName = os.path.join(args.opath, os.path.basename(fileName).split('.')[0] + "_" + "info" + '.csv')
        self.pathLE.setText(fileName)
        df = pd.read_csv(fileName)
        df = df.fillna('')
        model = PandasModel.PandasModel(df)
        self.pandasTv.setModel(model)

    def getSelectedIndexes(self):
        return self.pandasTv.selectedIndexes()



class TabViewer(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.layout = QtWidgets.QGridLayout()
        self.setLayout(self.layout)

        self.label1 = TableWidget1()

        self.tabwidget = QtWidgets.QTabWidget()
        self.tabwidget.addTab(self.label1, "Report Summary")
        self.tabwidget.addTab(self.label1.child_widget, "Details")
        self.tabwidget.mouseDoubleClickEvent = self.mousePressEvent
        self.layout.addWidget(self.tabwidget, 0, 0)

    def mousePressEvent(self, evt):
        print(self.label1.getSelectedIndexes())

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    screen = TabViewer()
    screen.show()
    sys.exit(app.exec_())