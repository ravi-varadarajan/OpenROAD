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

import os
import sys
import opendbpy as odb
import openroadpy as orp
from . import openroad_rc
from . import openroadGlobals as org
from . import openRoadTclBridge as ortcl

class PythonOpenRoadIntf(orp.OpenRoadPythonIntf):
    def __init__(self):
        super().__init__()
        self.dbId = -1
        org.OpenRoadGlobals.openRoadPyIntf = self

    def printMessage(self, msg):
        print(f"PyGui> Printing Message : {msg} in Python Got From C++ ")
        sys.stdout.flush()

    def setDatabaseId(self, dbId):
        self.dbId = dbId
        #print(f"Wow Got the DB Id : {self.dbId}")
        or_db = odb.dbDatabase.getDatabase(self.dbId)
        org.OpenRoadGlobals.openRoadDbHandle = or_db

        org.printDebugMsg(
            f"PyGui> Number of masters in the database = {or_db.getNumberOfMasters()}")
        org.OpenRoadGlobals.get_main_window().populateHierTree()

        lv_ = org.OpenRoadGlobals.get_layout_viewer()
        cnv = orp.GLCanvas.getCanvas("OR")
        if not cnv == None and cnv.getShapeCountFromAllLayers() != 0 and lv_ is not None:
            # print(f"Setting Database Id and the canvas contains {cnv.getShapeCountFromAllLayers()} \
            #      and the Bounding Box {cnv.getFullBBoxPy().repr()} ")
            sys.stdout.flush()
            lv_.glView.setTopCanvas(cnv)
            lv_.attachCanvasToLayoutView(cnv)
            # lv_.glView.updateMaxViewDepth()
            lv_.repaint()
        else:
            pass
            #print(f"Database Id {dbId} is set but the Canvas is Null")
            # sys.stdout.flush()
    def setTclEvalState(self, tclRunState, tclStatusMsg, channelOut):
        scriptW = org.OpenRoadGlobals.get_main_window().getScriptWidget()
        tclResObj = orp.openRoadTclRes()

        tclResObj.tclRes = tclStatusMsg
        tclResObj.tclRetVal = tclRunState

        scriptW.updateOutput(tclResObj, channelOut=channelOut)

    def redrawLayoutView(self):
        topCnv = orp.GLCanvas.getCanvas("OR")
        lv_ = org.OpenRoadGlobals.get_layout_viewer()
        if topCnv == None:
            #print("Canvas is Null can not be redrawn...")
            return
        elif topCnv.getShapeCountFromAllLayers() == 0:
            #print(f"Repopulating Canvas as it is empty")
            self.updateCanvas(topCnv)
            if topCnv.getShapeCountFromAllLayers() != 0:
                lv_.glView.setTopCanvas(topCnv)
                lv_.attachCanvasToLayoutView(topCnv)
                mainWin = org.OpenRoadGlobals.get_main_window()
                mainWin.updateViews(topCnv)
                lv_.repaint()
                return
        # print(f"Came to redraw Layout View in Python Number of shapes in the canvas = \
        #       {topCnv.getShapeCountFromAllLayers()}")
        sys.stdout.flush()
        lv_.glView.resetPixmaps()
        lv_.repaint()

    def pause(self):
        scriptW = org.OpenRoadGlobals.get_main_window().getScriptWidget()
        scriptW.pause()

        print(
            f"Returning from pause in Python with process id = {os.getpid()}")
        sys.stdout.flush()

    def zoomTo(self, llx, lly, urx, ury):
        glRect = orp.GLRectangel(llx, lly, urx, ury)
        print(f"Came to zoomTo In Python with rect {glrect.repr()}")
        sys.stdout.flush()

    def showStatusMessage(self, message):
        print(f"Came to show status message {message} in Python...")
        sys.stdout.flush()


def init():
    org.OpenRoadGlobals.openRoadIntf = PythonOpenRoadIntf()
    org.OpenRoadGlobals.openRoadTclBridge = ortcl.TclInterpBridge()
    print("Py interface built!!")
    sys.stdout.flush()

