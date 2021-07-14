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
from enum import Enum

SHOW_DEBUG_MSG = 'SHOW_DEBUG_MSG' in os.environ

def printDebugMsg(msg) :
    if SHOW_DEBUG_MSG :
        print(msg)

from enum import Enum

class LayerDataType(Enum) :
    COLOR_OR_PAT  = 1
    VISIBILITY    = 2
    SELECTABILITY = 3

class OpenRoadGlobals(object):
    openRoadIntf         = None
    openRoadTclBridge    = None
    openRoadMainWin      = None

    openRoadDbHandle     = None
    openRoadLayoutViewer = None

    globalMap         = {}

    @classmethod
    def get_openroad_intf(cls) :
        return cls.openRoadIntf

    @classmethod
    def get_tcl_bridge(cls) :
        return cls.openRoadTclBridge

    @classmethod
    def get_main_window(cls):
        return cls.openRoadMainWin

    @classmethod
    def get_openroad_db(cls):
        return cls.openRoadDbHandle

    @classmethod
    def get_layout_viewer(cls):
        return cls.openRoadLayoutViewer