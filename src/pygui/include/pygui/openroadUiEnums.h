/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, OpenROAD
// All rights reserved.
//
// BSD 3-Clause License
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
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

namespace OpenRoadUI {
enum ORTransformType
{
  OR_VIEW_TRANSFORM = 0,
  OR_CANVAS_TRANSFORM,
  OR_CELL_TRANSFORM
};

enum TxOpType
{
  UNKNOWN_TX = 0,
  ROTATION_TX,
  SCALE_TX,
  TRANSLATION_TX
};

enum ORShapeType
{
  OR_ABSTRACT_SHAPE = 0,
  // Canvas Shape
  OR_DB_SHAPE,
  OR_RECT_SHAPE,
  OR_SEGMENT_SHAPE,
  OR_POINT_SHAPE,
  OR_CANVASINST_SHAPE,
  OR_COMPOSITE_SHAPE,
  OR_MARKER_SHAPE
};

enum ORMarkerType
{
  OR_DIAMOND_MARKER = 0,
  OR_PLUS_MARKER,
  OR_CROSS_MARKER,
  OR_CIRCLE_MARKER
};

enum ORPenPatternType
{
  OR_FILL_NONE_PAT = 0,
  OR_FILL_SOLID_PAT,

  OR_LEFT_DIAG_PAT,
  OR_LEFT_DIAG_SPARSE_PAT,

  OR_RIGHT_DIAG_PAT,
  OR_RIGHT_DIAG_SPARSE_PAT,

  OR_HORIZONTAL_PAT,
  OR_HORIZONTAL_SPARSE_PAT,

  OR_VERTICAL_PAT,
  OR_VERTICAL_SPARSE_PAT,

  OR_CROSS_PAT,

  OR_CHECK_PAT,
  OR_BLOCK_PAT,

  OR_END_PAT
};

enum ORLayerType
{
  DUMMY_LAYER_0 = 0,
  DUMMY_LAYER_1,
  DUMMY_LAYER_2,
  DUMMY_LAYER_3,
  DUMMY_LAYER_4,
  DUMMY_LAYER_5,
  DUMMY_LAYER_6,

  BACKGROUND_LAYER,

  DESIGN_LAYER = 500,
  DESIGN_INSTANCE_LAYER,
  DESIGN_CELLTYPE_LAYER,
  DESIGN_STD_CELL_LAYER,
  DESIGN_BLACKBOX_LAYER,
  DESIGN_COVERCELL_LAYER,
  DESIGN_PHYSICAL_CELL_LAYER,
  DESIGN_FILLER_CELL_LAYER,
  DESIGN_MACRO_LAYER,
  DESIGN_IO_CELL_LAYER,
  DESIGN_IO_LAYER,

  METAL_LAYER,
  VIA_LAYER,

  WORLD_VIEW_LAYER
};

enum DrawViewShapeType
{
  SELECT_VIEW_SHAPE = 0x00001,
  HIGHLIGHT_VIEW_SHAPE = 0x00002,
  MARKER_VIEW_SHAPE = 0x00004,
  CANVAS_SHAPE = 0x00008,

  DRAW_MOTION_SHAPES = 0x00010  // While Motion Shapes are being drawn other
                                // shapes will not be drawn
};

enum DrawMotionShapeType
{
  RUBBERBAND_RECT = 0,
  RUBBERBAND_LINE,
  MOTION_OR_SHAPE
};

enum ArrowKeyType
{
  KEY_LEFT = 0,
  KEY_RIGHT,
  KEY_UP,
  KEY_DOWN
};

enum DBObjectType
{
  DB_INSTANCE = 0,
  DB_NET,
  DB_PIN
};

enum ViewOpType
{
  SELECT_OBJECT = 0,
  HIGHLIGHT_OBJECT,
  LOCATE_OBJECT
};
}  // namespace OpenRoadUI
