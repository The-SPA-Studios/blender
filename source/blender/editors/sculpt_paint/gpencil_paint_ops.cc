/* SPDX-License-Identifier: GPL-2.0-or-later. */

#include "BKE_gpencil_update_cache.h"
#include "BKE_object.h"
#include "BKE_report.h"

#include "BLI_listbase.h"

#include "ED_screen.h"

#include "DNA_material_types.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_types.h"

#include "WM_api.h"
#include "WM_types.h"

#include "gpencil_paint_intern.hh"
#include "paint_intern.h"

namespace blender::ed::sculpt_paint::gpencil {

static bool gpencil_paint_poll(bContext *C)
{
  if (!ED_operator_regionactive(C)) {
    CTX_wm_operator_poll_msg_set(C, "Active region not set");
    return false;
  }

  ScrArea *area = CTX_wm_area(C);
  /* 3D Viewport. */
  if (area->spacetype != SPACE_VIEW3D) {
    return false;
  }

  /* Check if Grease Pencil isn't already running. */
  if (ED_gpencil_session_active() != 0) {
    CTX_wm_operator_poll_msg_set(C, "Grease Pencil operator is already active");
    return false;
  }

  /* Only grease pencil object type. */
  bGPdata *gpd = CTX_data_gpencil_data(C);
  if (gpd == nullptr || !GPENCIL_PAINT_MODE(gpd)) {
    return false;
  }

  ToolSettings *ts = CTX_data_scene(C)->toolsettings;
  if (!ts->gp_paint->paint.brush) {
    CTX_wm_operator_poll_msg_set(C, "Grease Pencil has no active paint tool");
    return false;
  }

  return true;
}

static bool gpencil_start_paint_operation(bContext *C,
                                          PaintStroke *paint_stroke,
                                          const float initial_mouse_location[2])
{
  GPencilPaintOperation *gpop = nullptr;
  Scene *scene = CTX_data_scene(C);
  GpPaint *gpaint = scene->toolsettings->gp_paint;
  Brush *brush = BKE_paint_brush(&gpaint->paint);

  switch (brush->gpencil_tool) {
    case GPAINT_TOOL_DRAW:
      gpop = new_gpencil_draw_operation(C, initial_mouse_location);
      break;
    case GPAINT_TOOL_FILL:
      BLI_assert_unreachable();
      break;
    case GPAINT_TOOL_ERASE:
      BLI_assert_unreachable();
      break;
    case GPAINT_TOOL_TINT:
      BLI_assert_unreachable();
      break;
    default:
      BLI_assert_unreachable();
  }

  if (gpop) {
    paint_stroke_set_mode_data(paint_stroke, gpop);
    return true;
  }
  return false;
}

/* ------------------------------------------------------- */

static bool gpencil_paint_stroke_test_start(bContext *C, wmOperator *op, const float mouse[2])
{
  Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *obact = CTX_data_active_object(C);
  if (obact == nullptr || obact->type != OB_GPENCIL) {
    return false;
  }

  /* Make sure that we can draw on the active layer. */
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(static_cast<bGPdata *>(obact->data));
  if (gpl != nullptr && (gpl->flag & (GP_LAYER_LOCKED | GP_LAYER_HIDE)) != 0) {
    BKE_report(op->reports, RPT_ERROR, "Active layer is locked or hidden!");
    return false;
  }

  if (gpl != nullptr) {
    bGPDframe *first_frame = static_cast<bGPDframe *>(gpl->frames.first);
    if (first_frame != nullptr && scene->r.cfra < first_frame->framenum &&
        (ts->autokey_mode & AUTOKEY_ON) == 0) {
      BKE_report(op->reports, RPT_ERROR, "No keyframe to draw on!");
      return false;
    }
  }

  if (!gpencil_start_paint_operation(C, static_cast<PaintStroke *>(op->customdata), mouse)) {
    return false;
  }
  return true;
}

static void gpencil_paint_stroke_update_step(bContext *UNUSED(C),
                                             wmOperator *UNUSED(op),
                                             PaintStroke *stroke,
                                             PointerRNA *itemptr)
{
  GPencilPaintOperation *gpop = static_cast<GPencilPaintOperation *>(
      paint_stroke_mode_data(stroke));

  float mval[2];
  RNA_float_get_array(itemptr, "mouse", mval);
  float pressure = RNA_float_get(itemptr, "pressure");
  float size = RNA_float_get(itemptr, "size");
  bool use_eraser = RNA_boolean_get(itemptr, "pen_flip");

  gpop->on_paint_stroke_update(float2(mval), pressure, size, use_eraser);
}

static void gpencil_paint_stroke_redraw(const bContext *C,
                                        PaintStroke *UNUSED(stroke),
                                        bool UNUSED(final))
{
  ED_region_tag_redraw(CTX_wm_region(C));
}

static void gpencil_paint_stroke_done(const bContext *C, PaintStroke *stroke)
{
  GPencilPaintOperation *gpop = static_cast<GPencilPaintOperation *>(
      paint_stroke_mode_data(stroke));
  gpop->on_paint_stroke_done();
  WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, NULL);
  MEM_delete(gpop);
}

static int gpencil_paint_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  PaintStroke *paint_stroke;
  paint_stroke = paint_stroke_new(C,
                                  op,
                                  nullptr,
                                  gpencil_paint_stroke_test_start,
                                  gpencil_paint_stroke_update_step,
                                  gpencil_paint_stroke_redraw,
                                  gpencil_paint_stroke_done,
                                  event->type);
  op->customdata = paint_stroke;

  int return_value = op->type->modal(C, op, event);
  if (return_value == OPERATOR_FINISHED) {
    return OPERATOR_FINISHED;
  }
  WM_event_add_modal_handler(C, op);

  OPERATOR_RETVAL_CHECK(return_value);
  BLI_assert(return_value == OPERATOR_RUNNING_MODAL);

  return OPERATOR_RUNNING_MODAL;
}

static int gpencil_paint_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  return paint_stroke_modal(C, op, event, reinterpret_cast<PaintStroke **>(&op->customdata));
}

static void gpencil_paint_cancel(bContext *C, wmOperator *op)
{
  /* Will free PaintStroke memory. */
  paint_stroke_cancel(C, op, static_cast<PaintStroke *>(op->customdata));
}

void GPENCIL_OT_paint(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Grease Pencil Paint";
  ot->idname = "GPENCIL_OT_paint";
  ot->description = "Paint a new stroke in the active Grease Pencil object";

  /* api callbacks */
  ot->invoke = gpencil_paint_invoke;
  ot->modal = gpencil_paint_modal;
  ot->poll = gpencil_paint_poll;
  ot->cancel = gpencil_paint_cancel;

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_BLOCKING;

  paint_stroke_operator_properties(ot);
}

}  // namespace blender::ed::sculpt_paint::gpencil

/* -------------------------------------------------------------------- */
/** \name * Registration
 * \{ */

void ED_operatortypes_paint_gpencil()
{
  using namespace blender::ed::sculpt_paint::gpencil;
  WM_operatortype_append(GPENCIL_OT_paint);
}

/** \} */