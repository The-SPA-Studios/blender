/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_brush.h"
#include "BKE_context.h"
#include "BKE_gpencil.h"

#include "BLI_math_vec_types.hh"
#include "BLI_rand.h"

#include "DNA_brush_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_material_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "DEG_depsgraph_query.h"

#include "ED_gpencil.h"
#include "ED_keyframing.h"
#include "ED_view3d.h"

#include "MEM_guardedalloc.h"

#include "PIL_time.h"

struct bContext;

namespace blender::ed::sculpt_paint::gpencil {

class GPencilPaintOperation {
 public:
  GPencilPaintOperation(const bContext *C)
      : depsgraph_(CTX_data_depsgraph_pointer(C)),
        bmain_(CTX_data_main(C)),
        scene_(CTX_data_scene(C)),
        region_(CTX_wm_region(C)),
        view3d_(CTX_wm_view3d(C)),
        rv3d_(CTX_wm_region_view3d(C)),
        obact_(CTX_data_active_object(C)),
        ob_eval_(DEG_get_evaluated_object(depsgraph_, obact_)),
        gpd_(CTX_data_gpencil_data(C)),
        gpd_eval_(static_cast<bGPdata *>(ob_eval_->data)),
        ts_(CTX_data_tool_settings(C)),
        paint_(&ts_->gp_paint->paint)
  {
    if ((paint_->brush == NULL) || (paint_->brush->gpencil_settings == NULL)) {
      /* create new brushes */
      BKE_brush_gpencil_paint_presets(bmain_, ts_, true);
    }
    gpl_active_ = BKE_gpencil_layer_active_get(gpd_);
    brush_ = paint_->brush;
    brush_settings_ = paint_->brush->gpencil_settings;
    material_ = BKE_gpencil_object_material_ensure_from_active_input_brush(
        CTX_data_main(C), obact_, brush_);

    unsigned int rng_seed = static_cast<unsigned int>(PIL_check_seconds_timer_i() & UINT_MAX);
    rng_seed ^= POINTER_AS_UINT(this);
    rng_ = BLI_rng_new(rng_seed);
  }

  virtual ~GPencilPaintOperation()
  {
    BLI_rng_free(rng_);
  };

  virtual void on_paint_stroke_update(float2 location,
                                      float pressure,
                                      float size,
                                      bool use_eraser) = 0;
  virtual void on_paint_stroke_done() = 0;

  static bool poll(const bContext *C)
  {
    ScrArea *area = CTX_wm_area(C);
    if (area->spacetype != SPACE_VIEW3D) {
      return false;
    }

    Object *ob = CTX_data_active_object(C);
    if ((ob == nullptr) || (ob->type != OB_GPENCIL)) {
      return false;
    }
    return true;
  };

 protected:
  Depsgraph *depsgraph_;
  Main *bmain_;
  Scene *scene_;
  ARegion *region_;
  View3D *view3d_;
  RegionView3D *rv3d_;
  Object *obact_;
  Object *ob_eval_;
  bGPdata *gpd_;
  bGPdata *gpd_eval_;
  bGPDlayer *gpl_active_;
  ToolSettings *ts_;
  Paint *paint_;
  Brush *brush_;
  Material *material_;
  BrushGpencilSettings *brush_settings_;
  RNG *rng_;
};

GPencilPaintOperation *new_gpencil_draw_operation(const struct bContext *C,
                                                  const float2 intitial_mouse_location);
void GPENCIL_OT_paint(struct wmOperatorType *ot);

}  // namespace blender::ed::sculpt_paint::gpencil