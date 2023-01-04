/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2008 Blender Foundation. */

/** \file
 * \ingroup edgpencil
 */

#include "BKE_context.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_frame_cache.hh"

#include "DNA_scene_types.h"

#include "ED_gpencil.h"

#include "WM_api.h"
#include "WM_types.h"

#include "PIL_time_utildefines.h"

#include "gpencil_intern.h"

struct FrameTransformationCacheJob {
  Main *bmain;
  Scene *scene;
  ViewLayer *view_layer;
  Object *obact;

  std::map<blender::bke::tGPLayerParentKey, blender::bke::tGPLayerParent> parents;
};

static void frame_transformation_start_job(void *customdata,
                                           short *UNUSED(stop),
                                           short *do_update,
                                           float *progress)
{
  FrameTransformationCacheJob *ft_data = static_cast<FrameTransformationCacheJob *>(customdata);
  gpencil_build_layer_parents_map(
      ft_data->obact, ft_data->scene->r.sfra, ft_data->scene->r.efra, ft_data->parents);
  gpencil_evaluate_layer_parents_on_frames(ft_data->bmain,
                                           ft_data->scene,
                                           ft_data->view_layer,
                                           ft_data->obact,
                                           ft_data->parents,
                                           [&](int current, int total_items) {
                                             *progress = static_cast<float>(current) /
                                                         static_cast<float>(total_items);
                                             *do_update = true;
                                           });
}

static void frame_transformation_end_job(void *customdata)
{
  FrameTransformationCacheJob *ft_data = static_cast<FrameTransformationCacheJob *>(customdata);
  gpencil_write_frames_cache_from_layer_parent_map(ft_data->obact, ft_data->parents);
}

static void frame_transformation_customdata_free(void *customdata)
{
  FrameTransformationCacheJob *ft_data = static_cast<FrameTransformationCacheJob *>(customdata);
  MEM_delete(ft_data);
}

bool ED_gpencil_cache_frame_transformations_background(bContext *C, Object *obact)
{
  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *window = CTX_wm_window(C);
  Scene *scene = CTX_data_scene(C);

  if (WM_jobs_test(CTX_wm_manager(C), scene, WM_JOB_TYPE_GPENCIL_FRAME_TRANSFORMATION_CACHE)) {
    return false;
  }

  FrameTransformationCacheJob *ft_data = MEM_new<FrameTransformationCacheJob>(
      "FrameTransformationCacheJob");

  ft_data->bmain = CTX_data_main(C);
  ft_data->scene = scene;
  ft_data->view_layer = CTX_data_view_layer(C);
  ft_data->obact = obact;

  wmJob *job = WM_jobs_get(wm,
                           window,
                           scene,
                           "Ghost Frame Cache",
                           WM_JOB_PROGRESS,
                           WM_JOB_TYPE_GPENCIL_FRAME_TRANSFORMATION_CACHE);

  WM_jobs_customdata_set(job, ft_data, frame_transformation_customdata_free);
  WM_jobs_timer(job, 0.01, NC_GEOM | ND_DATA, NC_GEOM | ND_DATA);
  WM_jobs_callbacks(
      job, frame_transformation_start_job, nullptr, nullptr, frame_transformation_end_job);

  WM_jobs_start(wm, job);
  return true;
}

static int gpencil_cache_ghost_frame_transformations_exec(bContext *C, wmOperator *UNUSED(op))
{
  Scene *scene = CTX_data_scene(C);
  Object *ob = CTX_data_active_object(C);

  ED_gpencil_cache_frame_transformations_background(C, ob);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, scene);
  return OPERATOR_FINISHED;
}

void GPENCIL_OT_cache_ghost_frame_transformations(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Cache Ghost Frame Transformations";
  ot->idname = "GPENCIL_OT_cache_ghost_frame_transformations";
  ot->description = "Update ghost frame transformation cache";

  /* api callbacks */
  ot->exec = gpencil_cache_ghost_frame_transformations_exec;
  ot->poll = gpencil_add_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER;
}

static int gpencil_clear_ghost_frame_transformation_cache_exec(bContext *C, wmOperator *UNUSED(op))
{
  Scene *scene = CTX_data_scene(C);
  Object *ob = CTX_data_active_object(C);

  BKE_gpencil_clear_frames_transformation_cache(scene, ob);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, scene);
  return OPERATOR_FINISHED;
}

void GPENCIL_OT_clear_ghost_frame_transformation_cache(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Clear Ghost Frame Transformations Cache";
  ot->idname = "GPENCIL_OT_clear_ghost_frame_transformation_cache";
  ot->description = "Clear ghost frame transformation cache";

  /* api callbacks */
  ot->exec = gpencil_clear_ghost_frame_transformation_cache_exec;
  ot->poll = gpencil_add_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER;
}