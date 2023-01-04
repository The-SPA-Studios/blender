/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2011 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup edgpencil
 */

#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_listbase.h"
#include "DNA_gpencil_types.h"
#include "DNA_listBase.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_blender_undo.h"
#include "BKE_context.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_update_cache.h"
#include "BKE_main.h"
#include "BKE_undo_system.h"

#include "ED_gpencil.h"
#include "ED_undo.h"

#include "WM_api.h"
#include "WM_types.h"

#include "DEG_depsgraph.h"

#include "gpencil_intern.h"

typedef struct bGPundonode {
  struct bGPundonode *next, *prev;

  char name[BKE_UNDO_STR_MAX];
  struct bGPdata *gpd;
} bGPundonode;

static ListBase undo_nodes = {NULL, NULL};
static bGPundonode *cur_node = NULL;

int ED_gpencil_session_active(void)
{
  return (BLI_listbase_is_empty(&undo_nodes) == false);
}

int ED_undo_gpencil_step(bContext *C, const int step)
{
  bGPdata **gpd_ptr = NULL, *new_gpd = NULL;

  gpd_ptr = ED_gpencil_data_get_pointers(C, NULL);

  const eUndoStepDir undo_step = (eUndoStepDir)step;
  if (undo_step == STEP_UNDO) {
    if (cur_node->prev) {
      cur_node = cur_node->prev;
      new_gpd = cur_node->gpd;
    }
  }
  else if (undo_step == STEP_REDO) {
    if (cur_node->next) {
      cur_node = cur_node->next;
      new_gpd = cur_node->gpd;
    }
  }

  if (new_gpd) {
    if (gpd_ptr) {
      if (*gpd_ptr) {
        bGPdata *gpd = *gpd_ptr;
        bGPDlayer *gpld;

        BKE_gpencil_free_layers(&gpd->layers);

        /* copy layers */
        BLI_listbase_clear(&gpd->layers);

        LISTBASE_FOREACH (bGPDlayer *, gpl_undo, &gpd->layers) {
          /* make a copy of source layer and its data */
          gpld = BKE_gpencil_layer_duplicate(gpl_undo, true, true);
          BLI_addtail(&gpd->layers, gpld);
        }
      }
    }
    /* drawing batch cache is dirty now */
    DEG_id_tag_update(&new_gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
    new_gpd->flag |= GP_DATA_CACHE_IS_DIRTY;
  }

  WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, NULL);

  return OPERATOR_FINISHED;
}

void gpencil_undo_init(bGPdata *gpd)
{
  gpencil_undo_push(gpd);
}

static void gpencil_undo_free_node(bGPundonode *undo_node)
{
  /* HACK: animdata wasn't duplicated, so it shouldn't be freed here,
   * or else the real copy will segfault when accessed
   */
  undo_node->gpd->adt = NULL;

  BKE_gpencil_free_data(undo_node->gpd, false);
  MEM_freeN(undo_node->gpd);
}

void gpencil_undo_push(bGPdata *gpd)
{
  bGPundonode *undo_node;

  if (cur_node) {
    /* Remove all undone nodes from stack. */
    undo_node = cur_node->next;

    while (undo_node) {
      bGPundonode *next_node = undo_node->next;

      gpencil_undo_free_node(undo_node);
      BLI_freelinkN(&undo_nodes, undo_node);

      undo_node = next_node;
    }
  }

  /* limit number of undo steps to the maximum undo steps
   * - to prevent running out of memory during **really**
   *   long drawing sessions (triggering swapping)
   */
  /* TODO: Undo-memory constraint is not respected yet,
   * but can be added if we have any need for it. */
  if (U.undosteps && !BLI_listbase_is_empty(&undo_nodes)) {
    /* remove anything older than n-steps before cur_node */
    int steps = 0;

    undo_node = (cur_node) ? cur_node : undo_nodes.last;
    while (undo_node) {
      bGPundonode *prev_node = undo_node->prev;

      if (steps >= U.undosteps) {
        gpencil_undo_free_node(undo_node);
        BLI_freelinkN(&undo_nodes, undo_node);
      }

      steps++;
      undo_node = prev_node;
    }
  }

  /* create new undo node */
  undo_node = MEM_callocN(sizeof(bGPundonode), "gpencil undo node");
  BKE_gpencil_data_duplicate(NULL, gpd, &undo_node->gpd);

  cur_node = undo_node;

  BLI_addtail(&undo_nodes, undo_node);
}

void gpencil_undo_finish(void)
{
  bGPundonode *undo_node = undo_nodes.first;

  while (undo_node) {
    gpencil_undo_free_node(undo_node);
    undo_node = undo_node->next;
  }

  BLI_freelistN(&undo_nodes);

  cur_node = NULL;
}

/* -------------------------------------------------------------------- */
/** \name Implements ED Undo System for Grease Pencil
 * \{ */
typedef struct GPencilUndoStep {
  UndoStep step;
  UndoRefID_Object ob_ref_id;
  UndoRefID_Object *layer_parent_ref_ids;
  int layers_size;
  /* This is the structure that indicates the differential changes made coming into
   * (one-directional) this step. The data pointers in the structure are owned by the undo step
   * (i.e. they will be allocated and freed within the undo system). */
  GPencilUpdateCache *diff;
  /* Scene frame number at this step. */
  int scene_cfra;
  /* The grease pencil mode at this step. */
  eObjectMode object_mode;
} GPencilUndoStep;

static bool change_gpencil_mode_if_needed(bContext *C, Object *ob, eObjectMode object_mode)
{
  /* No mode change needed if they are the same. */
  if (ob->mode == object_mode) {
    return false;
  }
  bGPdata *gpd = (bGPdata *)ob->data;
  ob->mode = object_mode;
  ED_gpencil_setup_modes(C, gpd, object_mode);

  return true;
}

static void encode_gpencil_data_to_undo_step(bGPdata *gpd, GPencilUndoStep *gpd_undo_step)
{
  GPencilUpdateCache *update_cache = gpd->runtime.update_cache;

  if (update_cache == NULL) {
    /* Need a full-copy of the grease pencil data. */
    bGPdata *gpd_copy = NULL;
    BKE_gpencil_data_duplicate(NULL, gpd, &gpd_copy);
    gpd_copy->id.session_uuid = gpd->id.session_uuid;

    /* Clear any ID pointers because they will point to invalid memory. */
    gpd_copy->mat = NULL;
    gpd_copy->adt = NULL;

    gpd_undo_step->diff = BKE_gpencil_create_update_cache(gpd_copy, true);
  }
  else {
    gpd_undo_step->diff = BKE_gpencil_duplicate_update_cache_and_data(update_cache);
  }
}

typedef struct tGPencilUpdateCacheUndoTraverseData {
  bGPdata *gpd;
  bGPDlayer *gpl;
  bGPDframe *gpf;
  bGPDstroke *gps;
  int gpl_index;
  int gpf_index;
  int gps_index;
  bool tag_update_cache;
} tGPencilUpdateCacheUndoTraverseData;

static bool gpencil_decode_undo_step_layer_cb(GPencilUpdateCache *gpl_cache, void *user_data)
{
  tGPencilUpdateCacheUndoTraverseData *td = (tGPencilUpdateCacheUndoTraverseData *)user_data;
  td->gpl = BLI_findlinkfrom((Link *)td->gpl, gpl_cache->index - td->gpl_index);
  td->gpl_index = gpl_cache->index;
  bGPDlayer *gpl_new = (bGPDlayer *)gpl_cache->data;

  if (gpl_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Do a full copy of the layer. */
    bGPDlayer *gpl_next = td->gpl->next;
    BKE_gpencil_layer_delete(td->gpd, td->gpl);

    td->gpl = BKE_gpencil_layer_duplicate(gpl_new, true, true);
    BLI_insertlinkbefore(&td->gpd->layers, gpl_next, td->gpl);

    if (td->tag_update_cache) {
      /* Tag the layer here. */
      BKE_gpencil_tag_full_update(td->gpd, td->gpl, NULL, NULL);
    }
    return true;
  }

  if (gpl_cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    BKE_gpencil_layer_copy_settings(gpl_new, td->gpl);
    if (td->tag_update_cache) {
      BKE_gpencil_tag_light_update(td->gpd, td->gpl, NULL, NULL);
    }
  }

  td->gpf = td->gpl->frames.first;
  td->gpf_index = 0;
  return false;
}

static bool gpencil_decode_undo_step_frame_cb(GPencilUpdateCache *gpf_cache, void *user_data)
{
  tGPencilUpdateCacheUndoTraverseData *td = (tGPencilUpdateCacheUndoTraverseData *)user_data;
  td->gpf = BLI_findlinkfrom((Link *)td->gpf, gpf_cache->index - td->gpf_index);
  td->gpf_index = gpf_cache->index;
  bGPDframe *gpf_new = (bGPDframe *)gpf_cache->data;

  if (gpf_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Do a full copy of the frame. */
    bGPDframe *gpf_next = td->gpf->next;

    bool update_actframe = (td->gpl->actframe == td->gpf) ? true : false;
    BKE_gpencil_free_strokes(td->gpf);
    BLI_freelinkN(&td->gpl->frames, td->gpf);

    td->gpf = BKE_gpencil_frame_duplicate(gpf_new, true);
    BLI_insertlinkbefore(&td->gpl->frames, gpf_next, td->gpf);

    if (update_actframe) {
      td->gpl->actframe = td->gpf;
    }
    if (td->tag_update_cache) {
      /* Tag the frame here. */
      BKE_gpencil_tag_full_update(td->gpd, td->gpl, td->gpf, NULL);
    }
    return true;
  }

  if (gpf_cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    BKE_gpencil_frame_copy_settings(gpf_new, td->gpf);
    if (td->tag_update_cache) {
      BKE_gpencil_tag_light_update(td->gpd, td->gpl, td->gpf, NULL);
    }
  }

  td->gps = td->gpf->strokes.first;
  td->gps_index = 0;
  return false;
}

static bool gpencil_decode_undo_step_stroke_cb(GPencilUpdateCache *gps_cache, void *user_data)
{
  tGPencilUpdateCacheUndoTraverseData *td = (tGPencilUpdateCacheUndoTraverseData *)user_data;
  td->gps = BLI_findlinkfrom((Link *)td->gps, gps_cache->index - td->gps_index);
  td->gps_index = gps_cache->index;
  bGPDstroke *gps_new = (bGPDstroke *)gps_cache->data;

  if (gps_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Do a full copy of the stroke. */
    bGPDstroke *gps_next = td->gps->next;

    BLI_remlink(&td->gpf->strokes, td->gps);
    BKE_gpencil_free_stroke(td->gps);

    td->gps = BKE_gpencil_stroke_duplicate(gps_new, true, true);
    BLI_insertlinkbefore(&td->gpf->strokes, gps_next, td->gps);

    if (td->tag_update_cache) {
      /* Tag the stroke here. */
      BKE_gpencil_tag_full_update(td->gpd, td->gpl, td->gpf, td->gps);
    }
  }

  if (gps_cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    BKE_gpencil_stroke_copy_settings(gps_new, td->gps);
    if (td->tag_update_cache) {
      BKE_gpencil_tag_light_update(td->gpd, td->gpl, td->gpf, td->gps);
    }
  }
  return false;
}

static bool decode_undo_step_to_gpencil_data(GPencilUndoStep *gpd_undo_step,
                                             bGPdata *gpd,
                                             bool tag_gpd_update_cache)
{
  GPencilUpdateCache *update_cache = gpd_undo_step->diff;

  BLI_assert(update_cache != NULL);

  if (update_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Full-copy. */
    const bGPdata *gpd_undo = (bGPdata *)gpd_undo_step->diff->data;

    /* Copy all the settings on the grease pencil data. */
    BKE_gpencil_data_copy_settings(update_cache->data, gpd);
    /* Copy all the layers. */
    BKE_gpencil_free_layers(&gpd->layers);
    BLI_listbase_clear(&gpd->layers);
    LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd_undo->layers) {
      bGPDlayer *gpl_new = BKE_gpencil_layer_duplicate(gpl, true, true);
      BLI_addtail(&gpd->layers, gpl_new);
    }
    if (tag_gpd_update_cache) {
      BKE_gpencil_tag_full_update(gpd, NULL, NULL, NULL);
    }
    return true;
  }

  if (update_cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    BKE_gpencil_data_copy_settings(update_cache->data, gpd);
    if (tag_gpd_update_cache) {
      BKE_gpencil_tag_light_update(gpd, NULL, NULL, NULL);
    }
  }

  GPencilUpdateCacheTraverseSettings ts = {{
      gpencil_decode_undo_step_layer_cb,
      gpencil_decode_undo_step_frame_cb,
      gpencil_decode_undo_step_stroke_cb,
  }};

  tGPencilUpdateCacheUndoTraverseData data = {
      .gpd = gpd,
      .gpl = gpd->layers.first,
      .gpf = NULL,
      .gps = NULL,
      .gpl_index = 0,
      .gpf_index = 0,
      .gps_index = 0,
      .tag_update_cache = tag_gpd_update_cache,
  };

  BKE_gpencil_traverse_update_cache(update_cache, &ts, &data);

  return true;
}

/**
 * Returns true if any grease pencil data block (that is not the active one) has an update cache.
 */
static bool gpencil_undosys_check_any_update_cache_on_non_active(bContext *C, bGPdata *gpd_active)
{
  Main *bmain = CTX_data_main(C);
  LISTBASE_FOREACH (bGPdata *, gpd, &bmain->gpencils) {
    if (gpd != gpd_active && gpd->runtime.update_cache != NULL) {
      return true;
    }
  }
  return false;
}

static bool gpencil_undosys_poll(bContext *C)
{
  if (!U.experimental.use_gpencil_undo_system) {
    return false;
  }
  ViewLayer *view_layer = CTX_data_view_layer(C);
  Object *ob = OBACT(view_layer);
  if (ob == NULL || (ob->type != OB_GPENCIL)) {
    return false;
  }
  bGPdata *gpd = (bGPdata *)ob->data;
  if (!GPENCIL_ANY_MODE(gpd) || gpencil_undosys_check_any_update_cache_on_non_active(C, gpd)) {
    return false;
  }
  UndoStack *undo_stack = ED_undo_stack_get();
  Scene *scene = CTX_data_scene(C);
  if (gpd->runtime.update_cache == NULL) {
    if (undo_stack->step_active) {
      if (undo_stack->step_active->type == BKE_UNDOSYS_TYPE_GPENCIL) {
        GPencilUndoStep *us_prev = (GPencilUndoStep *)undo_stack->step_active;
        return us_prev->scene_cfra != scene->r.cfra;
      }
      return false;
    }
  }
  return true;
}

static bool gpencil_undosys_step_encode(struct bContext *C,
                                        struct Main *UNUSED(bmain),
                                        UndoStep *us_p)
{
  GPencilUndoStep *gpd_undo_step = (GPencilUndoStep *)us_p;

  UndoStack *undo_stack = ED_undo_stack_get();
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  Object *ob = OBACT(view_layer);
  bGPdata *gpd = (bGPdata *)ob->data;
  bool only_frame_changed = false;

  /* Mark this step as a barrier for memfile, to force a full undo in case we go
   * from this undo system to memfile. */
  us_p->use_old_bmain_data = false;

  /* Detect changes made to the gpencil datablock since last undo step.
   * Note: we can't use id.recalc since depsgraph update might have happened already. */
  gpd->id.recalc_up_to_undo_push = gpd->id.recalc_after_undo_push;
  gpd->id.recalc_after_undo_push = 0;

  /* In case the step we are about to encode would be the first in the gpencil undo system, ensure
   * that we do a full copy. */
  const bool force_full_update = undo_stack->step_active == NULL ||
                                 undo_stack->step_active->type != BKE_UNDOSYS_TYPE_GPENCIL;
  if (force_full_update) {
    BKE_gpencil_tag_full_update(gpd, NULL, NULL, NULL);
  }
  /* If the ID of the grease pencil object was not tagged or the update cache is empty, we assume
   * the data hasn't changed. */
  else if ((gpd->id.recalc_up_to_undo_push == 0) && gpd->runtime.update_cache == NULL) {

    /* If the previous step is of our undo system, check if the frame changed. */
    if (undo_stack->step_active && undo_stack->step_active->type == BKE_UNDOSYS_TYPE_GPENCIL) {
      GPencilUndoStep *us_prev = (GPencilUndoStep *)undo_stack->step_active;
      /* We want to be able to undo frame changes, so check this here. */
      only_frame_changed = us_prev->scene_cfra != scene->r.cfra;
      if (!only_frame_changed) {
        /* If the frame did not change, we don't need to encode anything, return. */
        return false;
      }
    }
    else {
      /* No change (that we want to undo) happened, return. */
      return false;
    }
  }

  /* TODO: Figure out if doing full-copies and using a lot of memory can be solved in some way. */
#if 0
  if (!only_frame_changed && gpd->runtime.update_cache == NULL) {
    // printf("GP UndoSystem Warning: Fallback to full copy.\n");
    return false;
  }
#endif

  gpd_undo_step->scene_cfra = scene->r.cfra;
  gpd_undo_step->object_mode = ob->mode;
  gpd_undo_step->ob_ref_id.ptr = ob;

  gpd_undo_step->layers_size = BLI_listbase_count(&gpd->layers);
  gpd_undo_step->layer_parent_ref_ids = MEM_callocN(
      gpd_undo_step->layers_size * sizeof(UndoRefID_Object), __func__);

  int i = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    gpd_undo_step->layer_parent_ref_ids[i].ptr = gpl->parent;
    i++;
  }

  /* If that step only encodes a frame change (data itself has not changed), return early. */
  if (only_frame_changed) {
    return true;
  }

  /* Encode the differential changes made to the gpencil data coming into this step. */
  encode_gpencil_data_to_undo_step(gpd, gpd_undo_step);
  /* Because the encoding of a gpencil undo step uses the update cache on the gpencil data, we can
   * tag it after the encode so that the update-on-write knows that it can be safely disposed. */

  /* Deal with update cache invalidation. */
  if (gpd->runtime.update_cache != NULL) {
    /* If gpd->id.recalc is unset, it means that depsgraph update already happened.
     * Therfore, we can free the update cache. */
    if (gpd->id.recalc == 0) {
      BKE_gpencil_free_update_cache(gpd);
    }
    /* Otherwise, we mark it as disposable for the next depsgraph update to free it. */
    else {
      gpd->flag |= GP_DATA_UPDATE_CACHE_DISPOSABLE;
    }
  }

  /* In case we forced a full update, we want to make sure that the gpd.runtime does not contain a
   * cache since the eval object already contains the correct data and we don't want to go through
   * an update-on-write. */
  if (force_full_update) {
    BKE_gpencil_free_update_cache(gpd);
  }

  return true;
}

static bool step_is_skippable(GPencilUndoStep *data_current,
                              GPencilUndoStep *gpd_undo_step_next,
                              bool is_target_step)
{
  /* If the step is e.g. a frame change, there is no cache, so we can safely skip it. */
  if (data_current->diff == NULL) {
    return true;
  }

  /* We must always decode the target step. */
  if (is_target_step) {
    return false;
  }

  /* If decoding the changes of the current step will be overwritten by the next one, then we can
   * skip it. */
  if (gpd_undo_step_next->diff != NULL &&
      BKE_gpencil_update_cache_A_subset_of_B(data_current->diff, gpd_undo_step_next->diff)) {
    return true;
  }

  /* Otherwise decode the step as usual. */
  return false;
}

static void gpencil_undosys_step_decode(struct bContext *C,
                                        struct Main *UNUSED(bmain),
                                        UndoStep *us_p,
                                        const eUndoStepDir dir,
                                        bool is_final)
{
  GPencilUndoStep *gpd_undo_step = (GPencilUndoStep *)us_p;

  Object *ob = gpd_undo_step->ob_ref_id.ptr;
  bGPdata *gpd = (bGPdata *)ob->data;

  if (gpd == NULL) {
    return;
  }

  /* The decode step of the undo should be the last time we write to the gpd update cache, so tag
   * it as disposable here and the update-on-write will be able to free it. */
  if (is_final) {
    gpd->flag |= GP_DATA_UPDATE_CACHE_DISPOSABLE;
    gpd->id.recalc_after_undo_push = 0;
  }

  Scene *scene = CTX_data_scene(C);
  if (gpd_undo_step->scene_cfra != scene->r.cfra) {
    scene->r.cfra = gpd_undo_step->scene_cfra;
    if (is_final) {
      DEG_id_tag_update(&scene->id, ID_RECALC_FRAME_CHANGE);
      WM_event_add_notifier(C, NC_SCENE | ND_FRAME, NULL);
    }
  }

  /* Check if a mode change needs to happen (by comparing the saved mode flags on the undo step
   * data) and switch to that mode. */
  const bool mode_changed = change_gpencil_mode_if_needed(C, ob, gpd_undo_step->object_mode);

  /* If the mode was updated, make sure to tag the ID and add notifiers. */
  if (mode_changed && is_final) {
    DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | ND_GPENCIL_EDITMODE, NULL);
    WM_event_add_notifier(C, NC_SCENE | ND_MODE, NULL);
  }

  if (dir == STEP_UNDO) {
    UndoStep *us_iter = us_p;
    /* Assume that a next steps always exists in case that we undo a step. */
    BLI_assert(us_iter->next != NULL);
    UndoStep *us_next = us_p->next;

    bool need_full_copy_decode = false;
    /* If we come from a step that was outside the gpencil undo system, assume that this was a mode
     * change from object mode and that we don't need to decode anything. */
    if (us_next->type != BKE_UNDOSYS_TYPE_GPENCIL) {
      if (mode_changed) {
        return;
      }
      need_full_copy_decode = true;
    }

    GPencilUndoStep *gpd_undo_step_iter = (GPencilUndoStep *)us_iter;
    GPencilUndoStep *gpd_undo_step_next = NULL;
    if (!need_full_copy_decode) {
      gpd_undo_step_next = (GPencilUndoStep *)us_next;
      /* If the next step does not cache any update, then it means that it did not change the
       * gpencil data. Therefore, we already are in the correct state and can return early. */
      if (gpd_undo_step_next->diff == NULL) {
        return;
      }
    }

    /* Find an undo step in the past, that contains enough data to be able to undo (e.g.
     * potentially recover) the step we came from. Skip over steps with no cache (e.g. a frame
     * change). */
    while (gpd_undo_step_iter->diff == NULL ||
           (need_full_copy_decode && gpd_undo_step_iter->diff->flag != GP_UPDATE_NODE_FULL_COPY) ||
           (!need_full_copy_decode && !BKE_gpencil_update_cache_A_subset_of_B(
                                          gpd_undo_step_next->diff, gpd_undo_step_iter->diff))) {
      us_iter = us_iter->prev;
      /* We assume that there are no "gaps" in the undo chain. There should always be a full-copy
       * at the beginning of a chain. */
      BLI_assert(us_iter != NULL && us_iter->type == BKE_UNDOSYS_TYPE_GPENCIL);
      gpd_undo_step_iter = (GPencilUndoStep *)us_iter;
    }

    /* Once we find a good undo step, we need to go Back to the Future, so re-apply all the steps
     * until we reach the target step. */
    while (us_iter != us_next) {
      gpd_undo_step_next = (GPencilUndoStep *)us_iter->next;
      /* Check if we can skip this step. If we can't we need to decode it. */
      if (!step_is_skippable(gpd_undo_step_iter, gpd_undo_step_next, us_iter->next == us_next)) {
        decode_undo_step_to_gpencil_data(gpd_undo_step_iter, gpd, true);
      }
      us_iter = us_iter->next;
      gpd_undo_step_iter = (GPencilUndoStep *)us_iter;
    }
  }
  else if (dir == STEP_REDO) {
    /* If the current step does not cache any update, then we don't need to decode anything.*/
    if (gpd_undo_step->diff == NULL) {
      return;
    }
    /* Otherwise, we apply the cached changes to the current gpencil data. */
    decode_undo_step_to_gpencil_data(gpd_undo_step, gpd, true);
  }
  else {
    BLI_assert_unreachable();
  }

  if (is_final) {
    int i = 0;
    LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
      gpl->parent = gpd_undo_step->layer_parent_ref_ids[i].ptr;
      i++;
    }
    /* Tag gpencil for depsgraph update. */
    gpd->flag |= GP_DATA_UPDATE_CACHE_DISPOSABLE;
    DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, NULL);
  }
}

static void gpencil_undosys_step_free(UndoStep *us_p)
{
  GPencilUndoStep *gpd_undo_step = (GPencilUndoStep *)us_p;

  /* If this undo step is the first, we want to keep its full copy of the grease pencil diff
   * (we assume that the first undo step always has this). Otherwise we free the step and its
   * diff. */
  if ((us_p->prev == NULL || us_p->prev->type != BKE_UNDOSYS_TYPE_GPENCIL) && us_p->next != NULL &&
      us_p->next->type == BKE_UNDOSYS_TYPE_GPENCIL) {
    GPencilUndoStep *gpd_undo_step_next = (GPencilUndoStep *)us_p->next;

    /* If e.g. a frame change happend, there is no cache so in this case we move the gpd pointer to
     * that step. */
    if (gpd_undo_step_next->diff == NULL) {
      bGPdata *gpd_copy = gpd_undo_step->diff->data;
      BLI_assert(gpd_copy != NULL);
      BLI_assert(gpd_undo_step->diff->flag == GP_UPDATE_NODE_FULL_COPY);

      gpd_undo_step_next->diff = BKE_gpencil_create_update_cache(gpd_copy, true);
      /* Make sure the gpd_copy is not freed below. */
      gpd_undo_step->diff->data = NULL;
    }
    /* If the next step does not have a full copy, we need to apply the changes of the next step
     * to our cached gpencil diff, copy, and move it to the next step (it will now be the
     * full-copy). */
    else if (gpd_undo_step_next->diff->flag != GP_UPDATE_NODE_FULL_COPY) {
      bGPdata *gpd_copy = gpd_undo_step->diff->data;
      BLI_assert(gpd_copy != NULL);
      BLI_assert(gpd_undo_step->diff->flag == GP_UPDATE_NODE_FULL_COPY);

      /* Apply the changes of the next step to the gpd full copy of the first step to that it
       * contains both changes. */
      decode_undo_step_to_gpencil_data(gpd_undo_step_next, gpd_copy, false);

      /* Replace the data of the next step with the (now updated) full copy. */
      BKE_gpencil_free_update_cache_and_data(gpd_undo_step_next->diff);
      gpd_undo_step_next->diff = BKE_gpencil_create_update_cache(gpd_copy, true);

      /* Because we just moved the pointer to the next step, set it to NULL in the original first
       * step to make sure the gpd_copy is not freed below. */
      gpd_undo_step->diff->data = NULL;
    }
    else {
      /* If the next step is a full copy, we can safely free the current step (since the first step
       * will be a full-copy). */
    }
  }

  /* Free the step and its data (because undo steps "own" the data contained in their cache). */
  if (gpd_undo_step->diff) {
    BKE_gpencil_free_update_cache_and_data(gpd_undo_step->diff);
  }

  MEM_freeN(gpd_undo_step->layer_parent_ref_ids);
}

static void gpencil_undosys_foreach_ID_ref(UndoStep *us_p,
                                           UndoTypeForEachIDRefFn foreach_ID_ref_fn,
                                           void *user_data)
{
  GPencilUndoStep *us = (GPencilUndoStep *)us_p;

  foreach_ID_ref_fn(user_data, ((UndoRefID *)&us->ob_ref_id));
  for (int i = 0; i < us->layers_size; i++) {
    UndoRefID_Object *ref_id = &us->layer_parent_ref_ids[i];
    if (ref_id->name[0] == '\0' && ref_id->ptr == NULL) {
      continue;
    }
    foreach_ID_ref_fn(user_data, (UndoRefID *)ref_id);
  }
}

void ED_gpencil_undosys_type(UndoType *ut)
{
  ut->name = "Grease Pencil Undo";
  ut->poll = gpencil_undosys_poll;
  ut->step_encode = gpencil_undosys_step_encode;
  ut->step_decode = gpencil_undosys_step_decode;
  ut->step_free = gpencil_undosys_step_free;

  ut->step_foreach_ID_ref = gpencil_undosys_foreach_ID_ref;

  ut->flags = UNDOTYPE_FLAG_NEED_CONTEXT_FOR_ENCODE;

  ut->step_size = sizeof(GPencilUndoStep);
}

/** \} */
