/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2008 Blender Foundation. */

/** \file
 * \ingroup bke
 */

#include <string>

#include "BKE_action.h"
#include "BKE_context.h"
#include "BKE_gpencil.h"
#include "BKE_scene.h"

#include "BLI_float4x4.hh"
#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_math_vector.h"
#include "BLI_set.hh"
#include "BLI_timeit.hh"
#include "BLI_vector.hh"

#include "DNA_gpencil_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph_query.h"

#include "PIL_time_utildefines.h"

#include "BKE_gpencil_frame_cache.hh"

static inline bool gpencil_frame_in_range(bGPDframe *gpf, int start_frame, int end_frame)
{
  return start_frame <= gpf->framenum && gpf->framenum <= end_frame;
}

namespace blender::bke {
constexpr bool operator<(tGPLayerParentKey a, tGPLayerParentKey b)
{
  /* If the object names are equal, test if bone names are different. */
  return a.ob_name < b.ob_name || (a.ob_name == b.ob_name && a.bone_name < b.bone_name);
}

void gpencil_build_layer_parents_map(Object *ob,
                                     int start_frame,
                                     int end_frame,
                                     std::map<tGPLayerParentKey, tGPLayerParent> &parents_map)
{
  bGPdata *gpd = (bGPdata *)ob->data;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* Make sure there are frames on this layer. */
    if (BLI_listbase_is_empty(&gpl->frames)) {
      continue;
    }
    /* Construct an identifier for this layer parent. */
    tGPLayerParentKey key{gpl->parent ? gpl->parent->id.name : ob->id.name,
                          (gpl->partype == PARBONE) ? gpl->parsubstr : "\0"};

    /* Check if the key has already been added. */
    auto elem = parents_map.find(key);
    if (elem != parents_map.end()) {
      /* Make sure to add the times of these frames, so we can retrieve them later. */
      LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
        if (gpencil_frame_in_range(static_cast<bGPDframe *>(gpf), start_frame, end_frame)) {
          elem->second.frames.insert(gpf->framenum);
        }
      }
      continue;
    }

    /* Now construct this parent. Can be just the object itself. */
    tGPLayerParent parent;
    parent.ob = gpl->parent ? gpl->parent : ob;
    if (gpl->partype == PARBONE) {
      parent.bone_name = gpl->parsubstr;
    }
    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      if (gpencil_frame_in_range(static_cast<bGPDframe *>(gpf), start_frame, end_frame)) {
        parent.frames.insert(gpf->framenum);
      }
    }
    parents_map.insert({key, parent});
  }
}

void gpencil_evaluate_layer_parents_on_frames(
    Main *bmain,
    Scene *scene,
    ViewLayer *view_layer,
    Object *ob,
    std::map<tGPLayerParentKey, tGPLayerParent> &parents_map,
    std::function<void(int, int)> progress = nullptr)
{
  /* Build a tempoary minimal dependency graph. */
  Depsgraph *temp_graph = DEG_graph_new(bmain, scene, view_layer, DAG_EVAL_VIEWPORT);
  ID *gpd_id = &ob->id;
  /* Use the ID of the grease pencil object. This will also add any object related to it. */
  DEG_graph_build_from_ids(temp_graph, &gpd_id, 1);

  /* Store the initial time. Will be reset back later. */
  // float initial_time = BKE_scene_ctime_get(scene);

  int total_elems = 0;
  for (auto const &[k, parent] : parents_map) {
    total_elems += parent.frames.size();
  }

  /* We iterate over all the identified parents. For each parent, iterate over the identified
   * frames. Then set the scene to this frame and evaluate the depsgraph. Finally, we compute the
   * transformation and store it in the matrices map for the current parent. */
  int i = 0;
  for (auto &[k, parent] : parents_map) {
    for (int frame : parent.frames) {
      DEG_evaluate_on_framechange(temp_graph, (float)frame);
      Object *ob_parent = DEG_get_evaluated_object(temp_graph, parent.ob);
      blender::float4x4 parent_mat{ob_parent->obmat};
      if (parent.bone_name != nullptr) {
        bPoseChannel *pchan = BKE_pose_channel_find_name(ob_parent->pose, parent.bone_name.data());
        parent_mat *= blender::float4x4(pchan->pose_mat);
      }
      parent.matrices.insert({frame, parent_mat});

      if (progress != nullptr) {
        progress(++i, total_elems);
      }
    }
  }
  if (progress != nullptr) {
    progress(total_elems, total_elems);
  }

  // BKE_scene_frame_set(scene, initial_time);
  DEG_graph_free(temp_graph);
}

void gpencil_write_frames_cache_from_layer_parent_map(
    Object *ob, const std::map<tGPLayerParentKey, tGPLayerParent> &parents)
{
  bGPdata *gpd = (bGPdata *)ob->data;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    tGPLayerParentKey key{gpl->parent ? gpl->parent->id.name : ob->id.name,
                          (gpl->partype == PARBONE) ? gpl->parsubstr : "\0"};
    auto elem = parents.find(key);
    if (elem != parents.end()) {
      auto &matrices = elem->second.matrices;
      LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
        auto elem_mat = matrices.find(gpf->framenum);
        if (elem_mat != matrices.end()) {
          mul_m4_m4m4(gpf->runtime.transform, gpl->layer_mat, elem_mat->second.values);
          gpf->runtime.transform_valid = 1;
        }
      }
    }
  }
}

};  // namespace blender::bke

void BKE_gpencil_update_frames_transformation_cache_ex(Main *bmain,
                                                       Scene *scene,
                                                       ViewLayer *view_layer,
                                                       Object *obact,
                                                       int start_frame,
                                                       int end_frame)
{
  std::map<blender::bke::tGPLayerParentKey, blender::bke::tGPLayerParent> parents;
  /* Build a map of layer parents. A layer parent can be the object itself or some other
   * object/bone. This will also contain on which frames the parents will have to be evaluated.
   */
  blender::bke::gpencil_build_layer_parents_map(obact, start_frame, end_frame, parents);

  /* Use the layer parent map to evaluate the objects on the right frames and store the
   * transformation matricies. */
  blender::bke::gpencil_evaluate_layer_parents_on_frames(bmain, scene, view_layer, obact, parents);

  /* Now we apply the matrices that we computed. Iterate over all frames, retrieve the matrix and
   * set it on the runtime data of the grease pencil frames. */
  blender::bke::gpencil_write_frames_cache_from_layer_parent_map(obact, parents);
}

void BKE_gpencil_update_frames_transformation_cache(Main *bmain,
                                                    Scene *scene,
                                                    ViewLayer *view_layer,
                                                    Object *obact)
{
  return BKE_gpencil_update_frames_transformation_cache_ex(
      bmain, scene, view_layer, obact, scene->r.sfra, scene->r.efra);
}

void BKE_gpencil_clear_frames_transformation_cache_ex(Object *obact,
                                                      int start_frame,
                                                      int end_frame)
{
  bGPdata *gpd = (bGPdata *)obact->data;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      if (gpencil_frame_in_range(gpf, start_frame, end_frame)) {
        gpf->runtime.transform_valid = 0;
        unit_m4(gpf->runtime.transform);
      }
    }
  }
}

void BKE_gpencil_clear_frames_transformation_cache(Scene *scene, Object *obact)
{
  return BKE_gpencil_clear_frames_transformation_cache_ex(obact, scene->r.sfra, scene->r.efra);
}
