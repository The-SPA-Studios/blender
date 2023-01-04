/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. */

#pragma once

/** \file
 * \ingroup bke
 */

#include <functional>
#include <map>
#include <unordered_set>

#include "BLI_float4x4.hh"
#include "BLI_string_ref.hh"

namespace blender::bke {

/* Key to uniquely identify a layer parent. */
struct tGPLayerParentKey {
  /* Name of the object. */
  blender::StringRef ob_name;
  /* If the object is an armature, store the bone name as well. */
  blender::StringRef bone_name;
};

struct tGPLayerParent {
  /* Reference to the parent of the layer. Can be the gpencil object itself, if the layer is not
   * parented. */
  struct Object *ob = nullptr;
  /* Name of the bone if the layer is parented to one. Otherwise nullptr. */
  blender::StringRef bone_name = nullptr;
  /* Set of frame numbers on which this parent will be evaluated. */
  std::unordered_set<int> frames;
  /* Maps the frame number to a transformation matrix. The parent will have been evaluated at each
   * of these key frames and the computed matrix will be the value stored. */
  std::map<int, blender::float4x4> matrices;
};

void gpencil_build_layer_parents_map(struct Object *ob,
                                     int start_frame,
                                     int end_frame,
                                     std::map<tGPLayerParentKey, tGPLayerParent> &parents_map);

void gpencil_evaluate_layer_parents_on_frames(
    struct Main *bmain,
    struct Scene *scene,
    struct ViewLayer *view_layer,
    struct Object *ob,
    std::map<tGPLayerParentKey, tGPLayerParent> &parents_map,
    std::function<void(int, int)> progress);

void gpencil_write_frames_cache_from_layer_parent_map(
    struct Object *ob, const std::map<tGPLayerParentKey, tGPLayerParent> &parents);

};  // namespace blender::bke