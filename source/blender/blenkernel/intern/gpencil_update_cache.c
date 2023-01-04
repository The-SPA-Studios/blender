/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. */

/** \file
 * \ingroup bke
 */

#include <stdio.h>

#include "BKE_gpencil_update_cache.h"

#include "BLI_dlrbTree.h"
#include "BLI_listbase.h"

#include "BKE_gpencil.h"

#include "DEG_depsgraph.h"

#include "DNA_gpencil_types.h"
#include "DNA_userdef_types.h"

#include "MEM_guardedalloc.h"

static GPencilUpdateCache *update_cache_alloc(int index, int flag, void *data)
{
  GPencilUpdateCache *new_cache = MEM_callocN(sizeof(GPencilUpdateCache), __func__);
  new_cache->children = BLI_dlrbTree_new();
  new_cache->flag = flag;
  new_cache->index = index;
  new_cache->data = data;

  return new_cache;
}

static short cache_node_compare(void *node, void *data)
{
  int index_a = ((GPencilUpdateCacheNode *)node)->cache->index;
  int index_b = ((GPencilUpdateCache *)data)->index;
  if (index_a == index_b) {
    return 0;
  }
  return index_a < index_b ? 1 : -1;
}

static DLRBT_Node *cache_node_alloc(void *data)
{
  GPencilUpdateCacheNode *new_node = MEM_callocN(sizeof(GPencilUpdateCacheNode), __func__);
  new_node->cache = ((GPencilUpdateCache *)data);
  return (DLRBT_Node *)new_node;
}

static void cache_node_free(void *node);

static void update_cache_free(GPencilUpdateCache *cache)
{
  BLI_dlrbTree_free(cache->children, cache_node_free);
  MEM_SAFE_FREE(cache->children);
  MEM_freeN(cache);
}

static void cache_node_free(void *node)
{
  GPencilUpdateCache *cache = ((GPencilUpdateCacheNode *)node)->cache;
  if (cache != NULL) {
    update_cache_free(cache);
  }
  MEM_freeN(node);
}

static void cache_node_update(void *node, void *data)
{
  GPencilUpdateCache *update_cache = ((GPencilUpdateCacheNode *)node)->cache;
  GPencilUpdateCache *new_update_cache = (GPencilUpdateCache *)data;

  const bool current_cache_covers_new_cache = new_update_cache->flag < update_cache->flag;

  /* In case:
   * - The new cache is a "no-copy".
   * - Or the new cache is a light copy and the current cache a full copy
   * then it means we are already caching "more" and we shouldn't update the current cache.
   * So we free the structure and return early.
   * - See the comment below for why we cannot assume an equal cache type covers the other.
   */
  if (current_cache_covers_new_cache) {
    update_cache_free(new_update_cache);
    return;
  }

  /* In case:
   * - The cache types are equal.
   * - Or the new cache contains more than the current cache (full copy > light copy > no copy)
   * the data pointer is updated. If the cache types are equal, this might be a no-op (when the new
   * data pointer is equal to the previous), but is necessary when the data pointer needs to
   * change. This can for example happen when the underlying data was reallocated, but the cache
   * type stayed the same.
   */
  update_cache->data = new_update_cache->data;
  update_cache->flag = new_update_cache->flag;

  /* In case the new cache does a full update, remove its children since they will be all
   * updated by this cache. */
  if (new_update_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* We don't free the tree itself here, because we just want to clear the children, not delete
     * the whole node. */
    BLI_dlrbTree_free(update_cache->children, cache_node_free);
  }

  /* Once we updated the data pointer and the flag, we can safely free the new cache structure. */
  update_cache_free(new_update_cache);
}

static void update_cache_node_create_ex(GPencilUpdateCache *root_cache,
                                        void *data,
                                        int gpl_index,
                                        int gpf_index,
                                        int gps_index,
                                        bool full_copy)
{
  if (root_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Entire data-block has to be recalculated, e.g. nothing else needs to be added to the cache.
     */
    return;
  }

  const int node_flag = full_copy ? GP_UPDATE_NODE_FULL_COPY : GP_UPDATE_NODE_LIGHT_COPY;

  if (gpl_index == -1) {
    root_cache->data = (bGPdata *)data;
    root_cache->flag = node_flag;
    if (full_copy) {
      /* Entire data-block has to be recalculated, remove all caches of "lower" elements. */
      BLI_dlrbTree_free(root_cache->children, cache_node_free);
    }
    return;
  }

  const bool is_layer_update_node = (gpf_index == -1);
  /* If the data pointer in #GPencilUpdateCache is NULL, this element is not actually cached
   * and does not need to be updated, but we do need the index to find elements that are in
   * levels below. E.g. if a stroke needs to be updated, the frame it is in would not hold a
   * pointer to it's data. */
  GPencilUpdateCache *gpl_cache = update_cache_alloc(
      gpl_index,
      is_layer_update_node ? node_flag : GP_UPDATE_NODE_NO_COPY,
      is_layer_update_node ? (bGPDlayer *)data : NULL);
  GPencilUpdateCacheNode *gpl_node = (GPencilUpdateCacheNode *)BLI_dlrbTree_add(
      root_cache->children, cache_node_compare, cache_node_alloc, cache_node_update, gpl_cache);

  BLI_dlrbTree_linkedlist_sync(root_cache->children);
  if (gpl_node->cache->flag == GP_UPDATE_NODE_FULL_COPY || is_layer_update_node) {
    return;
  }

  const bool is_frame_update_node = (gps_index == -1);
  GPencilUpdateCache *gpf_cache = update_cache_alloc(
      gpf_index,
      is_frame_update_node ? node_flag : GP_UPDATE_NODE_NO_COPY,
      is_frame_update_node ? (bGPDframe *)data : NULL);
  GPencilUpdateCacheNode *gpf_node = (GPencilUpdateCacheNode *)BLI_dlrbTree_add(
      gpl_node->cache->children,
      cache_node_compare,
      cache_node_alloc,
      cache_node_update,
      gpf_cache);

  BLI_dlrbTree_linkedlist_sync(gpl_node->cache->children);
  if (gpf_node->cache->flag == GP_UPDATE_NODE_FULL_COPY || is_frame_update_node) {
    return;
  }

  GPencilUpdateCache *gps_cache = update_cache_alloc(gps_index, node_flag, (bGPDstroke *)data);
  BLI_dlrbTree_add(gpf_node->cache->children,
                   cache_node_compare,
                   cache_node_alloc,
                   cache_node_update,
                   gps_cache);

  BLI_dlrbTree_linkedlist_sync(gpf_node->cache->children);
}

static void update_cache_node_create(
    bGPdata *gpd, bGPDlayer *gpl, bGPDframe *gpf, bGPDstroke *gps, bool full_copy)
{
  if (gpd == NULL) {
    return;
  }

  GPencilUpdateCache *root_cache = gpd->runtime.update_cache;
  if (root_cache == NULL) {
    gpd->runtime.update_cache = update_cache_alloc(0, GP_UPDATE_NODE_NO_COPY, NULL);
    root_cache = gpd->runtime.update_cache;
  }

  if (root_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Entire data-block has to be recalculated, e.g. nothing else needs to be added to the cache.
     */
    return;
  }

  const int gpl_index = (gpl != NULL) ? BLI_findindex(&gpd->layers, gpl) : -1;
  const int gpf_index = (gpl != NULL && gpf != NULL) ? BLI_findindex(&gpl->frames, gpf) : -1;
  const int gps_index = (gpf != NULL && gps != NULL) ? BLI_findindex(&gpf->strokes, gps) : -1;

  void *data = gps;
  if (!data) {
    data = gpf;
  }
  if (!data) {
    data = gpl;
  }
  if (!data) {
    data = gpd;
  }

  update_cache_node_create_ex(root_cache, data, gpl_index, gpf_index, gps_index, full_copy);
}

static void gpencil_traverse_update_cache_ex(GPencilUpdateCache *parent_cache,
                                             GPencilUpdateCacheTraverseSettings *ts,
                                             int depth,
                                             void *user_data)
{
  if (BLI_listbase_is_empty((ListBase *)parent_cache->children)) {
    return;
  }

  LISTBASE_FOREACH (GPencilUpdateCacheNode *, cache_node, parent_cache->children) {
    GPencilUpdateCache *cache = cache_node->cache;

    GPencilUpdateCacheIter_Cb cb = ts->update_cache_cb[depth];
    if (cb != NULL) {
      bool skip = cb(cache, user_data);
      if (skip) {
        continue;
      }
    }

    gpencil_traverse_update_cache_ex(cache, ts, depth + 1, user_data);
  }
}

typedef struct GPencilUpdateCacheDuplicateTraverseData {
  GPencilUpdateCache *new_cache;
  int gpl_index;
  int gpf_index;
} GPencilUpdateCacheDuplicateTraverseData;

static bool gpencil_duplicate_update_cache_layer_cb(GPencilUpdateCache *cache, void *user_data)
{
  GPencilUpdateCacheDuplicateTraverseData *td = (GPencilUpdateCacheDuplicateTraverseData *)
      user_data;

  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    /* Do a full copy of the layer. */
    bGPDlayer *gpl = (bGPDlayer *)cache->data;
    bGPDlayer *gpl_new = BKE_gpencil_layer_duplicate(gpl, true, true);
    update_cache_node_create_ex(td->new_cache, gpl_new, cache->index, -1, -1, true);
    return true;
  }
  else if (cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    bGPDlayer *gpl = (bGPDlayer *)cache->data;
    bGPDlayer *gpl_new = (bGPDlayer *)MEM_dupallocN(gpl);

    gpl_new->prev = gpl_new->next = NULL;
    gpl_new->actframe = NULL;
    gpl_new->parent = NULL;
    BLI_listbase_clear(&gpl_new->frames);
    BLI_listbase_clear(&gpl_new->mask_layers);
    update_cache_node_create_ex(td->new_cache, gpl_new, cache->index, -1, -1, false);
  }
  td->gpl_index = cache->index;
  return false;
}

static bool gpencil_duplicate_update_cache_frame_cb(GPencilUpdateCache *cache, void *user_data)
{
  GPencilUpdateCacheDuplicateTraverseData *td = (GPencilUpdateCacheDuplicateTraverseData *)
      user_data;
  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    bGPDframe *gpf = (bGPDframe *)cache->data;
    bGPDframe *gpf_new = BKE_gpencil_frame_duplicate(gpf, true);
    update_cache_node_create_ex(td->new_cache, gpf_new, td->gpl_index, cache->index, -1, true);
    return true;
  }
  else if (cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    bGPDframe *gpf = (bGPDframe *)cache->data;
    bGPDframe *gpf_new = MEM_dupallocN(gpf);
    gpf_new->prev = gpf_new->next = NULL;
    BLI_listbase_clear(&gpf_new->strokes);
    update_cache_node_create_ex(td->new_cache, gpf_new, td->gpl_index, cache->index, -1, false);
  }
  td->gpf_index = cache->index;
  return false;
}

static bool gpencil_duplicate_update_cache_stroke_cb(GPencilUpdateCache *cache, void *user_data)
{
  GPencilUpdateCacheDuplicateTraverseData *td = (GPencilUpdateCacheDuplicateTraverseData *)
      user_data;

  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    bGPDstroke *gps = (bGPDstroke *)cache->data;
    bGPDstroke *gps_new = BKE_gpencil_stroke_duplicate(gps, true, true);
    update_cache_node_create_ex(
        td->new_cache, gps_new, td->gpl_index, td->gpf_index, cache->index, true);
  }
  else if (cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    bGPDstroke *gps = (bGPDstroke *)cache->data;
    bGPDstroke *gps_new = MEM_dupallocN(gps);

    gps_new->prev = gps_new->next = NULL;
    gps_new->points = NULL;
    gps_new->triangles = NULL;
    gps_new->dvert = NULL;
    gps_new->editcurve = NULL;

    update_cache_node_create_ex(
        td->new_cache, gps_new, td->gpl_index, td->gpf_index, cache->index, false);
  }
  return true;
}

static bool gpencil_free_update_cache_layer_cb(GPencilUpdateCache *cache, void *UNUSED(user_data))
{
  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    BKE_gpencil_free_frames(cache->data);
    BKE_gpencil_free_layer_masks(cache->data);
  }
  if (cache->data) {
    MEM_freeN(cache->data);
  }
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

static bool gpencil_free_update_cache_frame_cb(GPencilUpdateCache *cache, void *UNUSED(user_data))
{
  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    BKE_gpencil_free_strokes(cache->data);
  }
  if (cache->data) {
    MEM_freeN(cache->data);
  }
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

static bool gpencil_free_update_cache_stroke_cb(GPencilUpdateCache *cache, void *UNUSED(user_data))
{
  if (cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    BKE_gpencil_free_stroke(cache->data);
  }
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

static bool gpencil_print_update_cache_layer_cb(GPencilUpdateCache *cache, void *UNUSED(user_data))
{
  printf("  - Layer: %s | Index: %d | Flag: %d | Tagged Frames: %d\n",
         (cache->data ? ((bGPDlayer *)cache->data)->info : "N/A"),
         cache->index,
         cache->flag,
         BLI_listbase_count((ListBase *)cache->children));
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

static bool gpencil_print_update_cache_frame_cb(GPencilUpdateCache *cache, void *UNUSED(user_data))
{
  printf("  - Frame: %d | Index: %d | Flag: %d | Tagged Strokes: %d\n",
         (cache->data ? ((bGPDframe *)cache->data)->framenum : -1),
         cache->index,
         cache->flag,
         BLI_listbase_count((ListBase *)cache->children));
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

static bool gpencil_print_update_cache_stroke_cb(GPencilUpdateCache *cache,
                                                 void *UNUSED(user_data))
{
  printf("     - Stroke Index: %d | | Flag: %d\n", cache->index, cache->flag);
  return cache->flag == GP_UPDATE_NODE_FULL_COPY;
}

/* -------------------------------------------------------------------- */
/** \name Update Cache API
 *
 * \{ */

GPencilUpdateCache *BKE_gpencil_create_update_cache(void *data, bool full_copy)
{
  return update_cache_alloc(
      0, full_copy ? GP_UPDATE_NODE_FULL_COPY : GP_UPDATE_NODE_LIGHT_COPY, data);
}

void BKE_gpencil_traverse_update_cache(GPencilUpdateCache *cache,
                                       GPencilUpdateCacheTraverseSettings *ts,
                                       void *user_data)
{
  gpencil_traverse_update_cache_ex(cache, ts, 0, user_data);
}

void BKE_gpencil_tag_full_update(bGPdata *gpd, bGPDlayer *gpl, bGPDframe *gpf, bGPDstroke *gps)
{
  update_cache_node_create(gpd, gpl, gpf, gps, true);
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY);
}

void BKE_gpencil_tag_light_update(bGPdata *gpd, bGPDlayer *gpl, bGPDframe *gpf, bGPDstroke *gps)
{
  update_cache_node_create(gpd, gpl, gpf, gps, false);
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY);
}

GPencilUpdateCache *BKE_gpencil_duplicate_update_cache_and_data(GPencilUpdateCache *gpd_cache)
{
  GPencilUpdateCache *new_cache = update_cache_alloc(0, gpd_cache->flag, NULL);
  bGPdata *gpd_new = NULL;
  if (gpd_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
    BKE_gpencil_data_duplicate(NULL, gpd_cache->data, &gpd_new);
    new_cache->data = gpd_new;
    return new_cache;
  }
  else if (gpd_cache->flag == GP_UPDATE_NODE_LIGHT_COPY) {
    gpd_new = MEM_dupallocN(gpd_cache->data);

    /* Clear all the pointers, since they shouldn't store anything. */
    BLI_listbase_clear(&gpd_new->layers);
    BLI_listbase_clear(&gpd_new->vertex_group_names);
    gpd_new->adt = NULL;
    gpd_new->mat = NULL;
    gpd_new->runtime.update_cache = NULL;

    new_cache->data = gpd_new;
  }

  GPencilUpdateCacheTraverseSettings ts = {{gpencil_duplicate_update_cache_layer_cb,
                                            gpencil_duplicate_update_cache_frame_cb,
                                            gpencil_duplicate_update_cache_stroke_cb}};

  GPencilUpdateCacheDuplicateTraverseData td = {
      .new_cache = new_cache,
      .gpl_index = -1,
      .gpf_index = -1,
  };

  BKE_gpencil_traverse_update_cache(gpd_cache, &ts, &td);
  return new_cache;
}

bool BKE_gpencil_update_cache_A_subset_of_B(GPencilUpdateCache *cache_A,
                                            GPencilUpdateCache *cache_B)
{
  /* We can return early in case the type indicates that A contains more information than B. */
  if (cache_A->flag > cache_B->flag) {
    return false;
  }
  /* If B contains all the information possible, A must be a subset. */
  if (cache_B->flag == GP_UPDATE_NODE_FULL_COPY) {
    return true;
  }

  /* For all nodes in A: */
  LISTBASE_FOREACH (GPencilUpdateCacheNode *, node_a, cache_A->children) {
    /* Look for a matching node in B */
    GPencilUpdateCacheNode *node_b = (GPencilUpdateCacheNode *)BLI_dlrbTree_search_exact(
        cache_B->children, cache_node_compare, node_a->cache);

    /* If we did not find such a node in B, A can never be a subset of B, return.*/
    if (node_b == NULL) {
      return false;
    }

    /* Recursively check the children of A against the children of B. If any of the children are
     * not a subset, then A cannot be a subset of B. */
    if (!BKE_gpencil_update_cache_A_subset_of_B(node_a->cache, node_b->cache)) {
      return false;
    }
  }

  /* We did not find anything in A that is not in B -> A must be a subset of B. */
  return true;
}

bool BKE_gpencil_free_update_cache_if_disposable(const Depsgraph *depsgraph, bGPdata *gpd)
{
  if (gpd == NULL) {
    return false;
  }
  /* If the gpencil undo system is active, make sure to only free the cache if
   * GP_DATA_UPDATE_CACHE_DISPOSABLE is set. Even though we already used the cache to update the
   * eval object, it might still be needed for the undo system (e.g if a modal operator is running,
   * it might call the update-on-write multiple times before an undo step is encoded). Only when
   * the undo system marks the cache as disposable can we safely free it here.*/
  const bool gpencil_undo_system_inactive = !(U.experimental.use_gpencil_undo_system &&
                                              GPENCIL_ANY_MODE(gpd));

  /* We check for the active depsgraph here to avoid freeing the cache on the original object
   * multiple times. This free is only needed for the case where we tagged a full update in the
   * update cache and did not do an update-on-write. */
  if (DEG_is_active(depsgraph) &&
      (gpencil_undo_system_inactive || gpd->flag & GP_DATA_UPDATE_CACHE_DISPOSABLE)) {
    BKE_gpencil_free_update_cache(gpd);
    return true;
  }
  return false;
}

void BKE_gpencil_free_update_cache(bGPdata *gpd)
{
  GPencilUpdateCache *gpd_cache = gpd->runtime.update_cache;
  if (gpd_cache) {
    update_cache_free(gpd_cache);
    gpd->runtime.update_cache = NULL;
  }
  gpd->flag &= ~GP_DATA_UPDATE_CACHE_DISPOSABLE;
}

void BKE_gpencil_free_update_cache_and_data(GPencilUpdateCache *gpd_cache)
{
  if (gpd_cache->data != NULL) {
    if (gpd_cache->flag == GP_UPDATE_NODE_FULL_COPY) {
      BKE_gpencil_free_data(gpd_cache->data, true);
      MEM_freeN(gpd_cache->data);
      update_cache_free(gpd_cache);
      return;
    }
    MEM_freeN(gpd_cache->data);
  }

  GPencilUpdateCacheTraverseSettings ts = {{gpencil_free_update_cache_layer_cb,
                                            gpencil_free_update_cache_frame_cb,
                                            gpencil_free_update_cache_stroke_cb}};

  BKE_gpencil_traverse_update_cache(gpd_cache, &ts, NULL);
  update_cache_free(gpd_cache);
}

void BKE_gpencil_print_update_cache(GPencilUpdateCache *update_cache)
{
  if (update_cache == NULL) {
    printf("No update cache\n");
    return;
  }
  printf("Update cache: - Flag: %d | Tagged Layers: %d\n",
         update_cache->flag,
         BLI_listbase_count((ListBase *)update_cache->children));

  GPencilUpdateCacheTraverseSettings ts = {{gpencil_print_update_cache_layer_cb,
                                            gpencil_print_update_cache_frame_cb,
                                            gpencil_print_update_cache_stroke_cb}};
  BKE_gpencil_traverse_update_cache(update_cache, &ts, NULL);
}

/** \} */
