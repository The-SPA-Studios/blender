/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup depsgraph
 */

#include "intern/eval/deg_eval_runtime_backup_gpencil.h"
#include "intern/depsgraph.h"

#include "BKE_gpencil.h"
#include "BKE_gpencil_update_cache.h"

#include "DNA_gpencil_types.h"
#include "DNA_userdef_types.h"

namespace blender::deg {

GPencilBackup::GPencilBackup(const Depsgraph *depsgraph) : depsgraph(depsgraph)
{
}

void GPencilBackup::init_from_gpencil(bGPdata *UNUSED(gpd))
{
}

void GPencilBackup::restore_to_gpencil(bGPdata *gpd)
{
  bGPdata *gpd_orig = reinterpret_cast<bGPdata *>(gpd->id.orig_id);

  BKE_gpencil_free_update_cache_if_disposable(reinterpret_cast<const ::Depsgraph *>(depsgraph),
                                              gpd_orig);

  /* Doing a copy-on-write copies the update cache pointer. Make sure to reset it
   * to NULL as we should never use the update cache from eval data. */
  gpd->runtime.update_cache = nullptr;
}

}  // namespace blender::deg
