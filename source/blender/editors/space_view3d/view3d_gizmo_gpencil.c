
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file
 * \ingroup spview3d
 */

#include <stdio.h>

#include "BKE_context.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_geom.h"
#include "BKE_gpencil_update_cache.h"

#include "BLI_ghash.h"
#include "BLI_lasso_2d.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_math_matrix.h"
#include "BLI_rect.h"
#include "BLI_utildefines.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "DNA_gpencil_types.h"
#include "DNA_workspace_types.h"

#include "ED_gizmo_library.h"
#include "ED_gizmo_utils.h"
#include "ED_gpencil.h"
#include "ED_screen.h"
#include "ED_select_utils.h"
#include "ED_space_api.h"
#include "ED_undo.h"
#include "ED_view3d.h"

#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_primitive.h"
#include "GPU_shader.h"
#include "GPU_state.h"
#include "GPU_vertex_format.h"

#include "MEM_guardedalloc.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "UI_resources.h"
#include "UI_view2d.h"

#include "WM_api.h"
#include "WM_message.h"
#include "WM_toolsystem.h"
#include "WM_types.h"

#include "view3d_intern.h"

/* -------------------------------------------------------------------- */
/** \name Transform Plane Gizmo
 * \{ */

typedef struct tTransformPlane {
  /* Object space location of the transform-plane. */
  float co[3];
  /* Object space normal direction of the transform-plane. */
  float no[3];
  /* Object space basis vector in U direction. */
  float u[3];
  /* Object space basis vector in V direction. */
  float v[3];
  /* Object space transform-plane equation. (ax + by + cz + d = 0). */
  float eq[4];
} tTransformPlane;

typedef struct tGPTransData {
  /* Pointer to the location. */
  float *loc;
  /* Initial location. */
  float iloc[3];
  /* Pointer to trans value. Used for pressure. */
  float *val;
  /* Initial pressure. */
  float ival;
} tGPTransData;

typedef struct tGPTransformFrameData {
  /* Pointer to the gp frame. */
  bGPDframe *frame;
  /* Array of pointers to strokes that are going to be transformed. */
  bGPDstroke **strokes;
  /* Array of transformation data for each stroke for each point. */
  tGPTransData **trans_data;
  /* Total number of strokes to be transformed in this layer. */
  uint tot_strokes;

  /* TODO: those are not used yet! */
  /* Transformation matrix for this frame (Frame Space). */
  float diff_mat[4][4];
  /* Inverse matrix of `diff_mat`. */
  float diff_inv[4][4];
} tGPTransformFrameData;

typedef struct tGPTransformLayerData {
  /* Pointer to the gp layer. */
  bGPDlayer *layer;
  /* Array of frame transform data. */
  tGPTransformFrameData *frames;
  /* Total number of gp frames used in the transformation. */
  uint tot_frames;

  /* Transformation matrix for this layer (Layer Space). */
  float diff_mat[4][4];
  /* Inverse matrix of `diff_mat`. */
  float diff_inv[4][4];
} tGPTransformLayerData;

typedef struct XFormBoxWidgetGroup {
  wmGizmo *gizmo;
  /* Only for view orientation. */
  struct {
    float viewinv_m3[3][3];
  } prev;

  /* Structure to hold all the transform-plane values. */
  tTransformPlane plane;

  bContext *C;
  ARegion *region;
  Object *ob;
  Depsgraph *depsgraph;
  bGPdata *gpd;
  ToolSettings *ts;

  struct {
    void *handle;
    GPUVertBuf *vert;
    GPUBatch *batch;
    uint pos;
  } custom_draw;

  float diff_mat[4][4];
  float diff_inv[4][4];

  /* Array of layer transform data. */
  tGPTransformLayerData *layers;
  /* Total number of gp layers used in the transformation. */
  uint tot_layers;

  size_t selection_hash;

  bool gizmo_active;

} XFormBoxWidgetGroup;

static void local_space_to_transform_plane_space(tTransformPlane *plane,
                                                 const float l_co[3],
                                                 float r_co[2])
{
  float tmp[3];
  sub_v3_v3v3(tmp, l_co, plane->co);
  r_co[0] = dot_v3v3(tmp, plane->u);
  r_co[1] = dot_v3v3(tmp, plane->v);
}

static void transform_plane_space_to_local_space(tTransformPlane *plane,
                                                 const float p_co[2],
                                                 float r_co[3])
{
  copy_v3_v3(r_co, plane->co);
  madd_v3_v3fl(r_co, plane->u, p_co[0]);
  madd_v3_v3fl(r_co, plane->v, p_co[1]);
}

static bool is_frame_active(const bGPDlayer *gpl, const bGPDframe *gpf, const bool is_multiedit)
{
  return (gpf == gpl->actframe) || (((gpf->flag & GP_FRAME_SELECT) != 0) && (is_multiedit));
}

static size_t create_selection_hash(XFormBoxWidgetGroup *xgzgroup)
{
  size_t hash = 42;
  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];
      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        hash = BLI_ghashutil_combine_hash(hash, BLI_ghashutil_ptrhash(gps));
      }
    }
  }
  return hash;
}

static size_t get_current_selection_hash(const bContext *C, XFormBoxWidgetGroup *xgzgroup)
{
  const bool is_multiedit = GPENCIL_MULTIEDIT_SESSIONS_ON(xgzgroup->gpd);
  /* Initial hash value. */
  size_t hash = 42;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &xgzgroup->gpd->layers) {
    if (!BKE_gpencil_layer_is_editable(gpl)) {
      continue;
    }

    /* Make sure to include the frame and layer in the hash. This will trigger a refresh when
     * e.g. the same strokes are moved to another layer. */
    hash = BLI_ghashutil_combine_hash(hash, BLI_ghashutil_ptrhash(gpl));

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      if (!is_frame_active(gpl, gpf, is_multiedit)) {
        continue;
      }

      hash = BLI_ghashutil_combine_hash(hash, BLI_ghashutil_ptrhash(gpf));

      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        /* skip strokes that are invalid for current view */
        if (ED_gpencil_stroke_can_use(C, gps) == false) {
          continue;
        }
        /* check if the color is editable */
        if (ED_gpencil_stroke_material_editable(xgzgroup->ob, gpl, gps) == false) {
          continue;
        }

        if (gps->flag & GP_STROKE_SELECT) {
          hash = BLI_ghashutil_combine_hash(hash, BLI_ghashutil_ptrhash(gps));
        }
      }
    }
  }
  return hash;
}

static int count_selected_strokes_in_frame(const bContext *C,
                                           Object *obact,
                                           const bGPDlayer *gpl,
                                           const bGPDframe *gpf)
{
  int strokes_count = 0;
  if (!BKE_gpencil_layer_is_editable(gpl) || gpf == NULL) {
    /* If layer is not editable or there is no keyframe, exit. */
    return 0;
  }

  /* Loop over strokes */
  LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
    /* Skip strokes that are invalid for current view. */
    if (ED_gpencil_stroke_can_use(C, gps) == false) {
      continue;
    }
    /* Check if the color is editable. */
    if (ED_gpencil_stroke_material_editable(obact, gpl, gps) == false) {
      continue;
    }

    if (gps->flag & GP_STROKE_SELECT) {
      strokes_count++;
    }
  }

  return strokes_count;
}

static int count_active_frames(const bGPDlayer *gpl, const bool is_multiedit)
{
  int count = 0;
  LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
    if (is_frame_active(gpl, gpf, is_multiedit)) {
      count++;
    }
  }
  return count;
}

static int count_selected_strokes_in_layer(const bContext *C, Object *obact, const bGPDlayer *gpl)
{
  int strokes_count = 0;
  /* If layer is not editable, exit. */
  const bool is_multiedit = GPENCIL_MULTIEDIT_SESSIONS_ON((bGPdata *)obact->data);
  LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
    if (!is_frame_active(gpl, gpf, is_multiedit)) {
      continue;
    }
    strokes_count += count_selected_strokes_in_frame(C, obact, gpl, gpf);
  }
  return strokes_count;
}

static void gpencil_xform_box_custom_draw_free_buffer(XFormBoxWidgetGroup *xgzgroup)
{
  if (xgzgroup->custom_draw.batch != NULL) {
    GPU_batch_discard(xgzgroup->custom_draw.batch);
    xgzgroup->custom_draw.batch = NULL;
  }
}

static void gpencil_xform_box_custom_draw_allocate_buffer(XFormBoxWidgetGroup *xgzgroup)
{
  GPUVertFormat *format = immVertexFormat();
  xgzgroup->custom_draw.pos = GPU_vertformat_attr_add(
      format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  int tot_verts = 0;
  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];
      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        /* Only consider last point if the stroke is cyclic, as we will need a segment
         * from last to first point. */
        tot_verts += gps->flag & GP_STROKE_CYCLIC ? gps->totpoints : gps->totpoints - 1;
      }
    }
  }

  xgzgroup->custom_draw.vert = GPU_vertbuf_create_with_format(format);
  GPU_vertbuf_data_alloc(xgzgroup->custom_draw.vert, tot_verts * 2);

  xgzgroup->custom_draw.batch = GPU_batch_create_ex(
      GPU_PRIM_LINES, xgzgroup->custom_draw.vert, NULL, GPU_BATCH_OWNS_VBO);
}

static void gpencil_xform_box_custom_draw_populate_buffer(XFormBoxWidgetGroup *xgzgroup)
{
  gpencil_xform_box_custom_draw_free_buffer(xgzgroup);
  gpencil_xform_box_custom_draw_allocate_buffer(xgzgroup);

  int idx = 0;
  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];

      /* Compute world matrix (frame + layer + object transform). */
      float diff_mat[4][4];
      mul_m4_series(diff_mat, xgzgroup->diff_mat, tld->diff_mat, tfd->diff_mat);

      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        for (int p = 0; p < gps->totpoints; p++) {
          /* Discard last point for non cyclic strokes, as it is the end of previous segment. */
          if (p == gps->totpoints - 1 && ((gps->flag & GP_STROKE_CYCLIC) == 0)) {
            continue;
          }
          tGPTransData *td_curr = &tfd->trans_data[k][p];
          /* Handle cyclic strokes. */
          const int p_next = (p + 1) % gps->totpoints;
          tGPTransData *td_next = &tfd->trans_data[k][p_next];
          float curr[3], next[3];

          /* Apply world matrix to points. */
          mul_v3_m4v3(curr, diff_mat, td_curr->loc);
          mul_v3_m4v3(next, diff_mat, td_next->loc);

          GPU_vertbuf_attr_set(
              xgzgroup->custom_draw.vert, xgzgroup->custom_draw.pos, idx * 2, curr);
          GPU_vertbuf_attr_set(
              xgzgroup->custom_draw.vert, xgzgroup->custom_draw.pos, (idx * 2) + 1, next);
          idx++;
        }
      }
    }
  }
}

static void gpencil_xform_box_custom_draw(const bContext *C, ARegion *UNUSED(region), void *arg)
{
  wmGizmoGroupType *gzgt = WM_gizmogrouptype_find("VIEW3D_GGT_gpencil_xform_box", false);
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)arg;
  View3D *v3d = CTX_wm_view3d(C);

  /* Don't show overlay if the gizmogroup's poll function returns false. */
  if (!gzgt->poll(C, gzgt)) {
    return;
  }

  if ((v3d->flag2 & V3D_HIDE_OVERLAYS) != 0) {
    return;
  }

  if (xgzgroup->custom_draw.batch == NULL) {
    return;
  }

  /* Don't show the overlay if the animation is playing. */
  if (ED_screen_animation_playing(CTX_wm_manager(C))) {
    return;
  }

  GPU_blend(GPU_BLEND_ALPHA);
  GPU_batch_program_set_builtin(xgzgroup->custom_draw.batch, GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);

  float color[4] = {0, 0, 0, 1.0f};
  UI_GetThemeColorType3fv(TH_GP_VERTEX_SELECT, SPACE_VIEW3D, color);
  GPU_batch_uniform_4fv(xgzgroup->custom_draw.batch, "color", color);

  float viewport[4];
  GPU_viewport_size_get_f(viewport);
  GPU_batch_uniform_2fv(xgzgroup->custom_draw.batch, "viewportSize", &viewport[2]);
  GPU_batch_uniform_1f(xgzgroup->custom_draw.batch, "lineWidth", U.pixelsize);

  GPU_batch_draw(xgzgroup->custom_draw.batch);

  GPU_blend(GPU_BLEND_NONE);
}

static void gpencil_xform_box_update_strokes(XFormBoxWidgetGroup *xgzgroup,
                                             const float transformation_matrix[4][4])
{
  float scale[2] = {1.0f, 1.0f};
  RNA_float_get_array(xgzgroup->gizmo->ptr, "scale", scale);

  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];

      /* Compute frame + layer inverse transformation matrix. */
      float diff_inv[4][4];
      mul_m4_m4m4(diff_inv, tfd->diff_inv, tld->diff_inv);

      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        for (int p = 0; p < gps->totpoints; p++) {
          tGPTransData *td = &tfd->trans_data[k][p];

          /* Update the position of the stroke point. */
          mul_v3_m4v3(td->loc, transformation_matrix, td->iloc);
          /* Unapply frame + layer transform contained in iloc. */
          mul_m4_v3(diff_inv, td->loc);

          /* Update the thickness of the stroke point. */
          float fac = fabsf(scale[0] * scale[1]);
          *td->val = td->ival * sqrtf(fac);
        }
        BKE_gpencil_tag_full_update(xgzgroup->gpd, tld->layer, tfd->frame, gps);
        BKE_gpencil_stroke_geometry_update(xgzgroup->gpd, gps);
      }
    }
  }
}

static void free_transform_layers(XFormBoxWidgetGroup *xgzgroup)
{
  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];
      for (int k = 0; k < tfd->tot_strokes; k++) {
        MEM_freeN(tfd->trans_data[k]);
      }
      MEM_freeN(tfd->trans_data);
      MEM_freeN(tfd->strokes);
    }
    MEM_freeN(tld->frames);
  }
  MEM_SAFE_FREE(xgzgroup->layers);
}

static void gpencil_xform_box_customdata_free(void *data)
{
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)data;
  free_transform_layers(xgzgroup);
  gpencil_xform_box_custom_draw_free_buffer(xgzgroup);
  ED_region_draw_cb_exit(xgzgroup->region->type, xgzgroup->custom_draw.handle);
  MEM_SAFE_FREE(xgzgroup);
}

static void gpencil_xform_box_calculate_bounding_box(XFormBoxWidgetGroup *xgzgroup,
                                                     rctf *r_rect,
                                                     float *r_dx,
                                                     float *r_dy)
{
  BLI_rctf_init_minmax(r_rect);
  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];
      /* Compute frame + layer transformation matrix. */
      float diff_mat[4][4];
      mul_m4_m4m4(diff_mat, tld->diff_mat, tfd->diff_mat);

      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        for (int p = 0; p < gps->totpoints; p++) {
          bGPDspoint *pt = &gps->points[p];
          float pt_loc_space[3];
          mul_v3_m4v3(pt_loc_space, diff_mat, &pt->x);
          float co[2];
          local_space_to_transform_plane_space(&xgzgroup->plane, pt_loc_space, co);
          BLI_rctf_do_minmax_v(r_rect, co);
        }
      }
    }
  }

  const float dx = BLI_rctf_cent_x(r_rect);
  const float dy = BLI_rctf_cent_y(r_rect);
  BLI_rctf_translate(r_rect, -dx, -dy);

  *r_dx = dx;
  *r_dy = dy;
}

static void gpencil_xform_box_init_trans_data(XFormBoxWidgetGroup *xgzgroup, float start_offset[2])
{
  float start_offset_local[3];
  transform_plane_space_to_local_space(&xgzgroup->plane, start_offset, start_offset_local);

  for (int i = 0; i < xgzgroup->tot_layers; i++) {
    tGPTransformLayerData *tld = &xgzgroup->layers[i];
    for (int j = 0; j < tld->tot_frames; j++) {
      tGPTransformFrameData *tfd = &tld->frames[j];
      /* Compute frame + layer transformation matrix. */
      float diff_mat[4][4];
      mul_m4_m4m4(diff_mat, tld->diff_mat, tfd->diff_mat);

      for (int k = 0; k < tfd->tot_strokes; k++) {
        bGPDstroke *gps = tfd->strokes[k];
        for (int p = 0; p < gps->totpoints; p++) {
          bGPDspoint *pt = &gps->points[p];
          tGPTransData *td = &tfd->trans_data[k][p];
          td->loc = &pt->x;
          /* Store initial location with frame and layer transform applied. */
          mul_v3_m4v3(td->iloc, diff_mat, &pt->x);
          /* Cancel start offset out for rotation to be centered. */
          if (start_offset != NULL) {
            sub_v3_v3(td->iloc, start_offset_local);
          }
          td->val = &pt->pressure;
          td->ival = pt->pressure;
        }
      }
    }
  }
}

static void gpencil_xform_box_create_trans_data(const bContext *C, XFormBoxWidgetGroup *xgzgroup)
{
  /* Count number of layers to be used by the transformation. */
  int layer_count = 0;
  /* Whether multiframe edit is active.*/
  const bool is_multiedit = GPENCIL_MULTIEDIT_SESSIONS_ON(xgzgroup->gpd);

  LISTBASE_FOREACH (bGPDlayer *, gpl, &xgzgroup->gpd->layers) {
    if (!BKE_gpencil_layer_is_editable(gpl)) {
      continue;
    }
    if (count_selected_strokes_in_layer(C, xgzgroup->ob, gpl) > 0) {
      layer_count++;
    }
  }

  const bool use_frame_offset = GP_USE_FRAME_OFFSET_MATRIX(CTX_data_scene(C), xgzgroup->gpd);

  xgzgroup->tot_layers = layer_count;
  xgzgroup->layers = MEM_callocN(xgzgroup->tot_layers * sizeof(*xgzgroup->layers), __func__);

  int tld_idx = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &xgzgroup->gpd->layers) {
    if (!BKE_gpencil_layer_is_editable(gpl)) {
      continue;
    }
    /* FIXME: This check is not ideal, find a better way to handle this. */
    if (count_selected_strokes_in_layer(C, xgzgroup->ob, gpl) <= 0) {
      continue;
    }

    bGPDframe *gpf = gpl->actframe;
    tGPTransformLayerData *tld = &xgzgroup->layers[tld_idx];
    tld->layer = gpl;

    BKE_gpencil_layer_transform_matrix_get(xgzgroup->depsgraph, xgzgroup->ob, gpl, tld->diff_mat);

    /* Save the layer matrix in object space to the transform layer data. */
    mul_m4_m4_pre(tld->diff_mat, xgzgroup->diff_inv);
    invert_m4_m4(tld->diff_inv, tld->diff_mat);

    tld->tot_frames = count_active_frames(gpl, is_multiedit);
    /* Layers without active frames should have been filtered by now. */
    BLI_assert(tld->tot_frames > 0);

    tld->frames = MEM_callocN(tld->tot_frames * sizeof(*tld->frames), __func__);
    int tfd_idx = 0;

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      if (!is_frame_active(gpl, gpf, is_multiedit)) {
        continue;
      }

      tGPTransformFrameData *tfd = &tld->frames[tfd_idx];
      tfd->frame = gpf;
      if (use_frame_offset) {
        copy_m4_m4(tfd->diff_mat, gpf->transformation_mat);
        invert_m4_m4(tfd->diff_inv, tfd->diff_mat);
      }
      else {
        unit_m4(tfd->diff_mat);
        unit_m4(tfd->diff_inv);
      }

      const int strokes_count = count_selected_strokes_in_frame(C, xgzgroup->ob, gpl, tfd->frame);
      /* Allocate transform strokes pointer array. */
      tfd->tot_strokes = strokes_count;
      tfd->strokes = MEM_callocN(tfd->tot_strokes * sizeof(*tfd->strokes), __func__);
      tfd->trans_data = MEM_callocN(tfd->tot_strokes * sizeof(*tfd->trans_data), __func__);

      int stroke_idx = 0;
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        /* skip strokes that are invalid for current view */
        if (ED_gpencil_stroke_can_use(C, gps) == false) {
          continue;
        }
        /* check if the color is editable */
        if (ED_gpencil_stroke_material_editable(xgzgroup->ob, gpl, gps) == false) {
          continue;
        }

        if (gps->flag & GP_STROKE_SELECT) {
          /* Store pointer to selected strokes. */
          tfd->strokes[stroke_idx] = gps;
          /* Allocate space for transform data. */
          tfd->trans_data[stroke_idx] = MEM_callocN(gps->totpoints * sizeof(tGPTransData),
                                                    __func__);
          stroke_idx++;
        }
      }
      tfd_idx++;
    }
    tld_idx++;
  }
}

static void gizmo_gpencil_xform_box_callback_get(const wmGizmo *gz,
                                                 wmGizmoProperty *gz_prop,
                                                 void *value)
{
  float(*matrix)[3] = value;
}

static void gizmo_gpencil_xform_box_callback_set(const wmGizmo *gz,
                                                 wmGizmoProperty *gz_prop,
                                                 void *value)
{
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)gz->parent_gzgroup->customdata;
  float(*matrix)[4] = value;
  gpencil_xform_box_update_strokes(xgzgroup, matrix);

  DEG_id_tag_update(&xgzgroup->gpd->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(xgzgroup->C, NC_GPENCIL | ND_DATA | NA_EDITED, NULL);
}

static void gpencil_xform_box_update_settings(const bContext *C, XFormBoxWidgetGroup *xgzgroup)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Scene *scene = CTX_data_scene(C);
  ARegion *region = CTX_wm_region(C);
  Object *ob = CTX_data_active_object(C);
  bGPdata *gpd = CTX_data_gpencil_data(C);
  ToolSettings *ts = CTX_data_tool_settings(C);
  wmGizmo *gz = xgzgroup->gizmo;

  xgzgroup->C = C;
  xgzgroup->depsgraph = depsgraph;
  xgzgroup->ob = ob;
  xgzgroup->gpd = gpd;
  xgzgroup->ts = ts;
  xgzgroup->region = region;

  /* Make this gizmo work in object space. */
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, ob);
  copy_m4_m4(gz->matrix_basis, ob_eval->obmat);
  copy_m4_m4(xgzgroup->diff_mat, gz->matrix_basis);
  invert_m4_m4(xgzgroup->diff_inv, xgzgroup->diff_mat);
}

static void gpencil_xform_box_initialize_gizmo(const bContext *C, XFormBoxWidgetGroup *xgzgroup)
{
  wmGizmo *gz = xgzgroup->gizmo;
  gpencil_xform_box_create_trans_data(C, xgzgroup);

  if (xgzgroup->tot_layers <= 0) {
    gpencil_xform_box_custom_draw_free_buffer(xgzgroup);
    WM_gizmo_set_flag(gz, WM_GIZMO_HIDDEN, true);
    return;
  }

  float start_offset[2] = {0.0f, 0.0f};
  float start_pivot[2] = {0.0f, 0.0f};
  float start_scale[2] = {1.0f, 1.0f};
  float dims[2] = {1.0f, 1.0f};

  rctf rect;
  gpencil_xform_box_calculate_bounding_box(xgzgroup, &rect, &start_offset[0], &start_offset[1]);
  dims[0] = BLI_rctf_size_x(&rect);
  dims[1] = BLI_rctf_size_y(&rect);

  gpencil_xform_box_init_trans_data(xgzgroup, start_offset);

  RNA_float_set_array(gz->ptr, "offset", start_offset);
  RNA_float_set_array(gz->ptr, "dimensions", dims);
  RNA_float_set(gz->ptr, "angle", 0.0f);
  RNA_float_set_array(gz->ptr, "scale", start_scale);
  RNA_float_set_array(gz->ptr, "pivot", start_pivot);
  RNA_boolean_set(gz->ptr, "force_update", true);

  gpencil_xform_box_custom_draw_populate_buffer(xgzgroup);

  WM_gizmo_set_flag(gz, WM_GIZMO_HIDDEN, false);
}

static void gpencil_xform_box_custom_draw_exit(const bContext *C,
                                               wmMsgSubscribeKey *UNUSED(msg_key),
                                               wmMsgSubscribeValue *msg_val)
{
  ARegion *region = (ARegion *)msg_val->owner;
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)msg_val->user_data;

  gpencil_xform_box_custom_draw_free_buffer(xgzgroup);
  if (ED_region_draw_cb_exit(region->type, xgzgroup->custom_draw.handle)) {
    xgzgroup->custom_draw.handle = NULL;
  }
}

static bool WIDGETGROUP_gpencil_generic_poll(const bContext *C, wmGizmoGroupType *gzgt)
{
  if (!ED_gizmo_poll_or_unlink_delayed_from_tool(C, gzgt)) {
    return false;
  }
  View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_TOOL)) {
    return false;
  }

  bGPdata *gpd = CTX_data_gpencil_data(C);
  if (!gpd || !BKE_gpencil_layer_active_get(gpd)) {
    return false;
  }
  if (!GPENCIL_PAINT_MODE(gpd)) {
    return false;
  }
  return true;
}

static void WIDGETGROUP_gpencil_xform_box_setup(const bContext *C, wmGizmoGroup *gzgroup)
{
  XFormBoxWidgetGroup *xgzgroup = MEM_callocN(sizeof(XFormBoxWidgetGroup), __func__);
  const wmGizmoType *gzt_xplane = WM_gizmotype_find("GIZMO_GT_xform_plane3d", true);
  xgzgroup->gizmo = WM_gizmo_new_ptr(gzt_xplane, gzgroup, NULL);
  wmGizmo *gz = xgzgroup->gizmo;

  /* Set the color of the gizmo. */
  UI_GetThemeColorType3fv(TH_GP_VERTEX_SELECT, SPACE_VIEW3D, gz->color);

  Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = CTX_data_tool_settings(C);

  gzgroup->customdata = xgzgroup;
  gzgroup->customdata_free = gpencil_xform_box_customdata_free;
  xgzgroup->gizmo_active = false;

  gpencil_xform_box_update_settings(C, xgzgroup);

  int lock_axis = ts->gp_sculpt.lock_axis;

  /* TODO: Support cursor alignment and view aligment. */
#if 0
  float origin[3];
  const View3DCursor *cursor = &scene->cursor;
  char align_flag = ts->gpencil_v3d_align;
  if (align_flag & GP_PROJECT_CURSOR) {
    copy_v3_v3(origin, cursor->location);
    mul_v3_m4v3(xgzgroup->plane.co, diff_inv, origin);
  }
  else {
    copy_v3_v3(origin, ob->obmat[3]);
    // mul_v3_m4v3(xgzgroup->plane.co, diff_inv, origin);
    zero_v3(xgzgroup->plane.co);
  }
#endif
  zero_v3(xgzgroup->plane.co);

  if (ELEM(lock_axis, GP_LOCKAXIS_X, GP_LOCKAXIS_Y, GP_LOCKAXIS_Z)) {
    /* AXIS X, Y or Z*/
    zero_v3(xgzgroup->plane.no);
    xgzgroup->plane.no[lock_axis - 1] = 1.0f;
  }

  RNA_float_set_array(gz->ptr, "normal", xgzgroup->plane.no);

  /* Calculate local U and V basis vectors. */
  ortho_basis_v3v3_v3(xgzgroup->plane.u, xgzgroup->plane.v, xgzgroup->plane.no);
  plane_from_point_normal_v3(xgzgroup->plane.eq, xgzgroup->plane.co, xgzgroup->plane.no);

  xgzgroup->custom_draw.batch = NULL;
  gpencil_xform_box_initialize_gizmo(C, xgzgroup);
  xgzgroup->selection_hash = create_selection_hash(xgzgroup);

  WM_gizmo_target_property_def_func(gz,
                                    "matrix",
                                    &(const struct wmGizmoPropertyFnParams){
                                        .value_get_fn = gizmo_gpencil_xform_box_callback_get,
                                        .value_set_fn = gizmo_gpencil_xform_box_callback_set,
                                        .range_get_fn = NULL,
                                        .user_data = NULL,
                                    });

  xgzgroup->custom_draw.handle = ED_region_draw_cb_activate(
      xgzgroup->region->type, gpencil_xform_box_custom_draw, xgzgroup, REGION_DRAW_POST_VIEW);
  ED_region_draw_cb_draw(C, xgzgroup->region, REGION_DRAW_POST_VIEW);
}

static void WIDGETGROUP_gpencil_xform_box_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  ARegion *region = CTX_wm_region(C);
  RegionView3D *rv3d = region->regiondata;

  XFormBoxWidgetGroup *xgzgroup = gzgroup->customdata;
  if (xgzgroup->gizmo_active) {
    return;
  }

  gpencil_xform_box_update_settings(C, xgzgroup);

  size_t current_hash = get_current_selection_hash(C, xgzgroup);
  /* If the selection has changed, we need to reset the bounding box. */
  if (xgzgroup->selection_hash != current_hash) {
    /* Reset trans data. */
    free_transform_layers(xgzgroup);
    gpencil_xform_box_initialize_gizmo(C, xgzgroup);
    xgzgroup->selection_hash = current_hash;
  }
  gpencil_xform_box_custom_draw_populate_buffer(xgzgroup);

  /* Needed to test view orientation changes. */
  copy_m3_m4(xgzgroup->prev.viewinv_m3, rv3d->viewinv);
}

static void WIDGETGROUP_gpencil_xform_invoke_prepare(const bContext *C,
                                                     wmGizmoGroup *gzgroup,
                                                     wmGizmo *gizmo,
                                                     const wmEvent *event)
{
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)gzgroup->customdata;
  xgzgroup->gizmo_active = true;
}

static void WIDGETGROUP_gpencil_xform_box_exit_cleanup(const bContext *C,
                                                       wmGizmoGroup *gzgroup,
                                                       wmGizmo *gz,
                                                       bool cancel)
{
  XFormBoxWidgetGroup *xgzgroup = (XFormBoxWidgetGroup *)gzgroup->customdata;
  xgzgroup->gizmo_active = false;
  gpencil_xform_box_custom_draw_populate_buffer(xgzgroup);
}

void VIEW3D_GGT_gpencil_xform_box(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Grease Pencil Quick Edit";
  gzgt->idname = "VIEW3D_GGT_gpencil_xform_box";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_SCALE);

  gzgt->gzmap_params.spaceid = SPACE_VIEW3D;
  gzgt->gzmap_params.regionid = RGN_TYPE_WINDOW;

  gzgt->poll = WIDGETGROUP_gpencil_generic_poll;
  gzgt->invoke_prepare = WIDGETGROUP_gpencil_xform_invoke_prepare;
  gzgt->setup = WIDGETGROUP_gpencil_xform_box_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_gpencil_xform_box_refresh;
  gzgt->exit_cleanup = WIDGETGROUP_gpencil_xform_box_exit_cleanup;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shift and Trace Gizmo
 * \{ */

typedef struct FrameOffsetWidgetGroup {
  wmGizmo *gz;

  bContext *C;
  Object *ob;
  Scene *scene;
  ViewLayer *view_layer;

  PropertyRNA *mode_prop;
  PropertyRNA *frame_prop;

  bGPdata *gpd;
  /* Pointer to the layer. */
  bGPDlayer *layer;
  /* Pointer to the active frame. */
  bGPDframe *actframe;

  /* Transformation matrix for the layer. */
  float diff_mat[4][4];
  /* Inverse matrix of `diff_mat`. */
  float diff_inv[4][4];

  tTransformPlane plane;

  float start_offset[2];
  float pivot[2];

  bool gizmo_active;
  bool use_current_frame;
  int frame;

  PointerRNA ptr;
} FrameOffsetWidgetGroup;

static void gpencil_frame_offset_group_free(void *data)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)data;
  MEM_SAFE_FREE(frameoffset_gzgroup);
}

typedef struct tGPIterData {
  bGPDlayer *find_layer;
  bGPDframe *find_frame;
  bool found;
} tGPIterData;

static void gpencil_frame_offset_get_frame_from_layer_cb(bGPDlayer *layer,
                                                         bGPDframe *frame,
                                                         bGPDstroke *UNUSED(stroke),
                                                         void *thunk)
{
  tGPIterData *data = (tGPIterData *)thunk;
  if (data->find_layer == layer && data->find_frame == frame) {
    data->found = true;
  }
}

static bGPDframe *gpencil_frame_offset_get_frame_from_layer(
    FrameOffsetWidgetGroup *frameoffset_gzgroup, bGPDlayer *gpl)
{
  /* TODO: Should we handle time offset modifiers? */
  bGPDframe *gpf = frameoffset_gzgroup->use_current_frame ?
                       gpl->actframe :
                       BKE_gpencil_layer_frame_find_prev(gpl, frameoffset_gzgroup->frame);

  /* If the frame has no strokes, it should not be visible. */
  if (gpf == NULL || BLI_listbase_is_empty(&gpf->strokes)) {
    return NULL;
  }

  tGPIterData iter_data = {.find_layer = gpl, .find_frame = gpf, .found = false};

  int cfra = frameoffset_gzgroup->scene->r.cfra;
  View3D *v3d = CTX_wm_view3d(frameoffset_gzgroup->C);

  BKE_gpencil_visible_stroke_advanced_iter(frameoffset_gzgroup->view_layer,
                                           frameoffset_gzgroup->ob,
                                           gpencil_frame_offset_get_frame_from_layer_cb,
                                           NULL,
                                           &iter_data,
                                           (v3d->gp_flag & V3D_GP_SHOW_ONION_SKIN),
                                           cfra);

  return iter_data.found ? gpf : NULL;
}

static bool gpencil_frame_offset_use_layer(FrameOffsetWidgetGroup *frameoffset_gzgroup,
                                           bGPDlayer *gpl)
{
  if ((gpl->flag & GP_LAYER_ACTIVE) == 0) {
    return false;
  }
  if (!BKE_gpencil_layer_is_editable(gpl)) {
    return false;
  }
  if (gpencil_frame_offset_get_frame_from_layer(frameoffset_gzgroup, gpl) == NULL) {
    return false;
  }
  return true;
}

static bool gpencil_update_active_frame(FrameOffsetWidgetGroup *frameoffset_gzgroup)
{
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(frameoffset_gzgroup->gpd);
  if (gpl == NULL || !BKE_gpencil_layer_is_editable(gpl)) {
    return false;
  }

  if (!gpencil_frame_offset_use_layer(frameoffset_gzgroup, gpl)) {
    return false;
  }

  bGPDframe *gpf = gpencil_frame_offset_get_frame_from_layer(frameoffset_gzgroup, gpl);

  /* Early return if active layer and frame have not changed. */
  if (frameoffset_gzgroup->layer == gpl && frameoffset_gzgroup->actframe == gpf) {
    return true;
  }

  if (frameoffset_gzgroup->layer != NULL) {
    zero_v2(frameoffset_gzgroup->pivot);
  }

  frameoffset_gzgroup->layer = gpl;
  /* TODO: Get diff_mat here. */
  frameoffset_gzgroup->actframe = gpf;

  return true;
}

static void gpencil_frame_calculate_bounding_box(FrameOffsetWidgetGroup *frameoffset_gzgroup,
                                                 tTransformPlane *plane,
                                                 rctf *r_rect)
{
  BLI_rctf_init_minmax(r_rect);
  bGPDframe *gpf = frameoffset_gzgroup->actframe;
  LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
    for (int i = 0; i < gps->totpoints; i++) {
      bGPDspoint *pt = &gps->points[i];
      float ploc[2];
      local_space_to_transform_plane_space(plane, &pt->x, ploc);
      BLI_rctf_do_minmax_v(r_rect, ploc);
    }
  }
}

static void gizmo_gpencil_frame_offset_matrix_callback_set(const wmGizmo *gz,
                                                           wmGizmoProperty *gz_prop,
                                                           void *value)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)
                                                    gz->parent_gzgroup->customdata;
  float matrix[4][4];
  copy_m4_m4(matrix, value);

  float scale[2], transformed_offset[2], transformed_offset_proj[3];
  float angle = RNA_float_get(gz->ptr, "angle");
  RNA_float_get_array(gz->ptr, "scale", scale);

  /* Undo the initial frame transformation on the start_offset. */
  mul_v2_v2v2(transformed_offset, scale, frameoffset_gzgroup->start_offset);
  rotate_v2_fl(transformed_offset, angle);

  /* Project transformed_offset in local space. */
  copy_v3_v3(transformed_offset_proj, frameoffset_gzgroup->plane.co);
  madd_v3_v3fl(transformed_offset_proj, frameoffset_gzgroup->plane.u, transformed_offset[0]);
  madd_v3_v3fl(transformed_offset_proj, frameoffset_gzgroup->plane.v, transformed_offset[1]);

  /* Remove the (transformed) start_offset from matrix. */
  sub_v3_v3(matrix[3], transformed_offset_proj);

  /* Bring the transformation into layerspace, not object space. */
  mul_m4_series(matrix, frameoffset_gzgroup->diff_mat, matrix, frameoffset_gzgroup->diff_inv);

  bGPDlayer *gpl = frameoffset_gzgroup->layer;
  bGPDframe *gpf = frameoffset_gzgroup->actframe;
  PointerRNA gpf_rna_ptr;
  RNA_pointer_create(&frameoffset_gzgroup->gpd->id, &RNA_GPencilFrame, gpf, &gpf_rna_ptr);
  RNA_float_set_array(&gpf_rna_ptr, "offset", matrix);

  /* Tag update. */
  float unitm4[4][4];
  unit_m4(unitm4);
  if (!equals_m4m4(unitm4, gpf->transformation_mat)) {
    BKE_gpencil_tag_light_update(frameoffset_gzgroup->gpd, gpl, gpf, NULL);
  }
  else {
    BKE_gpencil_tag_full_update(frameoffset_gzgroup->gpd, gpl, gpf, NULL);
  }

  DEG_id_tag_update(&frameoffset_gzgroup->gpd->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(frameoffset_gzgroup->C, NC_GPENCIL | NA_EDITED, NULL);
}

static void gpencil_frame_offset_update_gizmo(FrameOffsetWidgetGroup *frameoffset_gzgroup)
{
  wmGizmo *gz = frameoffset_gzgroup->gz;
  if (!gpencil_update_active_frame(frameoffset_gzgroup)) {
    WM_gizmo_set_flag(gz, WM_GIZMO_HIDDEN, true);
    return;
  }

  bGPDlayer *gpl = BKE_gpencil_layer_active_get(frameoffset_gzgroup->gpd);
  bGPDframe *gpf = gpencil_frame_offset_get_frame_from_layer(frameoffset_gzgroup, gpl);
  if (gpf == NULL) {
    WM_gizmo_set_flag(gz, WM_GIZMO_HIDDEN, true);
    return;
  }

  rctf bbox;
  float dims[2], start_offset[2];
  gpencil_frame_calculate_bounding_box(frameoffset_gzgroup, &frameoffset_gzgroup->plane, &bbox);
  const float dx = BLI_rctf_cent_x(&bbox);
  const float dy = BLI_rctf_cent_y(&bbox);

  dims[0] = BLI_rctf_size_x(&bbox);
  dims[1] = BLI_rctf_size_y(&bbox);

  start_offset[0] = dx;
  start_offset[1] = dy;
  copy_v2_v2(frameoffset_gzgroup->start_offset, start_offset);

  /* Apply frame transformation. */
  float offset[2];
  mul_v2_v2v2(offset, gpf->scale, start_offset);
  rotate_v2_fl(offset, gpf->angle);
  add_v2_v2(offset, gpf->offset);

  RNA_float_set_array(gz->ptr, "offset", offset);
  RNA_float_set_array(gz->ptr, "dimensions", dims);
  RNA_float_set(gz->ptr, "angle", gpf->angle);
  RNA_float_set_array(gz->ptr, "scale", gpf->scale);
  RNA_float_set_array(gz->ptr, "pivot", frameoffset_gzgroup->pivot);

  RNA_boolean_set(gz->ptr, "force_update", true);

  WM_gizmo_target_property_def_func(
      gz,
      "matrix",
      &(const struct wmGizmoPropertyFnParams){
          .value_get_fn = gizmo_gpencil_xform_box_callback_get,
          .value_set_fn = gizmo_gpencil_frame_offset_matrix_callback_set,
          .range_get_fn = NULL,
          .user_data = NULL,
      });

  WM_gizmo_set_flag(gz, WM_GIZMO_HIDDEN, false);
}

static void gpencil_frame_offset_update_diff_mat(const bContext *C,
                                                 FrameOffsetWidgetGroup *frameoffset_gzgroup)
{
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Object *ob = CTX_data_active_object(C);
  Object *ob_eval = (depsgraph != NULL) ? DEG_get_evaluated_object(depsgraph, ob) : ob;
  bGPdata *gpd = CTX_data_gpencil_data(C);

  float diff_mat[4][4], parent_inv[4][4];
  unit_m4(diff_mat);
  unit_m4(parent_inv);

  if (gpencil_update_active_frame(frameoffset_gzgroup) &&
      frameoffset_gzgroup->actframe != frameoffset_gzgroup->layer->actframe &&
      gpd->onion_space == GP_ONION_SPACE_WORLD) {
    bGPDframe *gpf_orig = frameoffset_gzgroup->actframe->runtime.gpf_orig ?
                              frameoffset_gzgroup->actframe->runtime.gpf_orig :
                              frameoffset_gzgroup->actframe;
    if (gpf_orig && gpf_orig->runtime.transform_valid) {
      copy_m4_m4(diff_mat, gpf_orig->runtime.transform);
      invert_m4_m4(parent_inv, diff_mat);
    }
  }
  else if (frameoffset_gzgroup->layer != NULL) {
    BKE_gpencil_layer_transform_matrix_get(depsgraph, ob, frameoffset_gzgroup->layer, diff_mat);
    BKE_gpencil_layer_parent_matrix_get(depsgraph, ob, frameoffset_gzgroup->layer, parent_inv);
    invert_m4(parent_inv);
  }

  copy_m4_m4(frameoffset_gzgroup->gz->matrix_basis, diff_mat);

  /* Get the matrix to layer space by applying the object inverse matrix from the diff matrix */
  mul_m4_m4m4(frameoffset_gzgroup->diff_mat, parent_inv, diff_mat);
  invert_m4_m4(frameoffset_gzgroup->diff_inv, frameoffset_gzgroup->diff_mat);
}

static bool WIDGETGROUP_gpencil_frame_offset_poll(const bContext *C, wmGizmoGroupType *gzgt)
{
  if (!WIDGETGROUP_gpencil_generic_poll(C, gzgt)) {
    return false;
  }
  Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = scene->toolsettings;
  if ((ts->gpencil_flags & GP_TOOL_FLAG_USE_FRAME_OFFSET_MATRIX) == 0) {
    return false;
  }
  return true;
}

static void WIDGETGROUP_gpencil_frame_offset_setup(const bContext *C, wmGizmoGroup *gzgroup)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = MEM_callocN(sizeof(FrameOffsetWidgetGroup),
                                                            __func__);
  const wmGizmoType *gzt_xplane = WM_gizmotype_find("GIZMO_GT_xform_plane3d", true);
  frameoffset_gzgroup->gz = WM_gizmo_new_ptr(gzt_xplane, gzgroup, NULL);

  gzgroup->customdata = frameoffset_gzgroup;
  gzgroup->customdata_free = gpencil_frame_offset_group_free;

  frameoffset_gzgroup->C = C;
  frameoffset_gzgroup->ob = CTX_data_active_object(C);
  frameoffset_gzgroup->scene = CTX_data_scene(C);
  frameoffset_gzgroup->view_layer = CTX_data_view_layer(C);
  frameoffset_gzgroup->gpd = CTX_data_gpencil_data(C);

  bToolRef *tref = WM_toolsystem_ref_from_context((bContext *)C);
  WM_toolsystem_ref_properties_ensure_from_gizmo_group(
      tref, gzgroup->type, &frameoffset_gzgroup->ptr);

  wmGizmo *gz = frameoffset_gzgroup->gz;
  UI_GetThemeColorType3fv(TH_TRANSFORM, SPACE_VIEW3D, gz->color);

  Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = scene->toolsettings;
  int lock_axis = ts->gp_sculpt.lock_axis;

  /* TODO: Support cursor alignment and view aligment. */
#if 0
  float origin[3];
  const View3DCursor *cursor = &scene->cursor;
  char align_flag = ts->gpencil_v3d_align;
  if (align_flag & GP_PROJECT_CURSOR) {
    copy_v3_v3(origin, cursor->location);
    mul_v3_m4v3(xgzgroup->plane.co, diff_inv, origin);
  }
  else {
    copy_v3_v3(origin, ob->obmat[3]);
    // mul_v3_m4v3(xgzgroup->plane.co, diff_inv, origin);
    zero_v3(xgzgroup->plane.co);
  }
#endif
  // zero_v3(xgzgroup->plane.co);
  float plane_normal[3], plane_co[3], plane_u[3], plane_v[3];
  zero_v3(plane_co);
  zero_v3(plane_normal);
  if (ELEM(lock_axis, GP_LOCKAXIS_X, GP_LOCKAXIS_Y, GP_LOCKAXIS_Z)) {
    /* AXIS X, Y or Z*/
    // zero_v3(xgzgroup->plane.no);
    // xgzgroup->plane.no[lock_axis - 1] = 1.0f;
    plane_normal[lock_axis - 1] = 1.0f;
  }

  /* Calculate local U and V basis vectors. */
  ortho_basis_v3v3_v3(plane_u, plane_v, plane_normal);

  RNA_float_set_array(gz->ptr, "normal", plane_normal);
  RNA_boolean_set(gz->ptr, "use_skew", false);

  copy_v3_v3(frameoffset_gzgroup->plane.co, plane_co);
  copy_v3_v3(frameoffset_gzgroup->plane.u, plane_u);
  copy_v3_v3(frameoffset_gzgroup->plane.v, plane_v);

  gpencil_frame_offset_update_diff_mat(C, frameoffset_gzgroup);
  gpencil_frame_offset_update_gizmo(frameoffset_gzgroup);
}

static void WIDGETGROUP_gpencil_frame_offset_invoke_prepare(const bContext *C,
                                                            wmGizmoGroup *gzgroup,
                                                            wmGizmo *gizmo,
                                                            const wmEvent *event)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)gzgroup->customdata;
  frameoffset_gzgroup->gizmo_active = true;
}

static void WIDGETGROUP_gpencil_frame_offset_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)gzgroup->customdata;

  /* When interacting with the gizmo, avoid repeatedly initiliazing the transform components
   * on the gizmo as they are already in sync.*/
  if (frameoffset_gzgroup->gizmo_active) {
    return;
  }

  Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = scene->toolsettings;
  frameoffset_gzgroup->use_current_frame = ts->gp_frame_offset.use_current_frame;
  frameoffset_gzgroup->frame = ts->gp_frame_offset.custom_frame;

  gpencil_frame_offset_update_diff_mat(C, frameoffset_gzgroup);
  gpencil_frame_offset_update_gizmo(frameoffset_gzgroup);
}

static void WIDGETGROUP_gpencil_frame_offset_message_subscribe(const bContext *C,
                                                               wmGizmoGroup *gzgroup,
                                                               struct wmMsgBus *mbus)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)gzgroup->customdata;
  ARegion *region = CTX_wm_region(C);
  wmMsgSubscribeValue msg_sub_value_gz_tag_refresh = {
      .owner = region,
      .user_data = gzgroup->parent_gzmap,
      .notify = WM_gizmo_do_msg_notify_tag_refresh,
  };

#define WM_msg_subscribe_rna_anon_custom_prop(mbus, type_, prop_, value) \
  { \
    PointerRNA msg_ptr_ = {0, type_}; \
    wmMsgParams_RNA msg_key_params_ = {{0}}; \
    msg_key_params_.ptr = msg_ptr_; \
    msg_key_params_.prop = prop_; \
\
    WM_msg_subscribe_rna_params(mbus, &msg_key_params_, value, __func__); \
  } \
  ((void)0)

  WM_msg_subscribe_rna_anon_prop(
      mbus, GPencilFrameOffsetSettings, use_current_frame, &msg_sub_value_gz_tag_refresh);
  WM_msg_subscribe_rna_anon_prop(
      mbus, GPencilFrameOffsetSettings, frame, &msg_sub_value_gz_tag_refresh);

  WM_msg_subscribe_rna_anon_prop(
      mbus, GreasePencil, onion_keyframe_type, &msg_sub_value_gz_tag_refresh);
  WM_msg_subscribe_rna_anon_prop(mbus, GreasePencil, onion_mode, &msg_sub_value_gz_tag_refresh);
}

static void WIDGETGROUP_gpencil_frame_offset_exit_cleanup(bContext *C,
                                                          wmGizmoGroup *gzgroup,
                                                          wmGizmo *gz,
                                                          bool cancel)
{
  FrameOffsetWidgetGroup *frameoffset_gzgroup = (FrameOffsetWidgetGroup *)gzgroup->customdata;
  frameoffset_gzgroup->gizmo_active = false;
  if (cancel) {
    return;
  }

  float angle = RNA_float_get(gz->ptr, "angle");
  float scale[2];
  float transformed_offset[2], offset[2];
  RNA_float_get_array(gz->ptr, "scale", scale);
  RNA_float_get_array(gz->ptr, "offset", offset);

  mul_v2_v2v2(transformed_offset, scale, frameoffset_gzgroup->start_offset);
  rotate_v2_fl(transformed_offset, angle);
  sub_v2_v2(offset, transformed_offset);

  bGPDframe *gpf = frameoffset_gzgroup->actframe;
  copy_v2_v2(gpf->offset, offset);
  gpf->angle = angle;
  copy_v2_v2(gpf->scale, scale);

  RNA_float_get_array(gz->ptr, "pivot", frameoffset_gzgroup->pivot);
}

void VIEW3D_GGT_gpencil_frame_offset(wmGizmoGroupType *gzgt)
{
  PropertyRNA *prop;

  gzgt->name = "Grease Pencil Frame Offset";
  gzgt->idname = "VIEW3D_GGT_gpencil_frame_offset";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_SCALE);

  gzgt->gzmap_params.spaceid = SPACE_VIEW3D;
  gzgt->gzmap_params.regionid = RGN_TYPE_WINDOW;

  gzgt->poll = WIDGETGROUP_gpencil_frame_offset_poll;
  gzgt->setup = WIDGETGROUP_gpencil_frame_offset_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->invoke_prepare = WIDGETGROUP_gpencil_frame_offset_invoke_prepare;
  gzgt->refresh = WIDGETGROUP_gpencil_frame_offset_refresh;
  gzgt->message_subscribe = WIDGETGROUP_gpencil_frame_offset_message_subscribe;
  gzgt->exit_cleanup = WIDGETGROUP_gpencil_frame_offset_exit_cleanup;
}

/** \} */
