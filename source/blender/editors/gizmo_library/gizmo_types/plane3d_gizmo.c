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
 *
 * The Original Code is Copyright (C) 2014 Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup edgizmolib
 *
 * \name Transform plane Gizmo
 *
 * \brief Transformation plane gizmo on a 3d plane. Mainly used for grease pencil transformations.
 */

#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_main.h"

#include "BLI_listbase.h"
#include "BLI_rect.h"

#include "ED_gizmo_library.h"
#include "ED_gpencil.h"
#include "ED_screen.h"
#include "ED_undo.h"
#include "ED_view3d.h"

#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_primitive.h"
#include "GPU_shader.h"
#include "GPU_state.h"
#include "GPU_vertex_format.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "UI_interface.h"
#include "UI_resources.h"
#include "UI_view2d.h"

#include "WM_api.h"
#include "WM_types.h"

#include "../gizmo_library_intern.h"

/* Structure to hold all the transform-plane values. */
typedef struct tPlane3D {
  /* World space location of the transform-plane. */
  float global_co[3];
  /* World space normal direction of the transform-plane. */
  float global_no[3];
  /* World space transform-plane equation. (ax + by + cz + d = 0). */
  float global[4];

  /* Object space location of the transform-plane. */
  float local_co[3];
  /* Object space normal direction of the transform-plane. */
  float local_no[3];
  /* Object space basis vector in U direction. */
  float local_u[3];
  /* Object space basis vector in V direction. */
  float local_v[3];
  /* Object space transform-plane equation. (ax + by + cz + d = 0). */
  float local[4];

  /* Basis matrices to convert from the 2d plane space to 3d. */
  float basis_mat[4][4], basis_inv[4][4];
} tPlane3D;

static void plane3d_space_to_global_space(tPlane3D *plane,
                                          const float matrix_basis[4][4],
                                          const float p_co[2],
                                          float r_co[3])
{
  copy_v3_v3(r_co, plane->local_co);
  madd_v3_v3fl(r_co, plane->local_u, p_co[0]);
  madd_v3_v3fl(r_co, plane->local_v, p_co[1]);
  mul_m4_v3(matrix_basis, r_co);
}

static bool screen_space_to_plane3d_space(ARegion *region,
                                          tPlane3D *plane,
                                          const float matrix_basis[4][4],
                                          const float mval[2],
                                          float r_co[2])
{
  float global_co[3];
  if (!ED_view3d_win_to_3d_on_plane(region, plane->global, mval, false, global_co)) {
    return false;
  }

  float diff_inv[4][4];
  invert_m4_m4(diff_inv, matrix_basis);

  float local_co[3], tmp[3];
  mul_v3_m4v3(local_co, diff_inv, global_co);
  sub_v3_v3v3(tmp, local_co, plane->local_co);
  r_co[0] = dot_v3v3(tmp, plane->local_u);
  r_co[1] = dot_v3v3(tmp, plane->local_v);
  return true;
}

/* Assumes `plane.local_no` and `plane.local_co` are setup. */
static void plane3d_update_plane_struct(tPlane3D *plane, const float matrix_basis[4][4])
{
  /* Calculate local U and V basis vectors. */
  ortho_basis_v3v3_v3(plane->local_u, plane->local_v, plane->local_no);
  plane_from_point_normal_v3(plane->local, plane->local_co, plane->local_no);

  /* Get all the plane coordinates and vectors in global space. */
  mul_v3_m4v3(plane->global_co, matrix_basis, plane->local_co);
  mul_v3_mat3_m4v3(plane->global_no, matrix_basis, plane->local_no);

  /* Calculate the global plane equation. */
  normalize_v3(plane->global_no);
  plane_from_point_normal_v3(plane->global, plane->global_co, plane->global_no);

  /* Calculate basis matrix. */
  unit_m4(plane->basis_mat);
  copy_v3_v3(plane->basis_mat[0], plane->local_u);
  copy_v3_v3(plane->basis_mat[1], plane->local_v);
  copy_v3_v3(plane->basis_mat[2], plane->local_no);
  invert_m4_m4(plane->basis_inv, plane->basis_mat);
}

/* ----------------------------------------------------------------------- */
/* GIZMO_GT_xform_plane3d */

/* XFormPlane3D.part */
enum {
  PART_NONE = -1,
  PART_BOX = 0,

  PART_SCALE_CONTROL_NW,
  PART_SCALE_CONTROL_NE,
  PART_SCALE_CONTROL_SE,
  PART_SCALE_CONTROL_SW,

  PART_SCALE_CONTROL_N,
  PART_SCALE_CONTROL_E,
  PART_SCALE_CONTROL_S,
  PART_SCALE_CONTROL_W,

  PART_SIDE_N,
  PART_SIDE_E,
  PART_SIDE_S,
  PART_SIDE_W,

  PART_ROTATE_CONTROL_NW,
  PART_ROTATE_CONTROL_NE,
  PART_ROTATE_CONTROL_SE,
  PART_ROTATE_CONTROL_SW,

  PART_CONTROL_CENTER,

  /* Keep last. */
  PART_TOTAL
};

#define IS_SCALE_CONTROL(x) (x >= PART_SCALE_CONTROL_NW && x <= PART_SCALE_CONTROL_W)
#define IS_SCALE_CORNER_CONTROL(x) (x >= PART_SCALE_CONTROL_NW && x <= PART_SCALE_CONTROL_SW)
#define IS_SCALE_SIDE_CONTROL(x) (x >= PART_SCALE_CONTROL_N && x <= PART_SCALE_CONTROL_W)
#define IS_ROTATE_CONTROL(x) (x >= PART_ROTATE_CONTROL_NW && x <= PART_ROTATE_CONTROL_SW)
#define IS_SKEW_CONTROL(x) (x >= PART_SIDE_N && x <= PART_SIDE_W)

#define SQUARE(x) (x * x)

typedef enum eXFormPlane3DActions {
  ACTION_IDLE = 0,
  ACTION_MOVE = (1 << 0),
  ACTION_ROTATE = (1 << 1),
  ACTION_SCALE = (1 << 2),
  ACTION_SKEW = (1 << 3),
  ACTION_MOVE_PIVOT = (1 << 4),
} eXFormPlane3DActions;

typedef enum eXFormPlane3DFlag {
  /* Side proportions stay the same while scaling. */
  IS_PROPORTIONAL_SCALING = (1 << 0),
  /* Scaling is done from the center in all directions. */
  IS_CENTER_SCALING = (1 << 1),
} eXFormPlane3DFlag;

typedef struct tMouseCollisionSettings {
  /* Mouse collision */
  float scale_corner_radius;
  float scale_side_radius;
  float rotate_corner_radius;
  float skew_side_width;
  float rotation_center_radius;
} tMouseCollisionSettings;

typedef struct tTransformation {
  /* Translation of the transform box in transform-plane space. */
  float x, y;
  /* Rotation angle of the transform box. */
  float angle;
  /* Scale along x and y of the transform box. */
  float s_x, s_y;
  /* Skew in x and y direction of the transform box. */
  float sk_x, sk_y;
  /* Center of rotation. */
  float c_x, c_y;
} tTransformation;

typedef struct XFormPlane3D {
  wmGizmo gizmo;
  ARegion *region;

  /* Corner positions of the box. */
  float corner_cp[4][3];
  float pcorner_cp[4][2];

  float sides_cp[4][3];
  float psides_cp[4][2];

  float center_cp[3];
  float pcenter_cp[2];
  rctf box;
  rctf box_scaled;

  int min_size;

  /* Box dimensions*/
  float dims[2];

  /* Status flag. */
  eXFormPlane3DFlag flag;

  /* State. */
  eXFormPlane3DActions current_action;

  /* Enabled actions */
  eXFormPlane3DActions enabled_actions;

  /* gizmo->highlight_part */
  int part;

  tMouseCollisionSettings m_collision;

  /* Local object space. */
  /* use gizmo->matrix_basis */

  /* Mouse coordinates in screen space. */
  float mval[2];
  float mval_prev[2];
  float mval_start[2];

  float pmval[2];
  float pmval_prev[2];
  float pmval_start[2];

  tPlane3D plane;

  struct {
    tTransformation start;
    tTransformation delta;

    float translation_mat[3][3];
    float rotation_mat[3][3];
    float rotation_inv[3][3];
    float scale_mat[3][3];
    float scale_inv[3][3];
    float skew_mat[3][3];
    float skew_inv[3][3];

    float prev_final_mat[3][3];
    /* Combined 2d transformation matrix using homogeneous coordinates. */
    float final_mat[3][3];
    float final_inv[3][3];
  } transform;

} XFormPlane3D;

static void rctf_to_corner_points(const rctf *rect, float corners[4][2])
{
  corners[0][0] = rect->xmin;
  corners[0][1] = rect->ymax;
  corners[1][0] = rect->xmax;
  corners[1][1] = rect->ymax;
  corners[2][0] = rect->xmax;
  corners[2][1] = rect->ymin;
  corners[3][0] = rect->xmin;
  corners[3][1] = rect->ymin;
}

static void rctf_get_center_v2(const rctf *rect, float r_center[2])
{
  r_center[0] = BLI_rctf_cent_x(rect);
  r_center[1] = BLI_rctf_cent_y(rect);
}

static void update_tranlation_matrix(XFormPlane3D *xplane)
{
  float translation[3][3];
  unit_m3(translation);

  translation[2][0] = xplane->transform.start.x + xplane->transform.delta.x;
  translation[2][1] = xplane->transform.start.y + xplane->transform.delta.y;

  copy_m3_m3(xplane->transform.translation_mat, translation);
}

static void update_rotation_matrix(XFormPlane3D *xplane)
{
  float rotation[3][3];
  unit_m3(rotation);

  const float angle = xplane->transform.start.angle + xplane->transform.delta.angle;

  float co = cosf(angle);
  float si = sinf(angle);

  rotation[0][0] = co;
  rotation[0][1] = si;
  rotation[1][0] = -si;
  rotation[1][1] = co;

  copy_m3_m3(xplane->transform.rotation_mat, rotation);
  invert_m3_m3(xplane->transform.rotation_inv, xplane->transform.rotation_mat);
}

static void update_scale_matrix(XFormPlane3D *xplane)
{
  float scale[3][3];
  unit_m3(scale);

  scale[0][0] = xplane->transform.start.s_x + xplane->transform.delta.s_x;
  scale[1][1] = xplane->transform.start.s_y + xplane->transform.delta.s_y;

  copy_m3_m3(xplane->transform.scale_mat, scale);
  invert_m3_m3(xplane->transform.scale_inv, xplane->transform.scale_mat);
}

static void update_skew_matrix(XFormPlane3D *xplane)
{
  float skew[3][3];
  unit_m3(skew);
  skew[1][0] = xplane->transform.start.sk_x + xplane->transform.delta.sk_x;
  skew[0][1] = xplane->transform.start.sk_y + xplane->transform.delta.sk_y;

  copy_m3_m3(xplane->transform.skew_mat, skew);
  invert_m3_m3(xplane->transform.skew_inv, xplane->transform.skew_mat);
}

static void compute_final_transform_matrix(XFormPlane3D *xplane)
{
  /* Combine all the computed matrices. */
  mul_m3_series(xplane->transform.final_mat,
                xplane->transform.translation_mat,
                xplane->transform.rotation_mat,
                xplane->transform.skew_mat,
                xplane->transform.scale_mat);
  invert_m3_m3(xplane->transform.final_inv, xplane->transform.final_mat);
}

static void update_and_compute_transform_matrices(XFormPlane3D *xplane)
{
  update_tranlation_matrix(xplane);
  update_rotation_matrix(xplane);
  update_scale_matrix(xplane);
  update_skew_matrix(xplane);
  compute_final_transform_matrix(xplane);
}

/* Assumes `xplane->plane.local_no` and `xplane->plane.local_co` are setup. */
static void xform_plane3d_update_plane_struct(const wmGizmo *gz, XFormPlane3D *xplane)
{
  plane3d_update_plane_struct(&xplane->plane, gz->matrix_basis);
}

static void xform_plane3d_draw_scale_cp(const float corners[4][3],
                                        const float sides[4][3],
                                        const float color[4],
                                        uint pos)
{
  float viewport[4];
  GPU_viewport_size_get_f(viewport);
  immUniform2fv("viewportSize", &viewport[2]);

  float black[3] = {0, 0, 0};
  // UI_GetThemeColorType3fv(TH_GP_VERTEX_SELECT, SPACE_VIEW3D, color);

  immUniformColor3fv(black);
  GPU_point_size(8.0f + 2.0f);
  immBegin(GPU_PRIM_POINTS, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, corners[i]);
  }
  immEnd();

  immUniformColor3fv(color);
  GPU_point_size(8.0f);
  immBegin(GPU_PRIM_POINTS, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, corners[i]);
  }
  immEnd();

  immUniformColor3fv(black);
  GPU_point_size(8.0f + 2.0f);
  immBegin(GPU_PRIM_POINTS, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, sides[i]);
  }
  immEnd();

  immUniformColor3fv(color);
  GPU_point_size(8.0f);
  immBegin(GPU_PRIM_POINTS, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, sides[i]);
  }
  immEnd();
}

static void xform_plane3d_draw_center_of_rotation_cp(const float center[3],
                                                     const float color[4],
                                                     uint pos)
{
  float viewport[4];
  GPU_viewport_size_get_f(viewport);
  immUniform2fv("viewportSize", &viewport[2]);

  float black[3] = {0, 0, 0};
  // UI_GetThemeColorType3fv(TH_GP_VERTEX_SELECT, SPACE_VIEW3D, color);

  immUniformColor3fv(black);
  GPU_point_size(12.0f + 2.0f);
  immBegin(GPU_PRIM_POINTS, 1);
  immVertex3fv(pos, center);
  immEnd();

  immUniformColor3fv(color);
  GPU_point_size(12.0f);
  immBegin(GPU_PRIM_POINTS, 1);
  immVertex3fv(pos, center);
  immEnd();
}

static void xform_plane3d_draw_rect(const float coords[4][3],
                                    const float color[4],
                                    uint pos,
                                    const float line_width)
{
  float viewport[4];
  GPU_viewport_size_get_f(viewport);
  immUniform2fv("viewportSize", &viewport[2]);

  float black[4] = {0, 0, 0, color[3]};
  // UI_GetThemeColorType3fv(TH_GP_VERTEX_SELECT, SPACE_VIEW3D, color);

  /* Draw black outline. */
  GPU_line_width(line_width + 2.0f);
  immUniformColor4fv(black);
  immBegin(GPU_PRIM_LINE_LOOP, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, coords[i]);
  }
  immEnd();

  /* Draw wire rectangle. */
  GPU_line_width(line_width);
  immUniformColor4fv(color);
  immBegin(GPU_PRIM_LINE_LOOP, 4);
  for (int i = 0; i < 4; i++) {
    immVertex3fv(pos, coords[i]);
  }
  immEnd();
}

static void gizmo_xform_plane3d_draw_ex(const float corners[4][3],
                                        const float sides[4][3],
                                        const float center[3],
                                        const float color[4],
                                        const float line_width,
                                        const int enabled_actions)
{
  GPU_depth_test(GPU_DEPTH_NONE);

  GPU_matrix_push_projection();
  GPU_polygon_offset(1.0f, 1.0f);
  GPU_matrix_push();

  GPU_line_smooth(true);
  GPU_blend(GPU_BLEND_ALPHA);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  if (line_width > 0.0f) {
    xform_plane3d_draw_rect(corners, color, pos, line_width);
  }
  if (enabled_actions & ACTION_SCALE) {
    xform_plane3d_draw_scale_cp(corners, sides, color, pos);
  }
  if (enabled_actions & ACTION_MOVE_PIVOT) {
    xform_plane3d_draw_center_of_rotation_cp(center, color, pos);
  }
  immUnbindProgram();

  GPU_blend(GPU_BLEND_NONE);
  GPU_line_smooth(false);

  GPU_matrix_pop();
  GPU_matrix_pop_projection();

  /* Reset to default. */
  GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
}

static void gizmo_xform_plane3d_draw(const bContext *C, wmGizmo *gz)
{
  XFormPlane3D *xplane = (XFormPlane3D *)gz;
  RegionView3D *rv3d = CTX_wm_region_view3d(C);

  float center[2];
  center[0] = BLI_rctf_cent_x(&xplane->box);
  center[1] = BLI_rctf_cent_y(&xplane->box);
  add_v2_v2v2(center, &xplane->transform.start.x, &xplane->transform.delta.x);

  /* Calculate the pixel size (world space) at the center of the gizmo. */
  float pt[3];
  plane3d_space_to_global_space(&xplane->plane, gz->matrix_basis, center, pt);
  float pixel_size = ED_view3d_pixel_size(rv3d, pt);
  /* Make sure the pixel size is positive and greater than zero. */
  CLAMP_MIN(pixel_size, 0.000001f);

  /* Account for the current scaling. */
  float scale[2], parent_scale;
  parent_scale = mat4_to_scale(gz->matrix_basis);
  add_v2_v2v2(scale, &xplane->transform.start.s_x, &xplane->transform.delta.s_x);
  mul_v2_fl(scale, parent_scale);

  /* Calculate the current width and height of the box (in pixels). */
  float width = fabsf((BLI_rctf_size_x(&xplane->box) / pixel_size) * scale[0]);
  float height = fabsf((BLI_rctf_size_y(&xplane->box) / pixel_size) * scale[1]);

  /* Create a scaled box that has a minimum size. */
  BLI_rctf_init(
      &xplane->box_scaled, xplane->box.xmin, xplane->box.xmax, xplane->box.ymin, xplane->box.ymax);
  CLAMP_MIN(width, xplane->min_size);
  CLAMP_MIN(height, xplane->min_size);
  BLI_rctf_resize_x(&xplane->box_scaled, (width * pixel_size) / scale[0]);
  BLI_rctf_resize_y(&xplane->box_scaled, (height * pixel_size) / scale[1]);

  /* Calculate corners. */
  float corner[4][2];
  rctf_to_corner_points(&xplane->box_scaled, corner);

  for (int i = 0; i < 4; i++) {
    float tmp[2];
    mul_v2_m3v2(tmp, xplane->transform.final_mat, corner[i]);
    /* Project 2d coordinates into 3d. */
    plane3d_space_to_global_space(&xplane->plane, gz->matrix_basis, tmp, xplane->corner_cp[i]);
    copy_v2_v2(xplane->pcorner_cp[i], corner[i]);
  }

  for (int i = 0; i < 4; i++) {
    interp_v3_v3v3(
        xplane->sides_cp[i], xplane->corner_cp[i], xplane->corner_cp[(i + 1) % 4], 0.5f);
    interp_v2_v2v2(xplane->psides_cp[i], corner[i], corner[(i + 1) % 4], 0.5f);
  }

  float center_transformed[2];
  add_v2_v2v2(xplane->pcenter_cp, &xplane->transform.start.c_x, &xplane->transform.delta.c_x);
  mul_v2_m3v2(center_transformed, xplane->transform.final_mat, xplane->pcenter_cp);
  plane3d_space_to_global_space(
      &xplane->plane, gz->matrix_basis, center_transformed, xplane->center_cp);

  gizmo_xform_plane3d_draw_ex(xplane->corner_cp,
                              xplane->sides_cp,
                              xplane->center_cp,
                              gz->color,
                              gz->line_width,
                              xplane->enabled_actions);
}

static void gizmo_xform_plane3d_move_update(XFormPlane3D *xplane)
{
  float tvec[2];
  sub_v2_v2v2(tvec, xplane->pmval, xplane->pmval_start);
  copy_v2_v2(&xplane->transform.delta.x, tvec);
}

static void gizmo_xform_plane3d_rotate_update(XFormPlane3D *xplane)
{
  float tvec[2], center[2];
  sub_v2_v2v2(tvec, xplane->pmval, xplane->pmval_start);
  copy_v2_v2(center, &xplane->transform.start.c_x);

  /* Multiply by the final matrix to get `center` to the correct position on the transform plane so
   * we can compare it to pmval. */
  mul_m3_v2(xplane->transform.prev_final_mat, center);

  float start_vec[2], end_vec[2];
  sub_v2_v2v2(start_vec, xplane->pmval, center);
  sub_v2_v2v2(end_vec, xplane->pmval_start, center);
  normalize_v2(start_vec);
  normalize_v2(end_vec);

  xplane->transform.delta.angle = angle_signed_v2v2(start_vec, end_vec);

  float offset_vec[2], tmp_dir[2], tmp_start[2];

  mul_v2_m3v2(tmp_start, xplane->transform.prev_final_mat, &xplane->transform.start.c_x);
  sub_v2_v2v2(offset_vec, tmp_start, &xplane->transform.start.x);

  negate_v2_v2(tmp_dir, offset_vec);
  rotate_v2_fl(tmp_dir, xplane->transform.delta.angle);

  add_v2_v2v2(&xplane->transform.delta.x, offset_vec, tmp_dir);
}

static void gizmo_xform_plane3d_scale_update(XFormPlane3D *xplane)
{
  const bool is_proportional_scaling = (bool)(xplane->flag & IS_PROPORTIONAL_SCALING);
  const bool is_center_scaling = (bool)(xplane->flag & IS_CENTER_SCALING);

  float mouse_vec[2], mouse_vec_aligned[2], vec_cp[2], center[2];
  float box_size_x = BLI_rctf_size_x(&xplane->box_scaled);
  float box_size_y = BLI_rctf_size_y(&xplane->box_scaled);
  float box_aspect = box_size_y / box_size_x;

  /* Vector to current mouse position from the start position in transform plane space. */
  sub_v2_v2v2(mouse_vec, xplane->pmval, xplane->pmval_start);

  copy_v2_v2(mouse_vec_aligned, mouse_vec);
  mul_m3_v2(xplane->transform.rotation_inv, mouse_vec_aligned);
  mul_m3_v2(xplane->transform.skew_inv, mouse_vec_aligned);

  /* Vector to the control point from the center of the transform box. */
  if (IS_SCALE_CORNER_CONTROL(xplane->part)) {
    copy_v2_v2(vec_cp, xplane->pcorner_cp[xplane->part - PART_SCALE_CONTROL_NW]);
  }
  else if (IS_SCALE_SIDE_CONTROL(xplane->part)) {
    copy_v2_v2(vec_cp, xplane->psides_cp[xplane->part - PART_SCALE_CONTROL_N]);
  }
  rctf_get_center_v2(&xplane->box_scaled, center);
  sub_v2_v2(vec_cp, center);

  const bool scale_x = fabsf(vec_cp[0]) > FLT_EPSILON;
  const bool scale_y = fabsf(vec_cp[1]) > FLT_EPSILON;

  /* Proportional scaling. */
  if (is_proportional_scaling && scale_x && scale_y) {

    float corner_vec[2];
    copy_v2_v2(corner_vec, vec_cp);
    if (!is_center_scaling) {
      mul_v2_fl(corner_vec, 2.0f);
    }
    corner_vec[0] /= box_size_x;
    corner_vec[1] /= box_size_y;
    mul_v2_v2(corner_vec, &xplane->transform.start.s_x);

    float transform_vec[2];
    copy_v2_v2(transform_vec, mouse_vec_aligned);
    transform_vec[0] /= box_size_x;
    transform_vec[1] /= box_size_y;
    add_v2_v2(transform_vec, corner_vec);

    float x = transform_vec[0], y = transform_vec[1];
    float w = fabsf(x / xplane->transform.start.s_x);
    float h = fabsf(y / xplane->transform.start.s_y);
    const float fac = -signf(corner_vec[0] * corner_vec[1]);
    const float ratio = fabsf(xplane->transform.start.s_y / xplane->transform.start.s_x);
    if ((x > 0 && y > 0) || (x < 0 && y < 0)) {
      if (w > h) {
        mouse_vec_aligned[1] = (fac * corner_vec[0]) + (x);
        mouse_vec_aligned[1] *= box_size_y * ratio;
      }
      else {
        mouse_vec_aligned[0] = (fac * corner_vec[1]) + (y);
        mouse_vec_aligned[0] *= box_size_x / ratio;
      }
    }
    else {
      if (w > h) {
        mouse_vec_aligned[1] = (fac * corner_vec[0]) - (x);
        mouse_vec_aligned[1] *= box_size_y * ratio;
      }
      else {
        mouse_vec_aligned[0] = (fac * corner_vec[1]) - (y);
        mouse_vec_aligned[0] *= box_size_x / ratio;
      }
    }
    copy_v2_v2(mouse_vec, mouse_vec_aligned);
    mul_m3_v2(xplane->transform.skew_mat, mouse_vec);
    mul_m3_v2(xplane->transform.rotation_mat, mouse_vec);
  }

  if (!is_center_scaling) {
    mul_v2_fl(mouse_vec, 0.5);
    mul_v2_fl(mouse_vec_aligned, 0.5);

    float offset_vec[2];
    if (scale_x && scale_y) {
      copy_v2_v2(offset_vec, mouse_vec);
    }
    else {
      project_v2_v2v2(offset_vec, mouse_vec_aligned, vec_cp);
      mul_m3_v2(xplane->transform.rotation_mat, offset_vec);
    }
    copy_v2_v2(&xplane->transform.delta.x, offset_vec);
  }
  else {
    zero_v2(&xplane->transform.delta.x);
  }

  /* The quotient between the vector from the center to the new (scaled) corner and the vector to
   * the current corner is the scale factor. E.g. this value is 1.0 when there is no change. In
   * this case we want to keep track of the change in scale factor, so we need to substract
   one.*/
  if (scale_x) {
    xplane->transform.delta.s_x = ((vec_cp[0] + mouse_vec_aligned[0]) / vec_cp[0]) - 1.0f;
  }
  if (scale_y) {
    xplane->transform.delta.s_y = ((vec_cp[1] + mouse_vec_aligned[1]) / vec_cp[1]) - 1.0f;
  }
}

static void gizmo_xform_plane3d_skew_update(XFormPlane3D *xplane)
{
  const bool is_center_scaling = (bool)(xplane->flag & IS_CENTER_SCALING);

  float mouse_vec[2], mouse_vec_aligned[2];
  sub_v2_v2v2(mouse_vec, xplane->pmval, xplane->pmval_start);

  copy_v2_v2(mouse_vec_aligned, mouse_vec);
  mul_m3_v2(xplane->transform.rotation_inv, mouse_vec_aligned);

  float center[2], vec_center[2];
  copy_v2_v2(vec_center, xplane->psides_cp[xplane->part - PART_SIDE_N]);
  rctf_get_center_v2(&xplane->box_scaled, center);
  sub_v2_v2(vec_center, center);

  if (!is_center_scaling) {
    mul_v2_fl(mouse_vec_aligned, 0.5f);
  }

  float offset_vec_x[2] = {mouse_vec_aligned[0], 0.0f};
  float offset_vec_y[2] = {0.0f, mouse_vec_aligned[1]};
  float skew_x = mouse_vec_aligned[0] / (xplane->dims[1] * xplane->transform.start.s_y * 0.5f);
  float skew_y = mouse_vec_aligned[1] / (xplane->dims[0] * xplane->transform.start.s_x * 0.5f);
  if (ELEM(xplane->part, PART_SIDE_N, PART_SIDE_S)) {
    xplane->transform.delta.sk_x = xplane->part == PART_SIDE_N ? skew_x : -skew_x;
    mul_m3_v2(xplane->transform.rotation_mat, offset_vec_x);
    if (!is_center_scaling) {
      copy_v2_v2(&xplane->transform.delta.x, offset_vec_x);
    }
  }
  else if (ELEM(xplane->part, PART_SIDE_E, PART_SIDE_W)) {
    xplane->transform.delta.sk_y = xplane->part == PART_SIDE_E ? skew_y : -skew_y;
    mul_m3_v2(xplane->transform.rotation_mat, offset_vec_y);
    if (!is_center_scaling) {
      copy_v2_v2(&xplane->transform.delta.x, offset_vec_y);
    }
  }
}

static void gizmo_xform_plane3d_pivot_update(XFormPlane3D *xplane)
{
  float mouse_vec[2], mouse_vec_aligned[2];
  sub_v2_v2v2(mouse_vec, xplane->pmval, xplane->pmval_start);

  copy_v2_v2(mouse_vec_aligned, mouse_vec);
  mul_m3_v2(xplane->transform.rotation_inv, mouse_vec_aligned);
  mul_m3_v2(xplane->transform.skew_inv, mouse_vec_aligned);
  mul_m3_v2(xplane->transform.scale_inv, mouse_vec_aligned);

  copy_v2_v2(&xplane->transform.delta.c_x, mouse_vec_aligned);
}

static void gizmo_xform_plane3d_out_matrix_update(bContext *C, wmGizmo *gz, XFormPlane3D *xplane)
{
  /* Output the angle and scale here because some gizmo group need these when the final matrix is
   * updated. */
  RNA_float_set(gz->ptr, "angle", xplane->transform.start.angle + xplane->transform.delta.angle);
  float scale[2];
  add_v2_v2v2(scale, &xplane->transform.start.s_x, &xplane->transform.delta.s_x);
  RNA_float_set_array(gz->ptr, "scale", scale);

  float final_mat[4][4], tmp_offset[4][4];
  unit_m4(tmp_offset);
  /* Convert the 3x3 matrix to a 4x4 one (3rd column + row is the identity). */
  copy_v2_v2(tmp_offset[0], xplane->transform.final_mat[0]);
  copy_v2_v2(tmp_offset[1], xplane->transform.final_mat[1]);
  copy_v2_v2(tmp_offset[3], xplane->transform.final_mat[2]);

  mul_m4_series(final_mat, xplane->plane.basis_mat, tmp_offset, xplane->plane.basis_inv);

  wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(gz, "matrix");
  if (WM_gizmo_target_property_is_valid(gz_prop)) {
    WM_gizmo_target_property_float_set_array(C, gz, gz_prop, (float *)final_mat);
  }
}

static void gizmo_xform_plane3d_output_transforms(wmGizmo *gz, XFormPlane3D *xplane)
{
  /* Output the individual transform components. */
  RNA_float_set_array(gz->ptr, "offset", &xplane->transform.start.x);
  RNA_float_set(gz->ptr, "angle", xplane->transform.start.angle);
  RNA_float_set_array(gz->ptr, "scale", &xplane->transform.start.s_x);
  RNA_float_set_array(gz->ptr, "pivot", &xplane->transform.start.c_x);
}

static void gizmo_xform_plane3d_update(XFormPlane3D *xplane)
{
  bool changed = false;
  switch (xplane->current_action) {
    case ACTION_MOVE: {
      gizmo_xform_plane3d_move_update(xplane);
      update_tranlation_matrix(xplane);
      changed = true;
      break;
    }
    case ACTION_ROTATE: {
      gizmo_xform_plane3d_rotate_update(xplane);
      update_rotation_matrix(xplane);
      update_tranlation_matrix(xplane);
      changed = true;
      break;
    }
    case ACTION_SCALE: {
      gizmo_xform_plane3d_scale_update(xplane);
      update_tranlation_matrix(xplane);
      update_scale_matrix(xplane);
      changed = true;
      break;
    }
    case ACTION_SKEW: {
      gizmo_xform_plane3d_skew_update(xplane);
      update_tranlation_matrix(xplane);
      update_skew_matrix(xplane);
      changed = true;
      break;
    }
    case ACTION_MOVE_PIVOT: {
      gizmo_xform_plane3d_pivot_update(xplane);
      break;
    }
    default:
      break;
  }

  if (changed) {
    compute_final_transform_matrix(xplane);
  }
}

enum {
  INIT_TRANFORM_RESET_DELTA = (1 << 0),
  INIT_TRANFORM_RESET_START = (1 << 1),
};

static void gizmo_xform_plane3d_init_transform_values(XFormPlane3D *xplane, const int flag)
{
  /* Initialize transform values. */
  tTransformation *start = &xplane->transform.start;
  tTransformation *delta = &xplane->transform.delta;
  if (flag & INIT_TRANFORM_RESET_START) {
    /* Set the default start values of the transform. */
    copy_v2_fl(&start->x, 0.0f);
    start->angle = 0.0f;
    copy_v2_fl(&start->s_x, 1.0f);
    copy_v2_fl(&start->sk_x, 0.0f);
    copy_v2_fl(&start->c_x, 0.0f);
  }
  if (flag & INIT_TRANFORM_RESET_DELTA) {
    /* Set the default delta values of the transform. */
    copy_v2_fl(&delta->x, 0.0f);
    delta->angle = 0.0f;
    copy_v2_fl(&delta->s_x, 0.0f);
    copy_v2_fl(&delta->sk_x, 0.0f);
    copy_v2_fl(&delta->c_x, 0.0f);
  }
}

static void gizmo_xform_plane3d_init_mouse_collision_settings(XFormPlane3D *xplane)
{
  xplane->m_collision.scale_corner_radius = 10.0f * UI_DPI_FAC;
  xplane->m_collision.scale_side_radius = 8.0f * UI_DPI_FAC;
  xplane->m_collision.rotate_corner_radius = 40.0f * UI_DPI_FAC;
  xplane->m_collision.skew_side_width = 5.0f * UI_DPI_FAC;
  xplane->m_collision.rotation_center_radius = 10.0f * UI_DPI_FAC;
}

static void gizmo_xform_plane3d_init_enabled_actions(wmGizmo *gz, XFormPlane3D *xplane)
{
  xplane->enabled_actions |= RNA_boolean_get(gz->ptr, "use_translation") ? ACTION_MOVE : 0;
  xplane->enabled_actions |= RNA_boolean_get(gz->ptr, "use_rotation") ? ACTION_ROTATE : 0;
  xplane->enabled_actions |= RNA_boolean_get(gz->ptr, "use_scale") ? ACTION_SCALE : 0;
  xplane->enabled_actions |= RNA_boolean_get(gz->ptr, "use_skew") ? ACTION_SKEW : 0;
  xplane->enabled_actions |= RNA_boolean_get(gz->ptr, "use_pivot") ? ACTION_MOVE_PIVOT : 0;
}

static void gizmo_xform_plane3d_apply_delta_transforms(XFormPlane3D *xplane)
{
  /* Apply current delta values to start transformations. */
  add_v2_v2(&xplane->transform.start.x, &xplane->transform.delta.x);
  xplane->transform.start.angle += xplane->transform.delta.angle;
  xplane->transform.start.angle = angle_wrap_rad(xplane->transform.start.angle);
  add_v2_v2(&xplane->transform.start.s_x, &xplane->transform.delta.s_x);
  add_v2_v2(&xplane->transform.start.sk_x, &xplane->transform.delta.sk_x);
  add_v2_v2(&xplane->transform.start.c_x, &xplane->transform.delta.c_x);

  /* Reset delta transformation values. */
  gizmo_xform_plane3d_init_transform_values(xplane, INIT_TRANFORM_RESET_DELTA);
}

static void gizmo_xform_plane3d_setup(wmGizmo *gz)
{
  XFormPlane3D *xplane = (XFormPlane3D *)gz;

  copy_v2_fl(xplane->mval_start, 0.0f);
  copy_v2_fl(xplane->pmval_start, 0.0f);

  copy_v3_fl(xplane->center_cp, 0.0f);
  copy_v2_fl(xplane->pcenter_cp, 0.0f);

  gizmo_xform_plane3d_init_transform_values(xplane,
                                            INIT_TRANFORM_RESET_START | INIT_TRANFORM_RESET_DELTA);
  gizmo_xform_plane3d_init_mouse_collision_settings(xplane);

  update_and_compute_transform_matrices(xplane);
}

static int gizmo_xform_plane3d_invoke(bContext *C, wmGizmo *gz, const wmEvent *event)
{
  XFormPlane3D *xplane = (XFormPlane3D *)gz;
  xplane->region = CTX_wm_region(C);

  copy_v2fl_v2i(xplane->mval, event->mval);
  /* TODO: What should we do when this call fails? */
  screen_space_to_plane3d_space(
      xplane->region, &xplane->plane, gz->matrix_basis, xplane->mval, xplane->pmval);

  /* Store final matrix state. */
  copy_m3_m3(xplane->transform.prev_final_mat, xplane->transform.final_mat);

  if (IS_SCALE_CONTROL(xplane->part)) {
    /* Enter scale operation. */
    copy_v2_v2(xplane->mval_start, xplane->mval);
    copy_v2_v2(xplane->pmval_start, xplane->pmval);

    xplane->current_action = ACTION_SCALE;
  }
  else if (IS_ROTATE_CONTROL(xplane->part)) {
    /* Enter rotate operation. */
    copy_v2_v2(xplane->pmval_start, xplane->pmval);

    xplane->current_action = ACTION_ROTATE;
  }
  else if (xplane->part == PART_BOX) {
    /* Enter move operation. */
    copy_v2_v2(xplane->mval_start, xplane->mval);
    copy_v2_v2(xplane->pmval_start, xplane->pmval);

    xplane->current_action = ACTION_MOVE;
  }
  else if (IS_SKEW_CONTROL(xplane->part)) {
    /* Enter skew operation. */
    copy_v2_v2(xplane->mval_start, xplane->mval);
    copy_v2_v2(xplane->pmval_start, xplane->pmval);

    xplane->current_action = ACTION_SKEW;
  }
  else if (xplane->part == PART_CONTROL_CENTER) {
    /* Enter move pivot operation. */
    copy_v2_v2(xplane->mval_start, xplane->mval);
    copy_v2_v2(xplane->pmval_start, xplane->pmval);

    /* Display the transform while moving the pivot point. */
    gz->flag |= WM_GIZMO_DRAW_MODAL;

    xplane->current_action = ACTION_MOVE_PIVOT;
  }

  return OPERATOR_RUNNING_MODAL;
}

/* Runs while the user interacts with this gizmo. */
static int gizmo_xform_plane3d_modal(bContext *C,
                                     wmGizmo *gz,
                                     const wmEvent *event,
                                     eWM_GizmoFlagTweak UNUSED(flag))
{
  ARegion *region = CTX_wm_region(C);
  XFormPlane3D *xplane = (XFormPlane3D *)gz;

  SET_FLAG_FROM_TEST(xplane->flag, (event->modifier & KM_SHIFT), IS_PROPORTIONAL_SCALING);

  switch (event->type) {
    case EVT_LEFTSHIFTKEY:
    case EVT_RIGHTSHIFTKEY: {
      if (event->val == KM_PRESS) {
        xplane->flag |= IS_PROPORTIONAL_SCALING;
      }
      if (event->val == KM_RELEASE) {
        xplane->flag &= ~IS_PROPORTIONAL_SCALING;
      }
      gizmo_xform_plane3d_update(xplane);
      gizmo_xform_plane3d_out_matrix_update(C, gz, xplane);
      break;
    }
    case EVT_LEFTALTKEY:
    case EVT_RIGHTALTKEY: {
      if (event->val == KM_PRESS) {
        xplane->flag |= IS_CENTER_SCALING;
      }
      if (event->val == KM_RELEASE) {
        xplane->flag &= ~IS_CENTER_SCALING;
      }
      gizmo_xform_plane3d_update(xplane);
      gizmo_xform_plane3d_out_matrix_update(C, gz, xplane);
      break;
    }
    case MOUSEMOVE: {
      copy_v2fl_v2i(xplane->mval, event->mval);
      /* TODO: What should we do when this call fails? */
      screen_space_to_plane3d_space(
          region, &xplane->plane, gz->matrix_basis, xplane->mval, xplane->pmval);

      gizmo_xform_plane3d_update(xplane);
      gizmo_xform_plane3d_out_matrix_update(C, gz, xplane);

      /* Save the mouse position. */
      copy_v2_v2(xplane->mval_prev, xplane->mval);
      copy_v2_v2(xplane->pmval_prev, xplane->pmval);
      break;
    }
    default:
      break;
  }

  return OPERATOR_RUNNING_MODAL;
}

static void gizmo_xform_plane3d_exit(bContext *C, wmGizmo *gz, const bool cancel)
{
  XFormPlane3D *xplane = (XFormPlane3D *)gz;

  if (cancel) {
    gizmo_xform_plane3d_init_transform_values(xplane, INIT_TRANFORM_RESET_DELTA);
    update_and_compute_transform_matrices(xplane);
    gizmo_xform_plane3d_out_matrix_update(C, gz, xplane);
    return;
  }

  /* Exit operation. */
  copy_v2_fl(xplane->mval_start, 0.0f);
  copy_v2_fl(xplane->pmval_start, 0.0f);

  /* Reset state and gizmo flag. */
  xplane->current_action = ACTION_IDLE;
  xplane->flag = 0;
  gz->flag &= ~WM_GIZMO_DRAW_MODAL;

  /* Apply delta transformations. */
  gizmo_xform_plane3d_apply_delta_transforms(xplane);

  /* After applying the deltas, write the transform values to the rna properties. */
  gizmo_xform_plane3d_output_transforms(gz, xplane);

  ED_undo_push(C, "Transform Plane Gizmo");
}

/* Return the cursor to use for the highlighted part of the gizmo. */
static int gizmo_xform_plane3d_get_cursor(wmGizmo *gz)
{
  int highlight_part = gz->highlight_part;
  switch (highlight_part) {
    case PART_SCALE_CONTROL_NE:
    case PART_SCALE_CONTROL_NW:
    case PART_SCALE_CONTROL_SW:
    case PART_SCALE_CONTROL_SE:
    case PART_SCALE_CONTROL_N:
    case PART_SCALE_CONTROL_W:
    case PART_SCALE_CONTROL_S:
    case PART_SCALE_CONTROL_E:
      return WM_CURSOR_NSEW_SCROLL;
    case PART_CONTROL_CENTER:
    case PART_ROTATE_CONTROL_NE:
    case PART_ROTATE_CONTROL_NW:
    case PART_ROTATE_CONTROL_SW:
    case PART_ROTATE_CONTROL_SE:
      return WM_CURSOR_CROSS;
    case PART_SIDE_E:
    case PART_SIDE_W:
      return WM_CURSOR_NS_ARROW;
    case PART_SIDE_N:
    case PART_SIDE_S:
      return WM_CURSOR_EW_ARROW;
    case PART_BOX:
      return WM_CURSOR_HAND;
    default:
      return WM_CURSOR_KNIFE;
  }
}

static int gizmo_xform_plane3d_test_select(bContext *C, wmGizmo *gz, const int mval[2])
{
  ARegion *region = CTX_wm_region(C);
  XFormPlane3D *xplane = (XFormPlane3D *)gz;
  tMouseCollisionSettings *m_collision = &xplane->m_collision;

  xplane->part = PART_NONE;

  /* First, check the center of rotation. */
  float screen_co[2];
  const float mval_fl[2] = {UNPACK2(mval)};
  if ((xplane->enabled_actions & ACTION_MOVE_PIVOT) &&
      ED_view3d_project_float_global(
          region, xplane->center_cp, screen_co, V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
          V3D_PROJ_RET_OK) {
    if (!ELEM(V2D_IS_CLIPPED, screen_co[0], screen_co[1]) &&
        len_squared_v2v2(mval_fl, screen_co) <= SQUARE(m_collision->rotation_center_radius)) {
      return (xplane->part = PART_CONTROL_CENTER);
    }
  }

  /* Then, check the scale controls. */
  if ((xplane->enabled_actions & ACTION_SCALE)) {
    for (int i = 0; i < 4; i++) {
      if (ED_view3d_project_float_global(region,
                                         xplane->corner_cp[i],
                                         screen_co,
                                         V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
          V3D_PROJ_RET_OK) {
        if (!ELEM(V2D_IS_CLIPPED, screen_co[0], screen_co[1]) &&
            len_squared_v2v2(mval_fl, screen_co) <= SQUARE(m_collision->scale_corner_radius)) {
          return (xplane->part = i + PART_SCALE_CONTROL_NW);
        }
      }

      if (ED_view3d_project_float_global(region,
                                         xplane->sides_cp[i],
                                         screen_co,
                                         V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
          V3D_PROJ_RET_OK) {
        if (!ELEM(V2D_IS_CLIPPED, screen_co[0], screen_co[1]) &&
            len_squared_v2v2(mval_fl, screen_co) <= SQUARE(m_collision->scale_side_radius)) {
          return (xplane->part = i + PART_SCALE_CONTROL_N);
        }
      }
    }
  }

  float pmval[2];
  screen_space_to_plane3d_space(region, &xplane->plane, gz->matrix_basis, mval_fl, pmval);

  /* Calculate the mouse coordinates relative to the collision box. */
  float cmval[2];
  mul_v2_m3v2(cmval, xplane->transform.final_inv, pmval);

  /* Side controls for skew. */
  if ((xplane->enabled_actions & ACTION_SKEW)) {
    for (int i = 0; i < 4; i++) {
      float screen_co1[2], screen_co2[2];
      if (ED_view3d_project_float_global(region,
                                         xplane->corner_cp[i],
                                         screen_co1,
                                         V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
              V3D_PROJ_RET_OK &&
          ED_view3d_project_float_global(region,
                                         xplane->corner_cp[(i + 1) % 4],
                                         screen_co2,
                                         V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
              V3D_PROJ_RET_OK) {
        if (!ELEM(V2D_IS_CLIPPED, screen_co1[0], screen_co1[1]) &&
            !ELEM(V2D_IS_CLIPPED, screen_co2[0], screen_co2[1]) &&
            dist_squared_to_line_segment_v2(mval_fl, screen_co1, screen_co2) <=
                SQUARE(m_collision->skew_side_width)) {
          return (xplane->part = i + PART_SIDE_N);
        }
      }
    }
  }

  /* Move control inside of the box. */
  if ((xplane->enabled_actions & ACTION_MOVE) && BLI_rctf_isect_pt_v(&xplane->box_scaled, cmval)) {
    return (xplane->part = PART_BOX);
  }

  /* Check the rotation controls last. */
  if (xplane->enabled_actions & ACTION_ROTATE) {
    for (int i = 0; i < 4; i++) {
      if (ED_view3d_project_float_global(region,
                                         xplane->corner_cp[i],
                                         screen_co,
                                         V3D_PROJ_RET_CLIP_BB | V3D_PROJ_RET_CLIP_WIN) ==
          V3D_PROJ_RET_OK) {
        if (!ELEM(V2D_IS_CLIPPED, screen_co[0], screen_co[1]) &&
            len_squared_v2v2(mval_fl, screen_co) <= SQUARE(m_collision->rotate_corner_radius)) {
          return (xplane->part = i + PART_ROTATE_CONTROL_NW);
        }
      }
    }
  }

  return xplane->part;
}

/* -------------------------------------------------------------------- */
/** \name Transform plane Gizmo API
 * \{ */

static void gizmo_xform_plane3d_property_update(wmGizmo *gz, wmGizmoProperty *UNUSED(gz_prop))
{
  XFormPlane3D *xplane = (XFormPlane3D *)gz;

  if (!RNA_boolean_get(gz->ptr, "force_update")) {
    return;
  }
  RNA_boolean_set(gz->ptr, "force_update", false);

  gizmo_xform_plane3d_init_transform_values(xplane,
                                            INIT_TRANFORM_RESET_START | INIT_TRANFORM_RESET_DELTA);

  /* Initialize dimensions and start offset. */
  RNA_float_get_array(gz->ptr, "dimensions", xplane->dims);
  RNA_float_get_array(gz->ptr, "offset", &xplane->transform.start.x);
  xplane->transform.start.angle = RNA_float_get(gz->ptr, "angle");
  RNA_float_get_array(gz->ptr, "scale", &xplane->transform.start.s_x);
  RNA_float_get_array(gz->ptr, "pivot", &xplane->transform.start.c_x);

  xplane->min_size = RNA_int_get(gz->ptr, "min_size");

  /* Copy the local translation to the local plane coordinate. */
  // copy_v3_v3(xplane->plane.local_co, gz->matrix_basis[3]);
  /* FIXME: this assumes we are always in object space, so the origin is 0,0. */
  zero_v3(xplane->plane.local_co);

  float normal[3];
  RNA_float_get_array(gz->ptr, "normal", normal);
  copy_v3_v3(xplane->plane.local_no, normal);

  xform_plane3d_update_plane_struct(gz, xplane);
  gizmo_xform_plane3d_init_enabled_actions(gz, xplane);

  /* Initialize box from dimensions. */
  BLI_rctf_init(&xplane->box,
                -(xplane->dims[0] / 2.0f),
                (xplane->dims[0] / 2.0f),
                -(xplane->dims[1] / 2.0f),
                (xplane->dims[1] / 2.0f));

  update_and_compute_transform_matrices(xplane);
}

static void GIZMO_GT_xform_plane3d(wmGizmoType *gzt)
{
  PropertyRNA *prop;

  /* identifiers */
  gzt->idname = "GIZMO_GT_xform_plane3d";

  /* api callbacks */
  gzt->setup = gizmo_xform_plane3d_setup;
  gzt->draw = gizmo_xform_plane3d_draw;
  gzt->invoke = gizmo_xform_plane3d_invoke;
  gzt->modal = gizmo_xform_plane3d_modal;
  gzt->exit = gizmo_xform_plane3d_exit;
  gzt->cursor_get = gizmo_xform_plane3d_get_cursor;
  gzt->test_select = gizmo_xform_plane3d_test_select;
  gzt->property_update = gizmo_xform_plane3d_property_update;

  gzt->struct_size = sizeof(XFormPlane3D);

  static float unit_v2[2] = {1.0f, 1.0f};
  static float default_normal[3] = {1.0f, 0.0f, 0.0f};

  prop = RNA_def_boolean(
      gzt->srna, "use_translation", true, "Use Translation", "Enable translation action");
  prop = RNA_def_boolean(
      gzt->srna, "use_rotation", true, "Use Rotation", "Enable rotation action");
  prop = RNA_def_boolean(gzt->srna, "use_scale", true, "Use Scale", "Enable scale action");
  prop = RNA_def_boolean(gzt->srna, "use_skew", true, "Use Skew", "Enable skew action");
  prop = RNA_def_boolean(gzt->srna, "use_pivot", true, "Use Pivot", "Enable pivot action");

  /**
   * Transformation plane normal in object space.
   * Note: The transformation plane is defined by the offset in gz->matrix_basis and this
   * normal vector. The offset can be changed using `WM_gizmo_set_matrix_location` and
   * `WM_gizmo_set_matrix_rotation_from_yz_axis`
   */
  prop = RNA_def_float_vector(gzt->srna,
                              "normal",
                              3,
                              default_normal,
                              -FLT_MAX,
                              FLT_MAX,
                              "Plane Normal",
                              "",
                              -FLT_MAX,
                              FLT_MAX);

  /* Dimensions of the selection on the plane (projected). This is the initial bounding box. */
  prop = RNA_def_float_vector(
      gzt->srna, "dimensions", 2, unit_v2, 0, FLT_MAX, "Dimensions", "", 0.0f, FLT_MAX);

  /* Current offset of the gizmo in transform plane space. */
  prop = RNA_def_float_vector(
      gzt->srna, "offset", 2, NULL, -FLT_MAX, FLT_MAX, "Offset", "", -FLT_MAX, FLT_MAX);

  prop = RNA_def_float(gzt->srna, "angle", 0, -FLT_MAX, FLT_MAX, "Angle", "", -M_PI, M_PI);

  prop = RNA_def_float_vector(
      gzt->srna, "scale", 2, unit_v2, -FLT_MAX, FLT_MAX, "Scale", "", -FLT_MAX, FLT_MAX);

  prop = RNA_def_float_vector(
      gzt->srna, "pivot", 2, NULL, -FLT_MAX, FLT_MAX, "Pivot", "", -FLT_MAX, FLT_MAX);

  prop = RNA_def_boolean(gzt->srna, "force_update", false, "", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);

  prop = RNA_def_int(gzt->srna,
                     "min_size",
                     60,
                     1,
                     INT32_MAX,
                     "Minimum Size",
                     "Minimum size of the gizmo in the view",
                     1,
                     INT32_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);

  /* Final transformation matrix in object space. */
  WM_gizmotype_target_property_def(gzt, "matrix", PROP_FLOAT, 16);
}

/* ----------------------------------------------------------------------- */
/* GIZMO_GT_translate_plane3d */

typedef struct TranslatePlane3D {
  wmGizmo gizmo;
  ARegion *region;

  float center[3];
  float pcenter[2];
  float radius;

  float mval[2];
  float mval_prev[2];
  float mval_start[2];

  float pmval[2];
  float pmval_prev[2];
  float pmval_start[2];

  tPlane3D plane;

  float start[2];
  float delta[2];

  float translation_mat[3][3];
  float translation_inv[3][3];
} TranslatePlane3D;

static void gizmo_translate_plane3d_init_transform_values(TranslatePlane3D *tplane, const int flag)
{
  if (flag & INIT_TRANFORM_RESET_START) {
    copy_v2_fl(tplane->start, 0.0f);
  }
  if (flag & INIT_TRANFORM_RESET_DELTA) {
    copy_v2_fl(tplane->delta, 0.0f);
  }
}

static void gizmo_translate_plane3d_update_matrix(TranslatePlane3D *tplane)
{
  float translation[3][3];
  unit_m3(translation);

  translation[2][0] = tplane->start[0] + tplane->delta[0];
  translation[2][1] = tplane->start[1] + tplane->delta[1];
  copy_m3_m3(tplane->translation_mat, translation);
  invert_m3_m3(tplane->translation_inv, tplane->translation_mat);
}

static void gizmo_translate_plane3d_out_matrix_update(bContext *C,
                                                      wmGizmo *gz,
                                                      TranslatePlane3D *tplane)
{
  float final_mat[4][4], tmp_offset[4][4];
  unit_m4(tmp_offset);
  /* Convert the 3x3 matrix to a 4x4 one (3rd column + row is the identity). */
  copy_v2_v2(tmp_offset[0], tplane->translation_mat[0]);
  copy_v2_v2(tmp_offset[1], tplane->translation_mat[1]);
  copy_v2_v2(tmp_offset[3], tplane->translation_mat[2]);

  mul_m4_series(final_mat, tplane->plane.basis_mat, tmp_offset, tplane->plane.basis_inv);

  wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(gz, "matrix");
  if (WM_gizmo_target_property_is_valid(gz_prop)) {
    WM_gizmo_target_property_float_set_array(C, gz, gz_prop, (float *)final_mat);
  }
}

static void gizmo_translate_plane3d_geom_draw(wmGizmo *gz,
                                              const float radius,
                                              const float color[4])
{
#define RESOLUTION 16
  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);

  float viewport[4];
  GPU_viewport_size_get_f(viewport);
  immUniform2fv("viewportSize", &viewport[2]);
  immUniform1f("lineWidth", gz->line_width * U.pixelsize);

  immUniformColor4fv(color);
  imm_draw_circle_wire_3d(pos, 0.0f, 0.0f, radius, RESOLUTION);
  immUnbindProgram();
#undef RESOLUTION
}

static void gizmo_translate_plane3d_draw_intern(const bContext *C,
                                                TranslatePlane3D *tplane,
                                                wmGizmo *gz,
                                                const bool select,
                                                const bool highlight)
{
  float color[4];

  gizmo_color_get(gz, highlight, color);

  float tmp_offset[4][4];
  unit_m4(tmp_offset);
  /* Convert the 3x3 matrix to a 4x4 one (3rd column + row is the identity). */
  copy_v2_v2(tmp_offset[0], tplane->translation_mat[0]);
  copy_v2_v2(tmp_offset[1], tplane->translation_mat[1]);
  copy_v2_v2(tmp_offset[3], tplane->translation_mat[2]);

  float normal_n[3], normal_unit[3] = {0.0f, 0.0f, 1.0f};
  normalize_v3_v3(normal_n, tplane->plane.local_no);

  float rot_mat3[3][3], rot_mat4[4][4];
  rotation_between_vecs_to_mat3(rot_mat3, normal_unit, normal_n);
  copy_m4_m3(rot_mat4, rot_mat3);

  float transformation_mat[4][4];
  mul_m4_series(transformation_mat,
                gz->matrix_basis,
                tplane->plane.basis_mat,
                tmp_offset,
                tplane->plane.basis_inv,
                rot_mat4);

  GPU_matrix_push();
  GPU_matrix_mul(transformation_mat);

  GPU_line_smooth(true);
  GPU_blend(GPU_BLEND_ALPHA);

  gizmo_translate_plane3d_geom_draw(gz, tplane->radius, color);

  GPU_blend(GPU_BLEND_NONE);
  GPU_line_smooth(false);
  GPU_matrix_pop();
}

static void gizmo_translate_plane3d_setup(wmGizmo *gz)
{
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;

  copy_v2_fl(tplane->mval_start, 0.0f);
  copy_v2_fl(tplane->pmval_start, 0.0f);

  copy_v3_fl(tplane->center, 0.0f);
  copy_v2_fl(tplane->pcenter, 0.0f);

  gizmo_translate_plane3d_init_transform_values(
      tplane, INIT_TRANFORM_RESET_START | INIT_TRANFORM_RESET_DELTA);
  gizmo_translate_plane3d_update_matrix(tplane);
}

static void gizmo_translate_plane3d_draw(const bContext *C, wmGizmo *gz)
{
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;
  const bool is_highlight = (gz->state & WM_GIZMO_STATE_HIGHLIGHT) != 0;

  gizmo_translate_plane3d_draw_intern(C, tplane, gz, false, is_highlight);
}

static int gizmo_translate_plane3d_invoke(bContext *C, wmGizmo *gz, const wmEvent *event)
{
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;
  tplane->region = CTX_wm_region(C);

  copy_v2fl_v2i(tplane->mval, event->mval);
  /* TODO: What should we do when this call fails? */
  screen_space_to_plane3d_space(
      tplane->region, &tplane->plane, gz->matrix_basis, tplane->mval, tplane->pmval);

  copy_v2_v2(tplane->mval_start, tplane->mval);
  copy_v2_v2(tplane->pmval_start, tplane->pmval);

  return OPERATOR_RUNNING_MODAL;
}

static int gizmo_translate_plane3d_modal(bContext *C,
                                         wmGizmo *gz,
                                         const wmEvent *event,
                                         eWM_GizmoFlagTweak UNUSED(flag))
{
  ARegion *region = CTX_wm_region(C);
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;

  switch (event->type) {
    case MOUSEMOVE: {
      copy_v2fl_v2i(tplane->mval, event->mval);
      if (!screen_space_to_plane3d_space(
              region, &tplane->plane, gz->matrix_basis, tplane->mval, tplane->pmval)) {
        return OPERATOR_RUNNING_MODAL;
      }

      /* Update delta translation */
      float tvec[2];
      sub_v2_v2v2(tvec, tplane->pmval, tplane->pmval_start);
      copy_v2_v2(tplane->delta, tvec);

      /* Update translation matrix. */
      gizmo_translate_plane3d_update_matrix(tplane);

      /* Output the matrix */
      gizmo_translate_plane3d_out_matrix_update(C, gz, tplane);

      /* Save the mouse position. */
      copy_v2_v2(tplane->mval_prev, tplane->mval);
      copy_v2_v2(tplane->pmval_prev, tplane->pmval);
      break;
    }
    default:
      break;
  }

  return OPERATOR_RUNNING_MODAL;
}

static void gizmo_translate_plane3d_exit(bContext *C, wmGizmo *gz, const bool cancel)
{
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;

  if (cancel) {
    gizmo_translate_plane3d_init_transform_values(tplane, INIT_TRANFORM_RESET_DELTA);
    gizmo_translate_plane3d_update_matrix(tplane);
    gizmo_translate_plane3d_out_matrix_update(C, gz, tplane);
    return;
  }

  copy_v2_fl(tplane->mval_start, 0.0f);
  copy_v2_fl(tplane->pmval_start, 0.0f);

  /* Reset gizmo flag. */
  gz->flag &= ~WM_GIZMO_DRAW_MODAL;

  /* Apply delta transformations. */
  add_v2_v2(tplane->start, tplane->delta);

  /* After applying the deltas, write the transform values to the rna properties. */
  RNA_float_set_array(gz->ptr, "offset", tplane->start);

  ED_undo_push(C, "Translation Plane Gizmo");
}

static int gizmo_translate_plane3d_get_cursor(wmGizmo *gz)
{
  return gz->highlight_part == 0 ? WM_CURSOR_NSEW_SCROLL : WM_CURSOR_DEFAULT;
}

static int gizmo_translate_plane3d_test_select(bContext *C, wmGizmo *gz, const int mval[2])
{
  ARegion *region = CTX_wm_region(C);
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;

  const float mval_fl[2] = {UNPACK2(mval)};
  float pmval[2];
  screen_space_to_plane3d_space(region, &tplane->plane, gz->matrix_basis, mval_fl, pmval);

  /* Calculate the mouse coordinates relative to the circle. */
  float cmval[2];
  mul_v2_m3v2(cmval, tplane->translation_inv, pmval);

  if (len_v2(cmval) < tplane->radius) {
    return 0;
  }

  return -1;
}

static void gizmo_translate_plane3d_property_update(wmGizmo *gz, wmGizmoProperty *gz_prop)
{
  TranslatePlane3D *tplane = (TranslatePlane3D *)gz;

  if (!RNA_boolean_get(gz->ptr, "force_update")) {
    return;
  }
  RNA_boolean_set(gz->ptr, "force_update", false);

  gizmo_translate_plane3d_init_transform_values(
      tplane, INIT_TRANFORM_RESET_START | INIT_TRANFORM_RESET_DELTA);

  /* Initialize start offset and radius. */
  RNA_float_get_array(gz->ptr, "offset", tplane->start);
  tplane->radius = RNA_float_get(gz->ptr, "radius");

  /* Copy the local translation to the local plane coordinate. */
  // copy_v3_v3(xplane->plane.local_co, gz->matrix_basis[3]);
  /* FIXME: this assumes we are always in object space, so the origin is 0,0. */
  zero_v3(tplane->plane.local_co);

  float normal[3];
  RNA_float_get_array(gz->ptr, "normal", normal);
  copy_v3_v3(tplane->plane.local_no, normal);

  plane3d_update_plane_struct(&tplane->plane, gz->matrix_basis);

  gizmo_translate_plane3d_update_matrix(tplane);
}

static void GIZMO_GT_translate_plane3d(wmGizmoType *gzt)
{
  PropertyRNA *prop;

  /* identifiers */
  gzt->idname = "GIZMO_GT_translate_plane3d";

  /* api callbacks */
  gzt->setup = gizmo_translate_plane3d_setup;
  gzt->draw = gizmo_translate_plane3d_draw;
  gzt->invoke = gizmo_translate_plane3d_invoke;
  gzt->modal = gizmo_translate_plane3d_modal;
  gzt->exit = gizmo_translate_plane3d_exit;
  gzt->cursor_get = gizmo_translate_plane3d_get_cursor;
  gzt->test_select = gizmo_translate_plane3d_test_select;
  gzt->property_update = gizmo_translate_plane3d_property_update;

  gzt->struct_size = sizeof(TranslatePlane3D);

  static float default_normal[3] = {1.0f, 0.0f, 0.0f};

  /**
   * Transformation plane normal in object space.
   * Note: The transformation plane is defined by the offset in gz->matrix_basis and this
   * normal vector. The offset can be changed using `WM_gizmo_set_matrix_location` and
   * `WM_gizmo_set_matrix_rotation_from_yz_axis`
   */
  prop = RNA_def_float_vector(gzt->srna,
                              "normal",
                              3,
                              default_normal,
                              -FLT_MAX,
                              FLT_MAX,
                              "Plane Normal",
                              "",
                              -FLT_MAX,
                              FLT_MAX);

  prop = RNA_def_float(gzt->srna, "radius", 1.0f, 0.0f, FLT_MAX, "Radius", "", 0.1f, 10.0f);

  /* Current offset of the gizmo in transform plane space. */
  prop = RNA_def_float_vector(
      gzt->srna, "offset", 2, NULL, -FLT_MAX, FLT_MAX, "Offset", "", -FLT_MAX, FLT_MAX);

  prop = RNA_def_boolean(gzt->srna, "force_update", false, "", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);

  /* Final transformation matrix in object space. */
  WM_gizmotype_target_property_def(gzt, "matrix", PROP_FLOAT, 16);
}

void ED_gizmotypes_plane3d(void)
{
  WM_gizmotype_append(GIZMO_GT_xform_plane3d);
  WM_gizmotype_append(GIZMO_GT_translate_plane3d);
}

/** \} */
