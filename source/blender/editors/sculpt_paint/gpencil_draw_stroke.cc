/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef __cplusplus
extern "C" {
#endif
#include "curve_fit_nd.h"
#ifdef __cplusplus
}
#endif

#include "BKE_colortools.h"
#include "BKE_curve.h"
#include "BKE_gpencil_geom.h"
#include "BKE_gpencil_update_cache.h"
#include "BKE_material.h"

#include "BLI_listbase.h"

#include "BLT_translation.h"

#include "BLI_array.hh"
#include "BLI_hash.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BLI_stack.hh"
#include "BLI_vector.hh"

#include <stdio.h>

#include "gpencil_paint_intern.hh"

namespace blender::ed::sculpt_paint::gpencil {

static void project_buffer_point_to_3d(Scene *scene,
                                       ARegion *region,
                                       Object *ob,
                                       const tGPspoint *point2D,
                                       float *depth,
                                       float r_out[3])
{
#define DEPTH_INVALID 1.0f
  ToolSettings *ts = scene->toolsettings;

  if (depth && (*depth == DEPTH_INVALID)) {
    depth = nullptr;
  }

  int mval_i[2];
  round_v2i_v2fl(mval_i, point2D->m_xy);

  if ((depth != nullptr) && (ED_view3d_autodist_simple(region, mval_i, r_out, 0, depth))) {
    /* projecting onto 3D-Geometry
     * - nothing more needs to be done here, since view_autodist_simple() has already done it
     */
    return;
  }
  float mval_prj[2];
  float rvec[3];

  /* Current method just converts each point in screen-coordinates to
   * 3D-coordinates using the 3D-cursor as reference.
   */
  ED_gpencil_drawing_reference_get(scene, ob, ts->gpencil_v3d_align, rvec);
  const float zfac = ED_view3d_calc_zfac(reinterpret_cast<RegionView3D *>(region->regiondata),
                                         rvec);

  if (ED_view3d_project_float_global(region, rvec, mval_prj, V3D_PROJ_TEST_NOP) ==
      V3D_PROJ_RET_OK) {
    float dvec[3];
    float xy_delta[2];
    sub_v2_v2v2(xy_delta, mval_prj, point2D->m_xy);
    ED_view3d_win_to_delta(region, xy_delta, zfac, dvec);
    sub_v3_v3v3(r_out, rvec, dvec);
  }
  else {
    zero_v3(r_out);
  }
#undef DEPTH_INVALID
}

/**
 * Forward differentiation for 2d cubic bezier segment.
 * Note: Copied from `BKE_forward_diff_bezier` and adapted to use float2.
 */
static void forward_diff_bezier_2d(
    float2 q0, float2 q1, float2 q2, float2 q3, int it, MutableSpan<tGPspoint> r_p)
{
  float f = static_cast<float>(it);
  float2 rt0 = q0;
  float2 rt1 = 3.0f * (q1 - q0) / f;
  f = f * f;
  float2 rt2 = 3.0f * (q0 - 2.0f * q1 + q2) / f;
  f = f * static_cast<float>(it);
  float2 rt3 = (q3 - q0 + 3.0f * (q1 - q2)) / f;

  q0 = rt0;
  q1 = rt1 + rt2 + rt3;
  q2 = 2 * rt2 + 6 * rt3;
  q3 = 6 * rt3;

  for (const int i : r_p.index_range()) {
    copy_v2_v2(r_p[i].m_xy, q0);
    q0 += q1;
    q1 += q2;
    q2 += q3;
  }
}

/**
 * Iterative implementation of the Ramer–Douglas–Peucker algorithm
 * (https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm)
 * Note: Assumes \param r_marked_keep has the same size as \param points.
 */
template<typename Function>
static void simplify_point_buffer_ex(Span<tGPspoint> points,
                                     int start_idx,
                                     int end_idx,
                                     float epsilon,
                                     const Function &get_max_dist_and_index,
                                     MutableSpan<bool> r_marked_keep)
{
  Stack<std::pair<int, int>> stack;
  stack.push({start_idx, end_idx});

  while (!stack.is_empty()) {
    auto [start, end] = stack.pop();

    float max_dist = 0.0f;
    int max_index = -1;
    get_max_dist_and_index(points, r_marked_keep, start, end, start_idx, &max_dist, &max_index);

    if (max_dist > epsilon) {
      stack.push({start, max_index});
      stack.push({max_index, end});
    }
    else {
      for (int i = start + 1; i < end; i++) {
        r_marked_keep[i - start_idx] = false;
      }
    }
  }
}

static void simplify_point_buffer_position(Span<tGPspoint> points,
                                           int start_idx,
                                           int end_idx,
                                           float epsilon,
                                           MutableSpan<bool> r_marked_keep)
{
  auto get_max_dist = [](Span<tGPspoint> points,
                         Span<bool> marked_keep,
                         int start,
                         int end,
                         int start_idx,
                         float *r_max_dist,
                         int *r_max_index) {
    for (int i = start + 1; i < end; i++) {
      if (marked_keep[i - start_idx]) {
        float2 point_on_line;
        closest_to_line_segment_v2(
            point_on_line, points[i].m_xy, points[start].m_xy, points[end].m_xy);

        float dist = len_v2v2(point_on_line, points[i].m_xy);
        if (dist > *r_max_dist) {
          *r_max_dist = dist;
          *r_max_index = i;
        }
      }
    }
  };
  simplify_point_buffer_ex(points, start_idx, end_idx, epsilon, get_max_dist, r_marked_keep);
}

static void simplify_point_buffer_thickness(Span<tGPspoint> points,
                                            int start_idx,
                                            int end_idx,
                                            float epsilon,
                                            MutableSpan<bool> r_marked_keep)
{
  auto get_max_dist = [](Span<tGPspoint> points,
                         Span<bool> marked_keep,
                         int start,
                         int end,
                         int start_idx,
                         float *r_max_dist,
                         int *r_max_index) {
    float avg = (points[start].pressure + points[end].pressure) / 2.0f;
    for (int i = start + 1; i < end; i++) {
      if (marked_keep[i - start_idx]) {
        float dist = fabsf(points[i].pressure - avg);
        if (dist > *r_max_dist) {
          *r_max_dist = dist;
          *r_max_index = i;
        }
      }
    }
  };
  simplify_point_buffer_ex(points, start_idx, end_idx, epsilon, get_max_dist, r_marked_keep);
}

static void simplify_point_buffer_strength(Span<tGPspoint> points,
                                           int start_idx,
                                           int end_idx,
                                           float epsilon,
                                           MutableSpan<bool> r_marked_keep)
{
  auto get_max_dist = [](Span<tGPspoint> points,
                         Span<bool> marked_keep,
                         int start,
                         int end,
                         int start_idx,
                         float *r_max_dist,
                         int *r_max_index) {
    float avg = (points[start].strength + points[end].strength) / 2.0f;
    for (int i = start + 1; i < end; i++) {
      if (marked_keep[i - start_idx]) {
        float dist = fabsf(points[i].strength - avg);
        if (dist > *r_max_dist) {
          *r_max_dist = dist;
          *r_max_index = i;
        }
      }
    }
  };
  simplify_point_buffer_ex(points, start_idx, end_idx, epsilon, get_max_dist, r_marked_keep);
}

/* Helper structure to represent a cubic besier handle in 2d space. */
struct BezTriple2D {
  float2 handle_l;
  float2 ctrl;
  float2 handle_r;
};

/**
 * Fit a curve to the given point buffer using the curvefitnd library and refit method.
 */
static Vector<tGPspoint> fit_curve_2d_to_point_buffer(Span<tGPspoint> points,
                                                      float error_threshold,
                                                      int resolution,
                                                      const bool is_cyclic)
{
  float *points_array = reinterpret_cast<float *>(
      MEM_callocN(sizeof(float) * points.size() * 2, __func__));
  MutableSpan<float2> normalized_points(reinterpret_cast<float2 *>(points_array), points.size());

  /* Calcualte the 2D bounding box. */
  float2 bound_min, bound_max;
  INIT_MINMAX2(bound_min, bound_max);
  for (int i = 0; i < points.size(); i++) {
    minmax_v2v2_v2(bound_min, bound_max, points[i].m_xy);
  }

  /* Normalize the point buffer. */
  float diag_length = len_v2v2(bound_min, bound_max);
  for (int i = 0; i < points.size(); i++) {
    normalized_points[i] = (float2(points[i].m_xy) - bound_min) / diag_length;
  }

  uint calc_flag = CURVE_FIT_CALC_HIGH_QUALIY;
  if (points.size() > 2 && is_cyclic) {
    calc_flag |= CURVE_FIT_CALC_CYCLIC;
  }

  float *cubic_array = nullptr;
  unsigned int cubic_array_len = 0;
  unsigned int *cubic_orig_index = nullptr;
  unsigned int *r_corners_index_array = nullptr;
  unsigned int r_corners_index_len = 0;
  int result = curve_fit_cubic_to_points_refit_fl(
      reinterpret_cast<float *>(normalized_points.data()),
      normalized_points.size(),
      2,
      error_threshold,
      calc_flag,
      nullptr,
      0,
      M_PI,
      &cubic_array,
      &cubic_array_len,
      &cubic_orig_index,
      &r_corners_index_array,
      &r_corners_index_len);

  if (result != 0 || cubic_array_len < 1) {
    return {};
  }

  Span<BezTriple2D> curve_points(reinterpret_cast<BezTriple2D *>(cubic_array), cubic_array_len);
  const int segments = cubic_array_len - (is_cyclic ? 0 : 1);
  const int sample_points_len = (segments * resolution + (is_cyclic ? 0 : 1));

  Vector<tGPspoint> sample_points(sample_points_len, {{0.0f}});
  auto sample_points_on_segment = [&](const tGPspoint orig_pt_curr,
                                      const tGPspoint orig_pt_next,
                                      const BezTriple2D curr_bezt,
                                      const BezTriple2D next_bezt,
                                      MutableSpan<tGPspoint> r_segment_points) {
    /* Map points back to original space (from 0..1). */
    float2 curr_a = (curr_bezt.ctrl * diag_length) + bound_min;
    float2 curr_b = (curr_bezt.handle_r * diag_length) + bound_min;
    float2 next_a = (next_bezt.handle_l * diag_length) + bound_min;
    float2 next_b = (next_bezt.ctrl * diag_length) + bound_min;

    /* Calculate positions. */
    forward_diff_bezier_2d(curr_a, curr_b, next_a, next_b, resolution, r_segment_points);

    for (const int a : r_segment_points.index_range()) {
      float fac = static_cast<float>(a) / static_cast<float>(resolution);
      fac = 3.0f * fac * fac - 2.0f * fac * fac * fac; /* Smooth. */
      r_segment_points[a].pressure = interpf(orig_pt_next.pressure, orig_pt_curr.pressure, fac);
      r_segment_points[a].strength = interpf(orig_pt_next.strength, orig_pt_curr.strength, fac);
    }

    /* TODO: vertex color, time, uv_rot. */
  };

  for (const int i : curve_points.index_range().drop_back(1)) {
    const tGPspoint orig_pt_curr = points[cubic_orig_index[i]];
    const tGPspoint orig_pt_next = points[cubic_orig_index[i + 1]];
    const BezTriple2D curr_bezt = curve_points[i];
    const BezTriple2D next_bezt = curve_points[i + 1];
    MutableSpan<tGPspoint> segment_points = sample_points.as_mutable_span().slice(i * resolution,
                                                                                  resolution);
    sample_points_on_segment(orig_pt_curr, orig_pt_next, curr_bezt, next_bezt, segment_points);
  }

  if (is_cyclic) {
    const tGPspoint orig_pt_curr = points[cubic_orig_index[curve_points.size() - 1]];
    const tGPspoint orig_pt_next = points[cubic_orig_index[0]];
    const BezTriple2D curr_bezt = curve_points.last();
    const BezTriple2D next_bezt = curve_points.first();
    MutableSpan<tGPspoint> segment_points = sample_points.as_mutable_span().slice(
        curve_points.size() - 1 * resolution, resolution);
    sample_points_on_segment(orig_pt_curr, orig_pt_next, curr_bezt, next_bezt, segment_points);
  }
  else {
    sample_points.last() = points.last();
  }

  /* Free all the temporary arrays. */
  MEM_freeN(points_array);
  if (cubic_array) {
    free(cubic_array);
  }
  if (cubic_orig_index) {
    free(cubic_orig_index);
  }
  if (r_corners_index_array) {
    free(r_corners_index_array);
  }

  return sample_points;
}

/**
 * Mutate the points of the polyline to match the shape of the given polycurve.
 */
static void interp_polyline_polycurve(MutableSpan<tGPspoint> r_polyline,
                                      Span<tGPspoint> polycurve,
                                      const float interp_factor,
                                      const bool UNUSED(is_cyclic))
{
  if (polycurve.size() < 1 || r_polyline.size() < 1 || interp_factor == 0.0f) {
    return;
  }

  double total_length_polyline = 0.0f;
  Vector<double> segment_lengths_polyline;
  for (const int i : r_polyline.index_range().drop_back(1)) {
    double segment_length = len_v2v2(r_polyline[i].m_xy, r_polyline[i + 1].m_xy);
    segment_lengths_polyline.append(segment_length);
    total_length_polyline += segment_length;
  }
  /* TODO: handle cyclic curves. */
  segment_lengths_polyline.append(0.0f);

  double total_length_polycurve = 0.0f;
  Vector<double> segment_lengths_polycurve;
  for (const int i : polycurve.index_range().drop_back(1)) {
    double segment_length = len_v2v2(polycurve[i].m_xy, polycurve[i + 1].m_xy);
    segment_lengths_polycurve.append(segment_length);
    total_length_polycurve += segment_length;
  }
  /* TODO: handle cyclic curves. */
  segment_lengths_polycurve.append(0.0f);

  /* Handle edgecase for point case. */
  if (total_length_polycurve <= 1e-8 || total_length_polyline <= 1e-8) {
    for (const int i : r_polyline.index_range()) {
      interp_v2_v2v2(
          r_polyline[i].m_xy, r_polyline[i].m_xy, polycurve.first().m_xy, interp_factor);
    }
    return;
  }

  Vector<double> polyline_normalized_accumulated_dists;
  double dist = 0.0f;
  for (const int i : IndexRange(r_polyline.size())) {
    polyline_normalized_accumulated_dists.append(dist / total_length_polyline);
    dist += segment_lengths_polyline[i];
  }

  Vector<double> polycurve_normalized_accumulated_dists;
  dist = 0.0f;
  for (const int i : IndexRange(polycurve.size())) {
    polycurve_normalized_accumulated_dists.append(dist / total_length_polycurve);
    dist += segment_lengths_polycurve[i];
  }

  /* Handle first point, then loop through the segments. */
  interp_v2_v2v2(
      r_polyline.first().m_xy, r_polyline.first().m_xy, polycurve.first().m_xy, interp_factor);
  for (const int i : r_polyline.index_range().drop_front(1)) {
    double dist = polyline_normalized_accumulated_dists[i];
    auto it = std::lower_bound(polycurve_normalized_accumulated_dists.begin(),
                               polycurve_normalized_accumulated_dists.end(),
                               dist);
    int index = std::distance(polycurve_normalized_accumulated_dists.begin(), it);
    BLI_assert(index < polycurve_normalized_accumulated_dists.size());

    tGPspoint before = polycurve[index - 1];
    tGPspoint after = polycurve[index];
    float factor = 0.0f;
    if (std::abs(dist - polycurve_normalized_accumulated_dists[index]) <= 1e-8) {
      factor = 1.0f;
    }
    else {
      double range = polycurve_normalized_accumulated_dists[index] -
                     polycurve_normalized_accumulated_dists[index - 1];
      factor = (range <= 1e-8) ?
                   1.0f :
                   (dist - polycurve_normalized_accumulated_dists[index - 1]) / range;
    }
    float2 target;
    interp_v2_v2v2(target, before.m_xy, after.m_xy, factor);
    interp_v2_v2v2(r_polyline[i].m_xy, r_polyline[i].m_xy, target, interp_factor);
  }
}

/* Algorithm taken from BKE_gpencil_stroke_smooth_point. Uses a discrete version of a gaussian blur
 * for smoothing. */
static void smooth_buffer_point_position_2d(Span<tGPspoint> points,
                                            int point_index,
                                            float influence,
                                            int iterations,
                                            const bool smooth_caps,
                                            const bool keep_shape,
                                            const bool is_cyclic,
                                            MutableSpan<tGPspoint> r_points)
{
  const int64_t size = points.size();
  /* If nothing to do, return early */
  if (size <= 2 || iterations <= 0) {
    return;
  }

  float2 pos = &points[point_index].m_xy;
  /* If smooth_caps is false, the caps will not be translated by smoothing. */
  if (!smooth_caps && !is_cyclic && ELEM(point_index, 0, size - 1)) {
    copy_v2_v2(r_points[point_index].m_xy, pos);
    return;
  }

  float sco[2] = {0.0f, 0.0f};
  float tmp[2];
  const int n_half = keep_shape ? (iterations * iterations) / 8 + iterations :
                                  (iterations * iterations) / 4 + 2 * iterations + 12;
  double w = keep_shape ? 2.0 : 1.0;
  double w2 = keep_shape ?
                  (1.0 / M_SQRT3) * exp((2 * iterations * iterations) / (double)(n_half * 3)) :
                  0.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = (float)(w - w2);
    float w_after = (float)(w - w2);

    if (is_cyclic) {
      before = (before % size + size) % size;
      after = after % size;
    }
    else {
      if (before < 0) {
        if (!smooth_caps) {
          w_before *= -before / (float)point_index;
        }
        before = 0;
      }
      if (after > size - 1) {
        if (!smooth_caps) {
          w_after *= (after - (size - 1)) / (float)(size - 1 - point_index);
        }
        after = size - 1;
      }
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    sub_v2_v2v2(tmp, points[before].m_xy, pos);
    madd_v2_v2fl(sco, tmp, w_before);
    sub_v2_v2v2(tmp, points[after].m_xy, pos);
    madd_v2_v2fl(sco, tmp, w_after);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / (double)(n_half + 1 - step);
    w2 *= (n_half * 3 + step) / (double)(n_half * 3 + 1 - step);
  }
  total_w += w - w2;

  mul_v2_fl(sco, (float)(1.0 / total_w));
  /* Shift back to global coordinates. */
  add_v2_v2(sco, pos);

  /* Based on influence factor, blend between original and optimal smoothed coordinate. */
  interp_v2_v2v2(r_points[point_index].m_xy, pos, sco, influence);
  return;
}

static void smooth_buffer_point_thickness_2d(Span<tGPspoint> points,
                                             int point_index,
                                             float influence,
                                             int iterations,
                                             const bool is_cyclic,
                                             MutableSpan<tGPspoint> r_points)
{
  /* If nothing to do, return early */
  const int64_t size = points.size();
  if (size <= 2 || iterations <= 0) {
    return;
  }

  /* See BKE_gpencil_stroke_smooth_point for details on the algorithm. */

  float pt_pressure = points[point_index].pressure;
  float pressure = 0.0f;
  const int n_half = (iterations * iterations) / 4 + iterations;
  double w = 1.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = (float)w;
    float w_after = (float)w;

    if (is_cyclic) {
      before = (before % size + size) % size;
      after = after % size;
    }
    else {
      CLAMP_MIN(before, 0);
      CLAMP_MAX(after, size - 1);
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    pressure += w_before * (points[before].pressure - pt_pressure);
    pressure += w_after * (points[after].pressure - pt_pressure);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / (double)(n_half + 1 - step);
  }
  total_w += w;
  /* The accumulated weight total_w should be
   * ~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100
   * here, but sometimes not quite. */
  pressure /= total_w;

  /* Based on influence factor, blend between original and optimal smoothed value. */
  r_points[point_index].pressure = pt_pressure + pressure * influence;
  return;
}

static void smooth_buffer_point_strength_2d(Span<tGPspoint> points,
                                            int point_index,
                                            float influence,
                                            int iterations,
                                            const bool is_cyclic,
                                            MutableSpan<tGPspoint> r_points)
{
  /* If nothing to do, return early */
  const int64_t size = points.size();
  if (size <= 2 || iterations <= 0) {
    return;
  }

  /* See BKE_gpencil_stroke_smooth_point for details on the algorithm. */

  float pt_strength = points[point_index].strength;
  float strength = 0.0f;
  const int n_half = (iterations * iterations) / 4 + iterations;
  double w = 1.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = (float)w;
    float w_after = (float)w;

    if (is_cyclic) {
      before = (before % size + size) % size;
      after = after % size;
    }
    else {
      CLAMP_MIN(before, 0);
      CLAMP_MAX(after, size - 1);
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    strength += w_before * (points[before].strength - pt_strength);
    strength += w_after * (points[after].strength - pt_strength);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / (double)(n_half + 1 - step);
  }
  total_w += w;
  /* The accumulated weight total_w should be
   * ~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100
   * here, but sometimes not quite. */
  strength /= total_w;

  /* Based on influence factor, blend between original and optimal smoothed value. */
  r_points[point_index].strength = pt_strength + strength * influence;
  return;
}

struct RandomSettings {
  float pen_press;

  float3 hsv;
  float pressure;
  float strength;
  float uv;
};

class GPencilDrawOperation : public GPencilPaintOperation {
 public:
  GPencilDrawOperation(const bContext *C, float2 initial_mouse_location)
      : GPencilPaintOperation(C), initial_mouse_location(initial_mouse_location)
  {
    /* Hack: This is currently needed because somehow the grease pencil brushes use settings on the
     * brush level (not the gpencil_settings) that influence the way the painting code handles the
     * input. We need to figure out what we can do to prevent this. */
    brush_->flag = 0;

    /* Initialize sbuffer. */
    cleanup_sbuffer();
    gpd_->runtime.sbuffer = ED_gpencil_sbuffer_ensure(
        nullptr, &gpd_->runtime.sbuffer_size, &gpd_->runtime.sbuffer_used, true);
    gpd_->runtime.matid = BKE_object_material_slot_find_index(obact_, material_);
    gpd_->runtime.sbuffer_brush = brush_;

    /* Copy to eval data so the sbuffer_stroke is drawn correctly. */
    gpd_eval_->runtime.matid = gpd_->runtime.matid;
    gpd_eval_->runtime.sbuffer_brush = gpd_->runtime.sbuffer_brush;

    gpd_->runtime.sbuffer_sflag |= GP_STROKE_3DSPACE;

    float defaultpixsize = 1000.0f / gpd_->pixfactor;
    if (gpl_active_ != nullptr) {
      stroke_diameter = static_cast<float>(brush_->size +
                                           static_cast<int>(gpl_active_->line_change)) /
                        defaultpixsize;
    }
    else {
      stroke_diameter = static_cast<float>(brush_->size) / defaultpixsize;
    }

    init_time = PIL_check_seconds_timer();

    is_vertex_fill = (GPENCIL_USE_VERTEX_COLOR_FILL(ts_, brush_) &&
                      (brush_->gpencil_settings->brush_draw_mode != GP_BRUSH_MODE_MATERIAL)) ||
                     (!GPENCIL_USE_VERTEX_COLOR_FILL(ts_, brush_) &&
                      (brush_->gpencil_settings->brush_draw_mode == GP_BRUSH_MODE_VERTEXCOLOR));

    is_vertex_stroke = (GPENCIL_USE_VERTEX_COLOR_STROKE(ts_, brush_) &&
                        (brush_->gpencil_settings->brush_draw_mode != GP_BRUSH_MODE_MATERIAL)) ||
                       (!GPENCIL_USE_VERTEX_COLOR_STROKE(ts_, brush_) &&
                        (brush_->gpencil_settings->brush_draw_mode == GP_BRUSH_MODE_VERTEXCOLOR));

    copy_v3_v3(vertex_color, brush_->rgb);
    vertex_color[3] = brush_settings_->vertex_factor;
    srgb_to_linearrgb_v4(vertex_color, vertex_color);

    /* Copy fill vertex color. */
    copy_v4_v4(gpd_->runtime.vert_color_fill,
               is_vertex_fill ? vertex_color : float4(material_->gp_style->fill_rgba));
    copy_v4_v4(gpd_eval_->runtime.vert_color_fill, gpd_->runtime.vert_color_fill);

    /* Initialize curve mappings. */
    BKE_curvemapping_init(brush_settings_->curve_sensitivity);
    BKE_curvemapping_init(brush_settings_->curve_strength);
    BKE_curvemapping_init(brush_settings_->curve_jitter);
    BKE_curvemapping_init(brush_settings_->curve_rand_pressure);
    BKE_curvemapping_init(brush_settings_->curve_rand_strength);
    BKE_curvemapping_init(brush_settings_->curve_rand_uv);
    BKE_curvemapping_init(brush_settings_->curve_rand_hue);
    BKE_curvemapping_init(brush_settings_->curve_rand_saturation);
    BKE_curvemapping_init(brush_settings_->curve_rand_value);

    if ((ts_->gpencil_v3d_align & (GP_PROJECT_DEPTH_VIEW | GP_PROJECT_DEPTH_STROKE)) != 0) {
      is_stroke_on_plane = false;
      ED_view3d_depth_override(
          depsgraph_, region_, view3d_, obact_, V3D_DEPTH_NO_GPENCIL, &view_depths);
    }

    /* Reserve some space for the points. */
    point_buffer.reserve(1024);

    int2 mval;
    round_v2i_v2fl(mval, initial_mouse_location);
    init_random_settings(mval);
  };

  ~GPencilDrawOperation()
  {
    cleanup_sbuffer();
    if (view_depths != nullptr) {
      ED_view3d_depths_free(view_depths);
    }
    ED_gpencil_sbuffer_update_eval(gpd_, ob_eval_);
  }

  void on_paint_stroke_update(float2 location,
                              float pressure,
                              float UNUSED(size),
                              bool UNUSED(use_eraser)) override
  {
    /* Create a new point. */
    tGPspoint pt = {0.0f};
    pt.strength = brush_settings_->draw_strength;
    pt.pressure = 1.0f;
    pt.uv_rot = 0.0f;
    copy_v2_v2(pt.m_xy, location);

    /* Radius. */
    if (brush_settings_->flag & GP_BRUSH_USE_PRESSURE) {
      pt.pressure *= BKE_curvemapping_evaluateF(brush_settings_->curve_sensitivity, 0, pressure);
    }

    /* Angle. FIXME: this produces bad results with short segments (because the points are not
     * smoothed). Maybe use a linear regression on the last few points as a vector instead? */
    if (brush_settings_->draw_angle_factor > 0.0f && point_buffer.size() > 1) {
      tGPspoint pt_prev = point_buffer.last();
      const float factor = brush_settings_->draw_angle_factor;
      const float angle = brush_settings_->draw_angle;
      const float v0[2] = {cos(angle), sin(angle)};
      float2 vec;
      sub_v2_v2v2(vec, pt.m_xy, pt_prev.m_xy);
      normalize_v2(vec);
      pt.pressure -= factor * (1.0f - fabs(dot_v2v2(v0, vec)));
      pt.pressure = std::clamp(pt.pressure, 0.0001f, 1.0f);
    }

    /* Color strength. */
    if (brush_settings_->flag & GP_BRUSH_USE_STRENGTH_PRESSURE) {
      pt.strength *= BKE_curvemapping_evaluateF(brush_settings_->curve_strength, 0, pressure);
      pt.strength = std::clamp(
          pt.strength, std::min(GPENCIL_STRENGTH_MIN, brush_settings_->draw_strength), 1.0f);
    }

    pt.time = static_cast<float>(PIL_check_seconds_timer() - init_time);
    copy_v4_v4(pt.vert_color,
               is_vertex_stroke ? vertex_color : float4(material_->gp_style->stroke_rgba));

    random_settings_.pen_press = pressure;

    apply_point_randomness(pt);

    /* Append the point to the buffer. */
    append_and_process_point_buffer(pt);
    update_sbuffer_from_point_buffer(processed_point_buffer);
    ED_gpencil_sbuffer_update_eval(gpd_, ob_eval_);
  }

  void on_paint_stroke_done() override
  {
    if (gpl_active_ == nullptr) {
      gpl_active_ = BKE_gpencil_layer_addnew(gpd_, DATA_("GP_Layer"), true, false);
      BKE_gpencil_tag_full_update(gpd_, nullptr, nullptr, nullptr);
    }

    if (processed_point_buffer.size() == 0) {
      return;
    }

    post_process_point_buffer();

    /* Find the GPFrame that the stroke should be inserted to. */
    eGP_GetFrame_Mode add_frame_mode;
    if (IS_AUTOKEY_ON(scene_)) {
      add_frame_mode = ((ts_->gpencil_flags & GP_TOOL_FLAG_RETAIN_LAST) != 0) ?
                           GP_GETFRAME_ADD_COPY :
                           GP_GETFRAME_ADD_NEW;
    }
    else {
      add_frame_mode = GP_GETFRAME_USE_PREV;
    }

    bGPDframe *actframe = gpl_active_->actframe;
    bGPDframe *gpf = BKE_gpencil_layer_frame_get(gpl_active_, scene_->r.cfra, add_frame_mode);
    /* Evaluate if a new frame was created by comparing to previous active frame. */
    const bool new_frame = actframe != gpf;

    /* No keyframe to insert the stroke into, return early. */
    if (gpf == nullptr) {
      return;
    }

    bGPDstroke *new_stroke = create_stroke_from_sbuffer(gpf);
    if (new_stroke == nullptr) {
      return;
    }

    /* Insert the stroke into the frame. */
    if ((ts_->gpencil_flags & GP_TOOL_FLAG_PAINT_ONBACK) != 0) {
      BLI_addhead(&gpf->strokes, new_stroke);
    }
    else {
      BLI_addtail(&gpf->strokes, new_stroke);
    }

    /* Tag for depsgraph updates. */
    if (!new_frame) {
      /* No frame created: tag active frame for update. */
      BKE_gpencil_tag_full_update(gpd_, gpl_active_, gpf, nullptr);
    }
    else {
      /* New frame created: tag the layer for update. */
      BKE_gpencil_tag_full_update(gpd_, gpl_active_, nullptr, nullptr);
    }
    DEG_id_tag_update(&gpd_->id, ID_RECALC_GEOMETRY);
    gpd_->flag |= GP_DATA_CACHE_IS_DIRTY;
  }

  static const bool poll(const bContext *C)
  {
    return GPencilPaintOperation::poll(C);
  }

 private:
  RandomSettings random_settings_;

  float2 initial_mouse_location;

  Vector<tGPspoint> point_buffer;
  Vector<tGPspoint> processed_point_buffer;
  float stroke_diameter;
  double init_time;

  bool is_vertex_fill;
  bool is_vertex_stroke;

  float4 vertex_color;

  bool is_stroke_on_plane = true;
  ViewDepths *view_depths = nullptr;

  void update_sbuffer_from_point_buffer(Span<tGPspoint> buffer)
  {
    gpd_->runtime.sbuffer_used = buffer.size();
    gpd_->runtime.sbuffer = ED_gpencil_sbuffer_ensure(
        static_cast<tGPspoint *>(gpd_->runtime.sbuffer),
        &gpd_->runtime.sbuffer_size,
        &gpd_->runtime.sbuffer_used,
        false);
    MutableSpan<tGPspoint> sbuffer_span(static_cast<tGPspoint *>(gpd_->runtime.sbuffer),
                                        gpd_->runtime.sbuffer_used);
    sbuffer_span.copy_from(buffer);
  }

  void apply_point_randomness(tGPspoint &point)
  {
    float value;
    /* Apply jitter (random change in perpendicular position). */
    if (!point_buffer.is_empty() && brush_settings_->draw_jitter > 0.0f) {
      float rand = BLI_rng_get_float(rng_) * 2.0f - 1.0f;
      float value = rand * 2.0f * brush_settings_->draw_jitter;
      if (brush_settings_->flag & GP_BRUSH_USE_JITTER_PRESSURE) {
        value *= BKE_curvemapping_evaluateF(
            brush_settings_->curve_jitter, 0, random_settings_.pen_press);
      }

      float vec[2];
      tGPspoint &pt_prev = point_buffer.last();
      sub_v2_v2v2(vec, point.m_xy, pt_prev.m_xy);
      normalize_v2(vec);
      rotate_v2_fl(vec, M_PI_2);

      /* Scale by displacement amount, and apply. */
      madd_v2_v2fl(point.m_xy, vec, value * 10.0f);
    }

    /* Apply randomness to pressure. */
    if (brush_settings_->draw_random_press > 0.0f) {
      if ((brush_settings_->flag2 & GP_BRUSH_USE_PRESS_AT_STROKE) == 0) {
        float rand = BLI_rng_get_float(rng_) * 2.0f - 1.0f;
        value = 1.0 + rand * 2.0 * brush_settings_->draw_random_press;
      }
      else {
        value = 1.0 + random_settings_.pressure * brush_settings_->draw_random_press;
      }

      /* Apply random curve. */
      if (brush_settings_->flag2 & GP_BRUSH_USE_PRESSURE_RAND_PRESS) {
        value *= BKE_curvemapping_evaluateF(
            brush_settings_->curve_rand_pressure, 0, random_settings_.pen_press);
      }

      point.pressure *= value;
      CLAMP(point.pressure, 0.1f, 1.0f);
    }

    /* Apply randomness to color strength. */
    if (brush_settings_->draw_random_strength > 0.0f) {
      if ((brush_settings_->flag2 & GP_BRUSH_USE_STRENGTH_AT_STROKE) == 0) {
        float rand = BLI_rng_get_float(rng_) * 2.0f - 1.0f;
        value = 1.0 + rand * brush_settings_->draw_random_strength;
      }
      else {
        value = 1.0 + random_settings_.strength * brush_settings_->draw_random_strength;
      }

      /* Apply random curve. */
      if (brush_settings_->flag2 & GP_BRUSH_USE_STRENGTH_RAND_PRESS) {
        value *= BKE_curvemapping_evaluateF(
            brush_settings_->curve_rand_pressure, 0, random_settings_.pen_press);
      }

      point.strength *= value;
      CLAMP(point.strength, GPENCIL_STRENGTH_MIN, 1.0f);
    }

    /* Apply randomness to uv texture rotation. */
    if (brush_settings_->uv_random > 0.0f) {
      if ((brush_settings_->flag2 & GP_BRUSH_USE_UV_AT_STROKE) == 0) {
        float rand = BLI_hash_int_01(BLI_hash_int_2d(static_cast<int>(point.m_xy[0]),
                                                     gpd_->runtime.sbuffer_used)) *
                         2.0f -
                     1.0f;
        value = rand * M_PI_2 * brush_settings_->uv_random;
      }
      else {
        value = random_settings_.uv * M_PI_2 * brush_settings_->uv_random;
      }

      /* Apply random curve. */
      if (brush_settings_->flag2 & GP_BRUSH_USE_UV_RAND_PRESS) {
        value *= BKE_curvemapping_evaluateF(
            brush_settings_->curve_rand_uv, 0, random_settings_.pen_press);
      }

      point.uv_rot += value;
      CLAMP(point.uv_rot, -M_PI_2, M_PI_2);
    }
  }

  void append_and_process_point_buffer(tGPspoint pt)
  {
    /* Merge points that are directly on top of each other. */
    if (!point_buffer.is_empty() && float2(point_buffer.last().m_xy) == float2(pt.m_xy)) {
      point_buffer.last() = pt;
    }
    else {
      /* Pre-subdivide for more acurate adaptive sampling. */
      if (point_buffer.size() > 1 && brush_settings_->sample_mode == GP_BRUSH_SAMPLE_ADAPTIVE &&
          brush_settings_->input_samples > 1) {
        tGPspoint prev_pt = point_buffer.last();
        int subdivisions = brush_settings_->input_samples;
        for (int j = 0; j < subdivisions; j++) {
          float fac = static_cast<float>(j + 1) / static_cast<float>(subdivisions);
          tGPspoint new_pt = {0.0f};
          new_pt.strength = interpf(pt.strength, prev_pt.strength, fac);
          new_pt.pressure = interpf(pt.pressure, prev_pt.pressure, fac);
          new_pt.uv_rot = interpf(pt.uv_rot, prev_pt.uv_rot, fac);
          new_pt.time = interpf(pt.time, prev_pt.time, fac);
          interp_v2_v2v2(new_pt.m_xy, prev_pt.m_xy, pt.m_xy, fac);
          interp_v4_v4v4(new_pt.vert_color, prev_pt.vert_color, pt.vert_color, fac);
          point_buffer.append(new_pt);
        }
      }
      else {
        /* Append the new point. */
        point_buffer.append(pt);
      }
    }

    /* Duplcaite point buffer. */
    processed_point_buffer.clear();
    processed_point_buffer = point_buffer;

    /* Apply active smoothing. */
    if ((brush_settings_->flag & GP_BRUSH_USE_ACTIVE_SMOOTHING) != 0) {
      int iterations = (brush_settings_->sample_mode == GP_BRUSH_SAMPLE_ADAPTIVE &&
                        brush_settings_->input_samples > 0) ?
                           brush_settings_->draw_smoothlvl * brush_settings_->input_samples :
                           brush_settings_->draw_smoothlvl;
      for (int i = 0; i < point_buffer.size(); i++) {
        smooth_buffer_point_position_2d(point_buffer,
                                        i,
                                        brush_settings_->active_smooth,
                                        iterations,
                                        false,
                                        true,
                                        false,
                                        processed_point_buffer);
        smooth_buffer_point_thickness_2d(processed_point_buffer,
                                         i,
                                         brush_settings_->active_smooth,
                                         iterations,
                                         false,
                                         processed_point_buffer);
        smooth_buffer_point_strength_2d(processed_point_buffer,
                                        i,
                                        brush_settings_->active_smooth,
                                        iterations,
                                        false,
                                        processed_point_buffer);
      }
    }

    /* Fixed sampling. */
    if (brush_settings_->sample_mode == GP_BRUSH_SAMPLE_FIXED &&
        processed_point_buffer.size() > 1 && brush_settings_->sample_distance > 0.0f) {

      Vector<tGPspoint> fixed_sampled_buffer;
      fixed_sampled_buffer.reserve(processed_point_buffer.size());
      fixed_sampled_buffer.append(processed_point_buffer.first());
      const float target_length = brush_settings_->sample_distance * stroke_diameter;

      float3 start_co, end_co;
      project_buffer_point_to_3d(
          scene_, region_, obact_, &processed_point_buffer.first(), nullptr, start_co);
      for (int i = 0; i < processed_point_buffer.size() - 1; i++) {
        tGPspoint start_pt = processed_point_buffer[i];
        tGPspoint end_pt = processed_point_buffer[i + 1];
        project_buffer_point_to_3d(scene_, region_, obact_, &end_pt, nullptr, end_co);
        float segment_length = len_v3v3(start_co, end_co);
        if (segment_length > target_length) {
          int subdivisions = static_cast<int>(ceilf((segment_length / target_length) - 1.0f));
          for (int j = 0; j < subdivisions; j++) {
            float fac = static_cast<float>(j + 1) / static_cast<float>(subdivisions + 1);
            tGPspoint pt_new = {0.0f};
            pt_new.strength = interpf(end_pt.strength, start_pt.strength, fac);
            pt_new.pressure = interpf(end_pt.pressure, start_pt.pressure, fac);
            pt_new.uv_rot = interpf(end_pt.uv_rot, start_pt.uv_rot, fac);
            pt_new.time = interpf(end_pt.time, start_pt.time, fac);
            interp_v2_v2v2(pt_new.m_xy, start_pt.m_xy, end_pt.m_xy, fac);
            interp_v4_v4v4(pt_new.vert_color, start_pt.vert_color, end_pt.vert_color, fac);
            fixed_sampled_buffer.append(pt_new);
          }
        }
        fixed_sampled_buffer.append(end_pt);
        start_co = end_co;
      }

      processed_point_buffer = std::move(fixed_sampled_buffer);
    }
  }

  void post_process_point_buffer()
  {
    bool changed = false;
    /* Curve fitting. */
    if ((brush_settings_->flag & GP_BRUSH_USE_CURVE_SMOOTHING) != 0) {
      float threshold = square_f(brush_settings_->curve_smooth_threshold) / 100.0f;
      Vector<tGPspoint> curve = fit_curve_2d_to_point_buffer(
          processed_point_buffer, threshold, 32, false);
      interp_polyline_polycurve(processed_point_buffer.as_mutable_span(),
                                curve.as_span(),
                                brush_settings_->curve_smooth_factor,
                                false);
      changed = true;
    }

    /* Adaptive smapling. */
    if (brush_settings_->sample_mode == GP_BRUSH_SAMPLE_ADAPTIVE &&
        processed_point_buffer.size() > 0) {
      int sample_size = processed_point_buffer.size();
      int begin_index = 0;
      int end_index = processed_point_buffer.size() - 1;

      Array<bool> position_marked_as_keep(sample_size, true);
      simplify_point_buffer_position(processed_point_buffer,
                                     begin_index,
                                     end_index,
                                     brush_settings_->simplify_f,
                                     position_marked_as_keep);

      Array<bool> thickness_marked_as_keep(sample_size, true);
      simplify_point_buffer_thickness(processed_point_buffer,
                                      begin_index,
                                      end_index,
                                      brush_settings_->simplify_f,
                                      thickness_marked_as_keep);

      Array<bool> strength_marked_as_keep(sample_size, true);
      simplify_point_buffer_strength(processed_point_buffer,
                                     begin_index,
                                     end_index,
                                     brush_settings_->simplify_f,
                                     strength_marked_as_keep);

      Vector<tGPspoint> adaptive_sampled_buffer;
      for (int i = begin_index; i <= end_index; i++) {
        int idx = i - begin_index;
        if (position_marked_as_keep[idx] || thickness_marked_as_keep[idx] ||
            strength_marked_as_keep[idx]) {
          adaptive_sampled_buffer.append(processed_point_buffer[i]);
        }
      }

      processed_point_buffer = std::move(adaptive_sampled_buffer);
      changed = true;
    }

    if (changed) {
      update_sbuffer_from_point_buffer(processed_point_buffer);
    }
  }

  bGPDstroke *create_stroke_from_sbuffer(bGPDframe *gpf)
  {
    /* Convert the sbuffer to a bGPDstroke. */
    bGPDstroke *gps = MEM_new<bGPDstroke>(__func__);
    gps->totpoints = gpd_->runtime.sbuffer_used;
    gps->mat_nr = std::max(0, gpd_->runtime.matid - 1);
    gps->flag = gpd_->runtime.sbuffer_sflag;
    gps->thickness = brush_->size;
    gps->hardeness = brush_settings_->hardeness;
    copy_v2_v2(gps->aspect_ratio, brush_settings_->aspect_ratio);
    gps->fill_opacity_fac = 1.0f;

    gps->tot_triangles = std::max(0, gpd_->runtime.sbuffer_used - 2);
    gps->caps[0] = gps->caps[1] = GP_STROKE_CAP_ROUND;
    gps->runtime.stroke_start = 1; /* Add one for the adjacency index. */
    ED_gpencil_fill_vertex_color_set(ts_, brush_, gps);
    /* Caps. */
    gps->caps[0] = gps->caps[1] = brush_settings_->caps_type;

    gps->points = static_cast<bGPDspoint *>(
        MEM_mallocN(gps->totpoints * sizeof(*gps->points), __func__));

    float origin[3], gpf_inv_mat[4][4];
    const bool use_frame_offset = (ts_->gpencil_flags & GP_TOOL_FLAG_USE_FRAME_OFFSET_MATRIX) != 0;
    ED_gpencil_drawing_reference_get(scene_, ob_eval_, ts_->gpencil_v3d_align, origin);
    if (use_frame_offset) {
      invert_m4_m4(gpf_inv_mat, gpf->transformation_mat);
    }

    float diff_mat[4][4], diff_imat[4][4];
    BKE_gpencil_layer_transform_matrix_get(depsgraph_, obact_, gpl_active_, diff_mat);
    invert_m4_m4(diff_imat, diff_mat);

    float *smooth_weights;
    if (!is_stroke_on_plane) {
      smooth_weights = static_cast<float *>(
          MEM_callocN(sizeof(float) * gps->totpoints, "stroke draw smooth weights"));
    }

    for (int i = 0; i < gpd_->runtime.sbuffer_used; i++) {
      tGPspoint *tpoint = &(static_cast<tGPspoint *>(gpd_->runtime.sbuffer))[i];
      bGPDspoint *pt = &gps->points[i];
      ED_gpencil_tpoint_to_point(region_, origin, reinterpret_cast<::tGPspoint *>(tpoint), pt);
      if (!is_stroke_on_plane && (view_depths != nullptr)) {
        if (project_tpoint_to_point_on_surface(tpoint, pt)) {
          smooth_weights[i] = 1.0f;
        }
      }
      else {
        ED_gpencil_project_point_to_plane(
            scene_, ob_eval_, gpl_active_, rv3d_, origin, ts_->gp_sculpt.lock_axis - 1, pt);
      }
      mul_m4_v3(diff_imat, &pt->x);
      if (use_frame_offset) {
        mul_m4_v3(gpf_inv_mat, &pt->x);
      }
      ED_gpencil_point_vertex_color_set(ts_, brush_, pt, tpoint);
      pt->time = tpoint->time;
      pt->flag = 0;
    }

    if (!is_stroke_on_plane) {
      /* We avoid artifacts by smoothing the stroke on the surface after projection. This is needed
       * since the depth buffer has a limited resolution and points might be projected to the same
       * location. */
      int iterations = (brush_settings_->sample_mode == GP_BRUSH_SAMPLE_ADAPTIVE &&
                        brush_settings_->input_samples > 0) ?
                           brush_settings_->draw_smoothlvl * brush_settings_->input_samples :
                           brush_settings_->draw_smoothlvl;
      BKE_gpencil_stroke_smooth(gps,
                                brush_settings_->active_smooth,
                                iterations,
                                true,
                                false,
                                false,
                                false,
                                true,
                                smooth_weights);
      MEM_freeN(smooth_weights);
    }

    gps->mat_nr = BKE_gpencil_object_material_get_index_from_brush(obact_, brush_);
    if (gps->mat_nr < 0) {
      if (obact_->actcol - 1 < 0) {
        gps->mat_nr = 0;
      }
      else {
        gps->mat_nr = obact_->actcol - 1;
      }
    }

    gps->inittime = init_time;

    /* Calc uv data along the stroke. */
    BKE_gpencil_stroke_uv_update(gps);

    BKE_gpencil_stroke_geometry_update(gpd_, gps);

    return gps;
  }

  void cleanup_sbuffer()
  {
    if (gpd_->runtime.sbuffer) {
      MEM_SAFE_FREE(gpd_->runtime.sbuffer);
      gpd_->runtime.sbuffer = nullptr;
    }

    gpd_->runtime.sbuffer_used = 0;
    gpd_->runtime.sbuffer_size = 0;
    gpd_->runtime.sbuffer_sflag = 0;
  }

  void init_random_settings(int2 mval)
  {
    int seed = (static_cast<int>(ceil(PIL_check_seconds_timer())) + 1) % 128;
    /* Use mouse position to get randomness. */
    int ix = mval[0] * seed;
    int iy = mval[1] * seed;
    int iz = ix + iy * seed;
    zero_v3(random_settings_.hsv);

    BrushGpencilSettings *brush_settings = brush_->gpencil_settings;
    /* Random to Hue. */
    if (brush_settings->random_hue > 0.0f) {
      float rand = BLI_hash_int_01(BLI_hash_int_2d(ix, iy)) * 2.0f - 1.0f;
      random_settings_.hsv[0] = rand * brush_settings->random_hue * 0.5f;
    }
    /* Random to Saturation. */
    if (brush_settings->random_saturation > 0.0f) {
      float rand = BLI_hash_int_01(BLI_hash_int_2d(iy, ix)) * 2.0f - 1.0f;
      random_settings_.hsv[1] = rand * brush_settings->random_saturation;
    }
    /* Random to Value. */
    if (brush_settings->random_value > 0.0f) {
      float rand = BLI_hash_int_01(BLI_hash_int_2d(ix * iz, iy * iz)) * 2.0f - 1.0f;
      random_settings_.hsv[2] = rand * brush_settings->random_value;
    }

    /* Random to pressure. */
    if (brush_settings->draw_random_press > 0.0f) {
      random_settings_.pressure = BLI_hash_int_01(BLI_hash_int_2d(ix + iz, iy + iz)) * 2.0f - 1.0f;
    }

    /* Random to color strength. */
    if (brush_settings->draw_random_strength) {
      random_settings_.strength = BLI_hash_int_01(BLI_hash_int_2d(ix + iy, iy + iz + ix)) * 2.0f -
                                  1.0f;
    }

    /* Random to uv texture rotation. */
    if (brush_settings->uv_random > 0.0f) {
      random_settings_.uv = BLI_hash_int_01(BLI_hash_int_2d(iy + iz, ix * iz)) * 2.0f - 1.0f;
    }
  }

  bool project_tpoint_to_point_on_surface(tGPspoint *tpt, bGPDspoint *pt)
  {
    int xy[2] = {(int)tpt->m_xy[0], (int)tpt->m_xy[1]};
    if (!view_depths || ((uint)xy[0] >= view_depths->w) || ((uint)xy[1] >= view_depths->h)) {
      return false;
    }
    float depth_fl = 1.0f;
    ED_view3d_depth_read_cached(view_depths, xy, 0, &depth_fl);
    const double depth = (double)depth_fl;
    if ((depth > view_depths->depth_range[0]) && (depth < view_depths->depth_range[1])) {
      if (ED_view3d_depth_unproject_v3(region_, xy, depth, &pt->x)) {
        float normal[3];
        if (ED_view3d_depth_read_cached_normal(region_, view_depths, xy, normal)) {
          madd_v3_v3fl(&pt->x, normal, stroke_diameter / 2.0f);
          return true;
        }
      }
    }

    return false;
  }
};

GPencilPaintOperation *new_gpencil_draw_operation(const bContext *C,
                                                  const float2 initial_mouse_location)
{
  if (GPencilDrawOperation::poll(C)) {
    return MEM_new<GPencilDrawOperation>(__func__, C, initial_mouse_location);
  }
  return nullptr;
}

}  // namespace blender::ed::sculpt_paint::gpencil