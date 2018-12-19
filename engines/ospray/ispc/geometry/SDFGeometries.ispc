/* Copyright (c) 2015-2018, EPFL/Blue Brain Project
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Jonas Karlsson <jonas.karlsson@epfl.ch>
 *
 * This file is part of Brayns <https://github.com/BlueBrain/Brayns>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

// ospray
#include "ospray/SDK/common/Model.ih"
#include "ospray/SDK/common/Ray.ih"
#include "ospray/SDK/geometry/Geometry.ih"
#include "ospray/SDK/math/box.ih"
#include "ospray/SDK/math/vec.ih"
#include "ospray/version.h"

// embree
#include "embree3/rtcore.isph"
#include "embree3/rtcore_geometry.isph"
#include "embree3/rtcore_scene.isph"
#include "utils/SafeIncrement.ih"

#define SDF_TYPE_SPHERE 0
#define SDF_TYPE_PILL 1
#define SDF_TYPE_CONE_PILL 2
#define SDF_TYPE_CONE_PILL_SIGMOID 3
#define SDF_TYPE_CUBIC_BEZIER 4

#define SDF_BLEND_FACTOR 0.1
#define SDF_BLEND_LERP_FACTOR 0.2

#define SDF_EPSILON 0.000001

#define kTolerance 0.00001
#define STURM_SIZE 21

#define MAGIC_FAIL 1234567.f
#define M_LN2 0.69314718055994530942
/////////////////////////////////////////////////////////////////////////////

// NOTE: This layout must match exactly the 'SDFGeometry' struct in
// 'SDFGeometry.h'
struct SDFGeometry
{
    uint64 userData;
    vec3f center;
    vec3f p0;
    vec3f p1;
    vec3f p2;
    float radius;
    float radius_mid;
    float radius_tip;
    float radius_p2;
    uint64 neighboursIndex;
    uint8 numNeighbours;
    uint8 type;
    vec3f derivative[3];
    float inv_leading_coefficient;
    float precomputed_coefficients[6];
    vec3f polynomial_form[4];
};

struct SDFGeometries
{
    uniform Geometry super;

    uniform SDFGeometry* uniform geometries;
    uniform uint64* uniform geometryRefs;
    uniform uint64* uniform neighbours;

    uniform bool useSafeIncrement;
};

float length_sq(vec3f& p)
{
    return p.x * p.x + p.y * p.y + p.z * p.z;
}

DEFINE_SAFE_INCREMENT(SDFGeometry);
DEFINE_SAFE_INCREMENT(uint64);

/////////////////////////////////////////////////////////////////////////////

struct SturmInterval
{
    varying float min;
    varying float max;
    varying int sign_min; // Sign changes for the minimum bound.
    varying int sign_max; // Sign changes for the max bound.
    varying int id;       // Id shared between this interval and its sibling.
    varying int expected_roots; // Total roots expected in this interval and its
                                // sibling.
};

SturmInterval createSturmInterval(float _min, float _max, int _sign_min,
                                  int _sign_max, int _id, int _roots)
{
    uniform SturmInterval s;
    s.min = _min;
    s.max = _max;
    s.sign_min = _sign_min;
    s.sign_max = _sign_max;
    s.id = _id;
    s.expected_roots = _roots;
    return s;
}

struct Interval
{
    float min;
    float max;
};

Interval toInterval(const SturmInterval& si)
{
    Interval i;
    i.min = si.min;
    i.max = si.max;
    return i;
}

struct SturmSequence5
{
    float data[STURM_SIZE];
};

struct QuinticSolver
{
    //    float tolerance_;
    varying SturmInterval interval_storage[100]; // FIXME, find proper size
    varying uint32 num_stored;
    uniform uint32 max_divisions;
};

struct Polynomial5
{
    varying float equation[6];
};

struct OutRoots
{
    varying float data[5];
};

struct CubicBezierSpline
{
    uniform vec3f control_points[4];
    uniform vec3f polynomial_form[4]; // Coefficents derived from the control
                                      // points.
    uniform vec3f derivative[3];
    uniform float precomputed_coefficients[6];
    uniform float inv_leading_coefficient;
};

vec3f EvaluateAt(const uniform CubicBezierSpline& s, const float t)
{
    // The polynomial for is faster at evaluating than the parametric.
    return t * (s.polynomial_form[2] +
                t * (s.polynomial_form[1] + t * s.polynomial_form[0])) +
           s.polynomial_form[3];
}

// Helper to solve a polynomial of a given degree.
float EvaluatePolynomial(uniform Polynomial5& polynomial, const int start,
                         const int degree, const float t)
{
    float result = 0.f;
    for (int i = start; i < start + degree; ++i)
    {
        result += polynomial.equation[i];
        result *= t;
    }
    result += polynomial.equation[start + degree];

    return result;
}

// Helper to solve a polynomial of a given degree.
float EvaluatePolynomial(uniform SturmSequence5& polynomial, const int start,
                         const int degree, const float t)
{
    float result = 0.f;
    for (int i = start; i < start + degree; ++i)
    {
        result += polynomial.data[i];
        result *= t;
    }
    result += polynomial.data[start + degree];

    return result;
}

// Input: polynomial with a leading coefficient of 1.0.
float EvaluateNormedEquation(uniform Polynomial5& polynomial, const float t)
{
    return EvaluatePolynomial(polynomial, 0, 5, t);
}

void BuildSturmSequence(uniform QuinticSolver& solver,
                        uniform Polynomial5& polynomial,
                        uniform SturmSequence5& out_sturm_polynomials)
{
    const int degree = 5;
    double sturm_polys[STURM_SIZE];

    for (int i = 0; i <= degree; ++i)
    {
        sturm_polys[i] = polynomial.equation[i];
        sturm_polys[degree + 1 + i] =
            (degree - i) * sturm_polys[i]; // derivative
    }

    int index = 6;
    int element_idx = 6 + 5; // start of third polynomial
    for (int i = 2; i <= degree; ++i)
    {
        const int poly = degree - i;
        const int p1 = index;
        const int p2 = index - (poly + 3);
        index += poly + 2;

        // long polynomial division
        const double remainder_coefficeint =
            (double)sturm_polys[p2 + 0] / sturm_polys[p1 + 0];
        const double remainder_sum =
            (sturm_polys[p2 + 1] -
             remainder_coefficeint * sturm_polys[p1 + 1]) /
            sturm_polys[p1 + 0];
        for (int c = 0; c <= poly; ++c, ++element_idx)
        {
            double coefficient = remainder_sum * sturm_polys[p1 + c + 1] -
                                 sturm_polys[p2 + c + 2];
            if (c != poly)
            {
                coefficient += remainder_coefficeint * sturm_polys[p1 + c + 2];
            }

            sturm_polys[element_idx] = coefficient;
        }
    }

    for (int i = 0; i < STURM_SIZE; ++i)
    {
        out_sturm_polynomials.data[i] = (float)sturm_polys[i];
    }
}

int CountSturmSignChanges(uniform SturmSequence5& sturm_polynomials,
                          const float t)
{
    int sign_changes = 0;
    int index = STURM_SIZE - 1;
    bool previous_positive = sturm_polynomials.data[index] >= 0.f;
    float solution = 0.f;
    for (int i = 1; i <= 5; ++i)
    {
        index -= i + 1;
        solution = EvaluatePolynomial(sturm_polynomials, index, i, t);
        bool curr_positive = solution >= 0.f;
        sign_changes += curr_positive != previous_positive;
        previous_positive = curr_positive;
    }

    return sign_changes;
}

float SolveBisection(uniform QuinticSolver& solver,
                     uniform Polynomial5& polynomial, const float interval_min,
                     const float interval_max)
{
    // Only intervals of where p(min) < 0 and p(max) > 0 can contain a solution.
    // This is equivalent to the distance function from the given position to
    // the solution having a negative slope at the min and a positive slope at
    // max.
    if (EvaluateNormedEquation(polynomial, interval_min) > 0)
    {
        return MAGIC_FAIL;
    }
    if (EvaluateNormedEquation(polynomial, interval_max) < 0)
    {
        return MAGIC_FAIL;
    }

    const int max_iterations = solver.max_divisions;
    float bisection_min = interval_min;
    float bisection_max = interval_max;
    for (int i = 0; i < max_iterations; ++i)
    {
        const float mid = (bisection_max + bisection_min) / 2.f;

        const float r = EvaluateNormedEquation(polynomial, mid);
        const bool curr_positive = signbits(r);
        if (curr_positive)
        {
            if (r >= -kTolerance)
            {
                return mid;
            }
            bisection_min = mid;
        }
        else
        {
            if (r <= kTolerance)
            {
                return mid;
            }
            bisection_max = mid;
        }
    }

    return bisection_min;
}

int Solve(uniform QuinticSolver& solver, uniform Polynomial5& polynomial,
          uniform OutRoots& out_roots, const float interval_min,
          const float interval_max)
{
    uniform SturmSequence5 sturm_polynomials;
    BuildSturmSequence(solver, polynomial, sturm_polynomials);

    // Set up the first interval.
    solver.num_stored = 0;
    int sign_min = CountSturmSignChanges(sturm_polynomials, interval_min);
    int sign_max = CountSturmSignChanges(sturm_polynomials, interval_max);
    const int total_roots = sign_min - sign_max;
    int id = 0;

    solver.interval_storage[solver.num_stored] =
        createSturmInterval(interval_min, interval_max, sign_min, sign_max,
                            id++, total_roots);
    solver.num_stored += 1;

    Interval root_intervals[5];
    int found_roots = 0;

    // print("sign_min %", sign_min);
    // print("sign_max %", sign_max);

    // Isolate roots
    while (solver.num_stored > 0 && total_roots != found_roots)
    {
        SturmInterval i = solver.interval_storage[solver.num_stored - 1];
        solver.num_stored -= 1;

        int num_roots = i.sign_min - i.sign_max;
        // print("num_roots %", num_roots);

        if (num_roots <= 0)
        {
            // print("% a");

            if (solver.num_stored != 0 &&
                solver.interval_storage[solver.num_stored - 1].id == i.id)
            {
                // print("% a1");

                i = solver.interval_storage[solver.num_stored - 1];
                solver.num_stored -= 1;
                num_roots = i.expected_roots;
            }
            else
            {
                continue;
            }
        }

        // Prune sibling intervals based on the results of the current interval.
        if (num_roots == i.expected_roots && (solver.num_stored != 0) &&
            solver.interval_storage[solver.num_stored - 1].id == i.id)
        {
            // print("% b");

            solver.num_stored -= 1;
        }
        else if (num_roots == i.expected_roots - 1 &&
                 (solver.num_stored != 0) &&
                 solver.interval_storage[solver.num_stored - 1].id ==
                     i.id) // This case was a biggest perf improvemnt.
        {
            // print("% b1");

            const SturmInterval si_tmp =
                solver.interval_storage[solver.num_stored - 1];
            root_intervals[found_roots++] = toInterval(si_tmp);
            solver.num_stored -= 1;
        }

        if (num_roots == 1)
        {
            // print("% c");

            root_intervals[found_roots++] = toInterval(i);
        }
        else
        {
            // print("% c1");

            float mid = (i.min + i.max) / 2.f;
            if (mid - i.min <= kTolerance)
            {
                // print("% c2");

                root_intervals[found_roots++] = toInterval(i);
            }
            else
            {
                // print("% ac3");

                // Divide the current interval and search deeper.
                const int sign_mid =
                    CountSturmSignChanges(sturm_polynomials, mid);
                solver.interval_storage[solver.num_stored] =
                    createSturmInterval(i.min, mid, i.sign_min, sign_mid, id,
                                        num_roots);
                solver.num_stored += 1;
                solver.interval_storage[solver.num_stored] =
                    createSturmInterval(mid, i.max, sign_mid, i.sign_max, id,
                                        num_roots);
                solver.num_stored += 1;
                ++id;
            }
        }
    }

    // print("%", found_roots);

    int num_real_roots = 0;
    for (int i = 0; i < found_roots; ++i)
    {
        // print("% d");

        const Interval interval = root_intervals[i];
        float root =
            SolveBisection(solver, polynomial, interval.min, interval.max);
        // print("root %", root);
        if (root != MAGIC_FAIL)
        {
            out_roots.data[num_real_roots++] = root;
        }
    }

    // print("%", num_real_roots);

    return num_real_roots;
}

float ClosestPointToSpline(const uniform CubicBezierSpline& spline,
                           uniform QuinticSolver& solver, const vec3f& position,
                           vec3f& closest)
{
    uniform Polynomial5 quintic;
    for (int i = 0; i < 6; i++)
    {
        quintic.equation[i] = spline.precomputed_coefficients[i];
    }

    for (int i = 0; i < 3; ++i)
    {
        quintic.equation[3 + i] += dot(position, spline.derivative[i]) *
                                   spline.inv_leading_coefficient;
    }

    uniform OutRoots realRoots;
    const int roots =
        Solve(solver, quintic, realRoots, kTolerance, 1.f - kTolerance);

    // Test the first control point.
    vec3f min_position = spline.control_points[0];
    float min_dist_sq = length_sq(position - min_position);

    // Test the roots.
    for (int i = 0; i < roots; ++i)
    {
        const vec3f root_position = EvaluateAt(spline, realRoots.data[i]);
        const float root_dist_sq = length_sq(position - root_position);
        if (root_dist_sq < min_dist_sq)
        {
            min_dist_sq = root_dist_sq;
            min_position = root_position;
        }
    }

    // Test the last control point.
    const float dist_sq = length_sq(position - spline.control_points[3]);
    if (dist_sq < min_dist_sq)
    {
        min_dist_sq = dist_sq;
        min_position = spline.control_points[3];
    }

    closest = min_position;
    return min_dist_sq;
}

/////////////////////////////////////////////////////////////////////////////

// https://en.wikipedia.org/wiki/Smoothstep
inline float smootherstep(const float x)
{
    return x * x * x * (x * (x * 6 - 15) + 10);
}

inline float sdSphere(const vec3f& p, const vec3f c, float r)
{
    return length(p - c) - r;
}

inline float sdCapsule(const vec3f& p, const vec3f a, const vec3f b, float r)
{
    const vec3f pa = p - a, ba = b - a;
    const float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h) - r;
}

inline float mix(const float x, const float y, const float a)
{
    return x * (1.f - a) + y * a;
}

// polynomial smooth min (k = 0.1);
inline float sminPoly(const float a, const float b, const float k)
{
    const float h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return mix(b, a, h) - k * h * (1.0 - h);
}

inline float sdConePill(const vec3f& p, const vec3f p0, const vec3f p1,
                        const float radius_bottom, const float radius_top,
                        const bool useSigmoid)
{
    const vec3f v = p1 - p0;
    const vec3f w = p - p0;

    const float c1 = dot(w, v);
    if (c1 <= 0)
        return length(p - p0) - radius_bottom;

    const float c2 = dot(v, v);
    if (c2 <= c1)
        return length(p - p1) - radius_top;

    const float b = c1 / c2;
    const vec3f Pb = p0 + b * v;

    const float thicknessAt = mix(radius_bottom, radius_top, b);
    const float thickness =
        useSigmoid
            ? 0.5 * (thicknessAt +
                     mix(radius_bottom, radius_top, smootherstep(b)))
            : thicknessAt;

    return length(p - Pb) - thickness;
}

vec3f evaluateBezierCurve(const float t, const vec3f& p0, const vec3f& p1,
                          const vec3f& p2, const vec3f& p3)
{
    const vec3f n = -1.f * p0 + 3.f * p1 + -3.f * p2 + p3;
    const vec3f r = 3.f * p0 + -6.f * p1 + 3.f * p2;
    const vec3f s = -3.f * p0 + 3.f * p1;
    const vec3f& v = p0;

    return t * (s + t * (r + t * n)) + v;
}

float getBezierRadius(const float t, const float r0, const float r1,
                      const float r2, const float r3)
{
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float mt = 1 - t;
    const float mt2 = mt * mt;
    const float mt3 = mt2 * mt;
    return r0 * mt3 + 3 * r1 * mt2 * t + 3 * r2 * mt * t2 + r3 * t3;
}

float evaluateBezierCurveDistance(const vec3f& p, const float t,
                                  const vec3f& p0, const vec3f& p1,
                                  const vec3f& p2, const vec3f& p3,
                                  const float r0, const float r1,
                                  const float r2, const float r3)
{
    const vec3f p_closest = evaluateBezierCurve(t, p0, p1, p2, p3);

    const float dx = p.x - p_closest.x;
    const float dy = p.y - p_closest.y;
    const float dz = p.z - p_closest.z;

    const float thick = getBezierRadius(t, r0, r1, r2, r3);

    return sqrtf(dx * dx + dy * dy + dz * dz) - thick;
}

inline float sdCubicBezier(const vec3f& p, const uniform SDFGeometry& primitive)
{
    uniform CubicBezierSpline spline;
    spline.control_points[0] = primitive.p0;
    spline.control_points[1] = primitive.center;
    spline.control_points[2] = primitive.p1;
    spline.control_points[3] = primitive.p2;

    spline.polynomial_form[0] = primitive.polynomial_form[0];
    spline.polynomial_form[1] = primitive.polynomial_form[1];
    spline.polynomial_form[2] = primitive.polynomial_form[2];
    spline.polynomial_form[3] = primitive.polynomial_form[3];

    spline.derivative[0] = primitive.derivative[0];
    spline.derivative[1] = primitive.derivative[1];
    spline.derivative[2] = primitive.derivative[2];

    spline.precomputed_coefficients[0] = primitive.precomputed_coefficients[0];
    spline.precomputed_coefficients[1] = primitive.precomputed_coefficients[1];
    spline.precomputed_coefficients[2] = primitive.precomputed_coefficients[2];
    spline.precomputed_coefficients[3] = primitive.precomputed_coefficients[3];
    spline.precomputed_coefficients[4] = primitive.precomputed_coefficients[4];
    spline.precomputed_coefficients[5] = primitive.precomputed_coefficients[5];

    spline.inv_leading_coefficient = primitive.inv_leading_coefficient;

    uniform QuinticSolver solver;
    solver.max_divisions = 1 + (uint32)(logf(1.f / kTolerance) / M_LN2);
    solver.num_stored = 0;
    vec3f closest;

    const float d = ClosestPointToSpline(spline, solver, p, closest);

    return sqrt(d) - 1.0f;
#if 0
    return -1.f;

    const int num_samples = 10;
    const float step = 1.f / num_samples;

    float mindist = 9999999.f;
    float bestt = 0.f;

    for (int i = 0; i <= num_samples; i++)
    {
        const float t = i * step;
        const float dcurr =
            evaluateBezierCurveDistance(p, t, p0, p1, p2, p3, r0, r1, r2, r3);
        if (dcurr < mindist)
        {
            mindist = dcurr;
            bestt = t;
        }
    }

    float d = mindist;
    float t = bestt;
    float t_low = max(0.f, t - step);
    float t_high = min(1.f, t + step);
    const float epsilon = 0.0000000001f;

    while (true)
    {
        float t_low_next = t - 0.5f * abs(t - t_low);
        float t_high_next = t + 0.5f * abs(t - t_high);
        float d_low_next = evaluateBezierCurveDistance(p, t_low_next, p0, p1,
                                                       p2, p3, r0, r1, r2, r3);
        float d_high_next = evaluateBezierCurveDistance(p, t_high_next, p0, p1,
                                                        p2, p3, r0, r1, r2, r3);

        if (d <= epsilon)
        {
            return d;
        }

        if (d_low_next < d_high_next && abs(d_low_next - d) > epsilon)
        {
            t_high = t;
            t = t_low_next;
            d = d_low_next;
        }
        else if (d_high_next < d_low_next && abs(d_high_next - d) > epsilon)
        {
            t_low = t;
            t = t_high_next;
            d = d_high_next;
        }
        else
        {
            return d;
        }
    }

    return d;
#endif
    //    const float dx = p.x - pT.x;
    //    const float dy = p.y - pT.y;
    //    const float dz = p.z - pT.z;
    //
    //    const float thick = get_thickness(t, thickness);
    //
    //    return sqrtf(dx * dx + dy * dy + dz * dz) - thick;
    //    return -1;
}

// AABB-Ray intersection ( http://prideout.net/blog/?p=64 ).
inline varying bool intersectBox(Ray& ray, const vec3f aabbMin,
                                 const vec3f aabbMax, float& t0, float& t1)
{
    const float epsilon = SDF_EPSILON;

    Ray r = ray;
    // We need to avoid division by zero in "vec3 invR = 1.0 / r.Dir;"
    if (r.dir.x == 0)
        r.dir.x = epsilon;

    if (r.dir.y == 0)
        r.dir.y = epsilon;

    if (r.dir.z == 0)
        r.dir.z = epsilon;

    const vec3f invR = 1.f / r.dir;
    const vec3f tbot = invR * (aabbMin - r.org);
    const vec3f ttop = invR * (aabbMax - r.org);
    const vec3f tmin = min(ttop, tbot);
    const vec3f tmax = max(ttop, tbot);
    vec2f t = make_vec2f(max(tmin.x, tmin.y), max(tmin.x, tmin.z));
    t0 = max(t.x, t.y);
    t = make_vec2f(min(tmax.x, tmax.y), min(tmax.x, tmax.z));
    t1 = min(t.x, t.y);
    return (t0 <= t1);
}

/////////////////////////////////////////////////////////////////////////////

uniform uint64 primToIdx(const uniform SDFGeometries& geometry,
                         const uniform uint64 primID)
{
    return *safeIncrement(geometry.useSafeIncrement, geometry.geometryRefs,
                          primID);
}

varying uint64 primToIdxVarying(const uniform SDFGeometries& geometry,
                                const varying uint64 primID)
{
    return *safeIncrement(geometry.useSafeIncrement, geometry.geometryRefs,
                          primID);
}

uniform uint64 getNeighbourIdx(const uniform SDFGeometries& geometry,
                               const uniform uint64 startIdx,
                               const uniform uint64 neighIdx)
{
    return *safeIncrement(geometry.useSafeIncrement, geometry.neighbours,
                          startIdx + neighIdx);
}

const uniform SDFGeometry* uniform getPrimitive(
    const uniform SDFGeometries& geometry, const uniform uint64 idx)
{
    return safeIncrement(geometry.useSafeIncrement, geometry.geometries, idx);
}

const uniform SDFGeometry* varying getPrimitiveVarying(
    const uniform SDFGeometries& geometry, const varying uint64 idx)
{
    return safeIncrement(geometry.useSafeIncrement, geometry.geometries, idx);
}

static void SDFGeometries_postIntersect(uniform Geometry* uniform geometry,
                                        uniform Model* uniform model,
                                        varying DifferentialGeometry& dg,
                                        const varying Ray& ray,
                                        uniform int64 flags)
{
    dg.geometry = geometry;
    vec3f Ng = ray.Ng;
    vec3f Ns = Ng;

    if (flags & DG_NORMALIZE)
    {
        Ng = normalize(Ng);
        Ns = normalize(Ns);
    }
    if (flags & DG_FACEFORWARD)
    {
        if (dot(ray.dir, Ng) >= 0.f)
            Ng = neg(Ng);
        if (dot(ray.dir, Ns) >= 0.f)
            Ns = neg(Ns);
    }

    dg.Ng = Ng;
    dg.Ns = Ns;
}

uniform box3fa calcBounds(const uniform SDFGeometry& primitive)
{
    if (primitive.type == SDF_TYPE_SPHERE)
    {
        return make_box3fa(primitive.center - make_vec3f(primitive.radius),
                           primitive.center + make_vec3f(primitive.radius));
    }
    else if (primitive.type == SDF_TYPE_CUBIC_BEZIER)
    {
        const uniform vec3f minV = make_vec3f(
            min(primitive.p2.x,
                min(primitive.center.x, min(primitive.p0.x, primitive.p1.x))),
            min(primitive.p2.y,
                min(primitive.center.y, min(primitive.p0.y, primitive.p1.y))),
            min(primitive.p2.z,
                min(primitive.center.z, min(primitive.p0.z, primitive.p1.z))));

        const uniform vec3f maxV = make_vec3f(
            max(primitive.p2.x,
                max(primitive.center.x, max(primitive.p0.x, primitive.p1.x))),
            max(primitive.p2.y,
                max(primitive.center.y, max(primitive.p0.y, primitive.p1.y))),
            max(primitive.p2.z,
                max(primitive.center.z, max(primitive.p0.z, primitive.p1.z))));

        const uniform float maxRadius =
            max(primitive.radius_tip,
                max(primitive.radius_mid,
                    max(primitive.radius, primitive.radius_p2)));
        const uniform vec3f radiusV = make_vec3f(maxRadius);

        return make_box3fa(minV - radiusV, maxV + radiusV);
    }
    // else primitive.type == SDF_TYPE_CONE_PILL ||
    //      primitive.type == SDF_TYPE_PILL ||
    //      primitive.type == SDF_TYPE_CONE_PILL_SIGMOID
    const uniform vec3f minV = make_vec3f(min(primitive.p0.x, primitive.p1.x),
                                          min(primitive.p0.y, primitive.p1.y),
                                          min(primitive.p0.z, primitive.p1.z));

    const uniform vec3f maxV = make_vec3f(max(primitive.p0.x, primitive.p1.x),
                                          max(primitive.p0.y, primitive.p1.y),
                                          max(primitive.p0.z, primitive.p1.z));
    return make_box3fa(minV - make_vec3f(primitive.radius),
                       maxV + make_vec3f(primitive.radius));
}

void SDFGeometries_bounds(const RTCBoundsFunctionArguments* uniform args)
{
    const uniform SDFGeometries* uniform self =
        (const uniform SDFGeometries* uniform)args->geometryUserPtr;
    const uniform uint64 primID = args->primID;

    const uniform uint64 idx = primToIdx(*self, primID);

    *((box3fa * uniform)args->bounds_o) = calcBounds(*getPrimitive(*self, idx));
}

inline float calcDistance(const uniform SDFGeometry& primitive, const vec3f& p)
{
    if (primitive.type == SDF_TYPE_SPHERE)
        return sdSphere(p, primitive.center, primitive.radius);
    if (primitive.type == SDF_TYPE_PILL)
        return sdCapsule(p, primitive.p0, primitive.p1, primitive.radius);
    if (primitive.type == SDF_TYPE_CONE_PILL ||
        primitive.type == SDF_TYPE_CONE_PILL_SIGMOID)
    {
        return sdConePill(p, primitive.p0, primitive.p1, primitive.radius,
                          primitive.radius_tip,
                          primitive.type == SDF_TYPE_CONE_PILL_SIGMOID);
    }
    if (primitive.type == SDF_TYPE_CUBIC_BEZIER)
    {
        return sdCubicBezier(p, primitive);
    }

    return -1.0;
}

inline float map(const uniform SDFGeometry& primitive, const vec3f& p,
                 const uniform SDFGeometries& geometry)
{
    float d = calcDistance(primitive, p);
    const float r0 = primitive.radius;

    const uint8 numNeighs = primitive.numNeighbours;

    for (uniform int i = 0; i < numNeighs; i++)
    {
        const uniform uint64 index =
            getNeighbourIdx(geometry, primitive.neighboursIndex, i);

        const uniform SDFGeometry& neighbour = *getPrimitive(geometry, index);

        const float dOther = calcDistance(neighbour, p);
        const float r1 = neighbour.radius;
        const float blendFactor =
            mix(min(r0, r1), max(r0, r1), SDF_BLEND_LERP_FACTOR);

        d = sminPoly(dOther, d, blendFactor * SDF_BLEND_FACTOR);
    }

    return d;
}

inline vec3f calcNormal(const uniform SDFGeometry& primitive, const float eps,
                        const vec3f& pos, const uniform SDFGeometries& geometry)
{
    const float x0 = map(primitive, pos + make_vec3f(eps, 0.f, 0.f), geometry);
    const float x1 = map(primitive, pos - make_vec3f(eps, 0.f, 0.f), geometry);
    const float y0 = map(primitive, pos + make_vec3f(0.f, eps, 0.f), geometry);
    const float y1 = map(primitive, pos - make_vec3f(0.f, eps, 0.f), geometry);
    const float z0 = map(primitive, pos + make_vec3f(0.f, 0.f, eps), geometry);
    const float z1 = map(primitive, pos - make_vec3f(0.f, 0.f, eps), geometry);

    return normalize(make_vec3f(x0 - x1, y0 - y1, z0 - z1));
}

inline float intersect(const uniform SDFGeometry& primitive, const vec3f& ro,
                       const vec3f& rd, const uniform SDFGeometries& geometry)
{
    Ray ray;
    ray.dir = rd;
    ray.org = ro;

    const uniform box3fa bbox = calcBounds(primitive);

    float t0_pre, t1_pre;
    const vec3f aabbMin = make_vec3f(bbox.lower);
    const vec3f aabbMax = make_vec3f(bbox.upper);

    if (!intersectBox(ray, aabbMin, aabbMax, t0_pre, t1_pre))
        return -1;

    const float t0 = t0_pre;
    const float t1 = t1_pre;

    const int samplesPerRay = 100;
    float t = t0;
    float res = -1.f;
    float h = 1.f;
    for (int i = 0; i < samplesPerRay; ++i)
    {
        if (h < SDF_EPSILON || t > t1)
            break;
        h = map(primitive, ro + rd * t, geometry);
        res = t;
        t += h;
    }
    if (t > t1)
        res = -1.f;
    return res;
}

unmasked void SDFGeometries_intersect(
    const RTCIntersectFunctionNArguments* uniform args)
{
    const uniform SDFGeometries* uniform self =
        (uniform SDFGeometries * uniform)args->geometryUserPtr;
    const uniform int primID = args->primID;

    const uniform SDFGeometry& primitive =
        *getPrimitive(*self, primToIdx(*self, primID));

    varying Ray* uniform ray = (varying Ray * uniform)args->rayhit;

    const float t_in = intersect(primitive, ray->org, ray->dir, *self);
    if (t_in > 0 && t_in > ray->t0 && t_in < ray->t)
    {
        const vec3f pos = ray->org + t_in * ray->dir;

        ray->primID = primID;
        ray->geomID = self->super.geomID;
        ray->instID = args->context->instID[0];
        ray->t = t_in;
        ray->Ng = calcNormal(primitive, getIntersectionError(ray->org, t_in),
                             pos, *self);
    }
}

export void* uniform SDFGeometries_create(void* uniform cppEquivalent)
{
    uniform SDFGeometries* uniform geom = uniform new uniform SDFGeometries;
    Geometry_Constructor(&geom->super, cppEquivalent,
                         SDFGeometries_postIntersect, 0, 0, 0);
    return geom;
}

export void SDFGeometriesGeometry_set(void* uniform _self, void* uniform _model,
                                      void* uniform data,
                                      int uniform numPrimitives,
                                      void* uniform neighbours,
                                      int uniform numNeighbours,
                                      void* uniform geometries)
{
    uniform SDFGeometries* uniform self =
        (uniform SDFGeometries * uniform)_self;
    uniform Model* uniform model = (uniform Model * uniform)_model;

    RTCGeometry geom =
        rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);
    uniform uint32 geomID = rtcAttachGeometry(model->embreeSceneHandle, geom);

    self->super.model = model;
    self->super.geomID = geomID;
    self->super.numPrimitives = numPrimitives;
    self->geometryRefs = (uniform uint64 * uniform)data;
    self->neighbours = (uniform uint64 * uniform)neighbours;
    self->geometries = (uniform SDFGeometry * uniform)geometries;

    // NOTE: self->data is always smaller than self->geometries
    self->useSafeIncrement =
        needsSafeIncrement(self->geometries, numPrimitives) ||
        needsSafeIncrement(self->neighbours, numNeighbours);

    rtcSetGeometryUserData(geom, self);
    rtcSetGeometryUserPrimitiveCount(geom, numPrimitives);
    rtcSetGeometryBoundsFunction(
        geom, (uniform RTCBoundsFunction)&SDFGeometries_bounds, self);
    rtcSetGeometryIntersectFunction(
        geom, (uniform RTCIntersectFunctionN)&SDFGeometries_intersect);
    rtcSetGeometryOccludedFunction(
        geom, (uniform RTCOccludedFunctionN)&SDFGeometries_intersect);
    rtcCommitGeometry(geom);
    rtcReleaseGeometry(geom);
}

export void testCurve(void* uniform primitive_in)
{
    uniform SDFGeometry primitive = *((SDFGeometry*)primitive_in);

    for (int i = 74; i < 139; i++)
    {
        for (int j = 48; j < 100; j++)
        {
            vec3f p;
            p.x = (float)i;
            p.y = (float)j;
            p.z = 10.f;
            float d = sdCubicBezier(p, primitive);
            print("(%, %) %\n", i, j, d);
        }
    }
    // print("%\n", d);

    // output[0] = d;
}
