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

#include "SDFGeometry.h"

namespace brayns
{
SDFGeometry createSDFSphere(const Vector3f& center, const float radius,
                            const uint64_t data)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.center = center;
    geom.radius = radius;
    geom.type = SDFType::Sphere;
    return geom;
}

SDFGeometry createSDFPill(const Vector3f& p0, const Vector3f& p1,
                          const float radius, const uint64_t data)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.p0 = p0;
    geom.p1 = p1;
    geom.radius = radius;
    geom.type = SDFType::Pill;
    return geom;
}

SDFGeometry createSDFConePill(const Vector3f& p0, const Vector3f& p1,
                              const float radiusBottom, const float radiusTip,
                              const uint64_t data)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.p0 = p0;
    geom.p1 = p1;
    geom.radius = radiusBottom;
    geom.radius_tip = radiusTip;

    if (radiusBottom < radiusTip)
    {
        std::swap(geom.p0, geom.p1);
        std::swap(geom.radius, geom.radius_tip);
    }

    geom.type = SDFType::ConePill;
    return geom;
}

SDFGeometry createSDFConePillSigmoid(const Vector3f& p0, const Vector3f& p1,
                                     const float radiusBottom,
                                     const float radiusTip, const uint64_t data)
{
    SDFGeometry geom = createSDFConePill(p0, p1, radiusBottom, radiusTip, data);
    geom.type = SDFType::ConePillSigmoid;
    return geom;
}

SDFGeometry createSDFCubicBezier(const Vector3f& p0, const Vector3f& p1,
                                 const Vector3f& p2, const Vector3f& p3,
                                 const float r0, const float r1, const float r2,
                                 const float r3, const uint64_t data)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.p0 = p0;
    geom.center = p1;
    geom.p1 = p2;
    geom.p2 = p3;
    geom.radius = r0;
    geom.radius_mid = r1;
    geom.radius_tip = r2;
    geom.radius_p2 = r3;
    geom.type = SDFType::CubicBezier;

    // Calculate polynomial solving data

    // Expanding out the parametric cubic Bezier curver equation.
    const auto n = -1.f * p0 + 3.f * p1 + -3.f * p2 + p3;
    const auto r = 3.f * p0 + -6.f * p1 + 3.f * p2;
    const auto s = -3.f * p0 + 3.f * p1;
    const auto& v = p0;

    geom.polynomial_form[0] = n;
    geom.polynomial_form[1] = r;
    geom.polynomial_form[2] = s;
    geom.polynomial_form[3] = v; // p0

    // The derivative which is a quadratic equation.
    const auto j = 3.f * n;
    const auto k = 2.f * r;
    const auto& m = s;

    geom.derivative[0] = j;
    geom.derivative[1] = k;
    geom.derivative[2] = m;

    // - Dot(polynomial_form_, derivative_) divided by the leading coefficient
    geom.inv_leading_coefficient = -1.f / (j.dot(n));
    geom.precomputed_coefficients[0] = 1.0f;
    geom.precomputed_coefficients[1] = -(j.dot(r) + k.dot(n));
    geom.precomputed_coefficients[2] = -(j.dot(s) + k.dot(r) + m.dot(n));
    geom.precomputed_coefficients[3] = -(j.dot(v) + k.dot(s) + m.dot(r));
    geom.precomputed_coefficients[4] = -(k.dot(v) + m.dot(s));
    geom.precomputed_coefficients[5] = -m.dot(v);
    for (int i = 1; i < 6; ++i)
    {
        geom.precomputed_coefficients[i] *= geom.inv_leading_coefficient;
    }

    return geom;
}

Boxd getSDFBoundingBox(const SDFGeometry& geom)
{
    Boxd bounds;
    switch (geom.type)
    {
    case brayns::SDFType::Sphere:
    {
        bounds.merge(geom.center - Vector3f(geom.radius));
        bounds.merge(geom.center + Vector3f(geom.radius));
        break;
    }
    case brayns::SDFType::Pill:
    {
        bounds.merge(geom.p0 - Vector3f(geom.radius));
        bounds.merge(geom.p0 + Vector3f(geom.radius));
        bounds.merge(geom.p1 - Vector3f(geom.radius));
        bounds.merge(geom.p1 + Vector3f(geom.radius));
        break;
    }
    case brayns::SDFType::ConePill:
    case brayns::SDFType::ConePillSigmoid:
    {
        bounds.merge(geom.p0 - Vector3f(geom.radius));
        bounds.merge(geom.p0 + Vector3f(geom.radius));
        bounds.merge(geom.p1 - Vector3f(geom.radius_tip));
        bounds.merge(geom.p1 + Vector3f(geom.radius_tip));
        break;
    }
    case brayns::SDFType::CubicBezier:
    {
        const float max_radius =
            std::max(geom.radius_tip, std::max(geom.radius, geom.radius_mid));
        bounds.merge(geom.p0 - Vector3f(max_radius));
        bounds.merge(geom.p0 + Vector3f(max_radius));
        bounds.merge(geom.p1 - Vector3f(max_radius));
        bounds.merge(geom.p1 + Vector3f(max_radius));
        break;
    }
    default:
        throw std::runtime_error("No bounds found for SDF type.");
    }
    return bounds;
}

} // namespace brayns
