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

#pragma once

#include <brayns/common/types.h>

namespace brayns
{
enum class SDFType : uint8_t
{
    Sphere = 0,
    Pill = 1,
    ConePill = 2,
    ConePillSigmoid = 3,
    CubicBezier = 4
};

struct SDFGeometry
{
    uint64_t userData;
    Vector3f center;
    Vector3f p0;
    Vector3f p1;
    Vector3f p2;
    float radius = -1.f;
    float raidus_mid = -1.f;
    float radius_tip = -1.f;
    float radius_p2 = -1.f;
    uint64_t neighboursIndex = 0;
    uint8_t numNeighbours = 0;
    SDFType type;
};

inline SDFGeometry createSDFSphere(const Vector3f& center, const float radius,
                                   const uint64_t data = 0)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.center = center;
    geom.radius = radius;
    geom.type = SDFType::Sphere;
    return geom;
}

inline SDFGeometry createSDFPill(const Vector3f& p0, const Vector3f& p1,
                                 const float radius, const uint64_t data = 0)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.p0 = p0;
    geom.p1 = p1;
    geom.radius = radius;
    geom.type = SDFType::Pill;
    return geom;
}

inline SDFGeometry createSDFConePill(const Vector3f& p0, const Vector3f& p1,
                                     const float radiusBottom,
                                     const float radiusTip,
                                     const uint64_t data = 0)
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

inline SDFGeometry createSDFConePillSigmoid(const Vector3f& p0,
                                            const Vector3f& p1,
                                            const float radiusBottom,
                                            const float radiusTip,
                                            const uint64_t data = 0)
{
    SDFGeometry geom = createSDFConePill(p0, p1, radiusBottom, radiusTip, data);
    geom.type = SDFType::ConePillSigmoid;
    return geom;
}

inline SDFGeometry createSDFCubicBezier(const Vector3f& p0, const Vector3f& p1,
                                        const Vector3f& p2, const Vector3f& p3,
                                        const float r0, const float r1,
                                        const float r2, const float r3,
                                        const uint64_t data = 0)
{
    SDFGeometry geom;
    geom.userData = data;
    geom.p0 = p0;
    geom.center = p1;
    geom.p1 = p2;
    geom.p2 = p3;
    geom.radius = r0;
    geom.raidus_mid = r1;
    geom.radius_tip = r2;
    geom.radius_p2 = r3;
    geom.type = SDFType::CubicBezier;
    return geom;
}

inline Boxd getSDFBoundingBox(const SDFGeometry& geom)
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
            std::max(geom.radius_tip, std::max(geom.radius, geom.raidus_mid));
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
