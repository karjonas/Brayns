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
    std::array<Vector3f, 3> derivative;
    float inv_leading_coefficient = 0.f;
    std::array<float, 6> precomputed_coefficients{
        {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
};

SDFGeometry createSDFSphere(const Vector3f& center, const float radius,
                            const uint64_t data = 0);

SDFGeometry createSDFPill(const Vector3f& p0, const Vector3f& p1,
                          const float radius, const uint64_t data = 0);

SDFGeometry createSDFConePill(const Vector3f& p0, const Vector3f& p1,
                              const float radiusBottom, const float radiusTip,
                              const uint64_t data = 0);

SDFGeometry createSDFConePillSigmoid(const Vector3f& p0, const Vector3f& p1,
                                     const float radiusBottom,
                                     const float radiusTip,
                                     const uint64_t data = 0);

SDFGeometry createSDFCubicBezier(const Vector3f& p0, const Vector3f& p1,
                                 const Vector3f& p2, const Vector3f& p3,
                                 const float r0, const float r1, const float r2,
                                 const float r3, const uint64_t data = 0);

Boxd getSDFBoundingBox(const SDFGeometry& geom);

} // namespace brayns
