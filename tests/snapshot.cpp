/* Copyright (c) 2018, EPFL/Blue Brain Project
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Daniel.Nachbaur@epfl.ch
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

#define BOOST_TEST_MODULE braynsSnapshot

#include <jsonSerialization.h>
#include <tests/paths.h>

#include "ClientServer.h"
#include <brayns/engine/Engine.h>
#include <brayns/engine/Renderer.h>

#include "tests/PDiffHelpers.h"

BOOST_GLOBAL_FIXTURE(ClientServer);

BOOST_AUTO_TEST_CASE(snapshot)
{
    brayns::SnapshotParams params;
    params.format = "png";
    params.size = {50, 50};
    params.quality = 90;

    auto image =
        makeRequest<brayns::SnapshotParams,
                    brayns::ImageGenerator::ImageBase64>("snapshot", params);

    BOOST_CHECK(compareBase64TestImage(image, "snapshot.png"));
}

BOOST_AUTO_TEST_CASE(snapshot_empty_params)
{
    BOOST_CHECK_THROW((makeRequest<brayns::SnapshotParams,
                                   brayns::ImageGenerator::ImageBase64>(
                          "snapshot", brayns::SnapshotParams())),
                      rockets::jsonrpc::response_error);
}

BOOST_AUTO_TEST_CASE(snapshot_illegal_format)
{
    brayns::SnapshotParams params;
    params.size = {5, 5};
    params.format = "";
    BOOST_CHECK_THROW(
        (makeRequest<brayns::SnapshotParams,
                     brayns::ImageGenerator::ImageBase64>("snapshot", params)),
        rockets::jsonrpc::response_error);
}
