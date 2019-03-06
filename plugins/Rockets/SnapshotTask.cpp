/* Copyright (c) 2015-2019, EPFL/Blue Brain Project
 * All rights reserved. Do not distribute without permission.
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

#include <brayns/common/tasks/Task.h>

#include "ImageGenerator.h"
#include <brayns/common/utils/utils.h>
#include <brayns/engine/Camera.h>
#include <brayns/engine/Engine.h>
#include <brayns/engine/FrameBuffer.h>
#include <brayns/engine/Renderer.h>
#include <brayns/engine/Scene.h>
#include <brayns/parameters/ParametersManager.h>

#include "SnapshotTask.h"

namespace brayns
{
SnapshotFunctor::SnapshotFunctor(Engine &engine, SnapshotParams &&params,
                                 ImageGenerator &imageGenerator)
    : _params(std::move(params))
    , _imageGenerator(imageGenerator)
    , _engine(engine)
{
}

ImageGenerator::ImageBase64 SnapshotFunctor::operator()()
{
    auto lock{_engine.getRenderScopeLock()};

    auto &renderingParameters =
        _engine.getParametersManager().getRenderingParameters();
    auto &renderer = _engine.getRenderer();

    auto &camera = _engine.getCamera();

    const auto aspectPrev = camera.getProperty<double>("aspect");
    const auto sppPrev = renderingParameters.getSamplesPerPixel();

    camera.updateProperty("aspect", double(_params.size.x) / _params.size.y);
    renderingParameters.setSamplesPerPixel(_params.samplesPerPixel);
    renderer.commit();

    ImageGenerator::ImageBase64 image;

    // Only render one frame to avoid ghosting if camera is moved or simulation
    // frame changes between accumulation frames.
    {
        const auto nameShort =
            _params.name.empty() ? "" : " " + shortenString(_params.name);
        const auto msg = "Render snapshot" + nameShort + "...";

        const auto isStereo =
            camera.hasProperty("stereo") && camera.getProperty<bool>("stereo");
        const auto names = isStereo ? strings{"0L", "0R"} : strings{"default"};

        progress(msg, 0.1f, 0.f);

        std::vector<FrameBufferPtr> frameBuffers;
        for (const auto &name : names)
        {
            frameBuffers.push_back(
                _engine.createFrameBuffer(name, _params.size,
                                          FrameBufferFormat::rgba_i8));
        }

        for (auto &frameBuffer : frameBuffers)
        {
            camera.setBufferTarget(frameBuffer->getName());
            camera.markModified(false);
            camera.commit();
            camera.resetModified();
            renderer.render(frameBuffer);
            frameBuffer->incrementAccumFrames();
        }

        image = _imageGenerator.createImage(frameBuffers, _params.format,
                                            _params.quality);
    }

    renderingParameters.setSamplesPerPixel(sppPrev);
    camera.updateProperty("aspect", aspectPrev);

    return image;
}
} // namespace brayns
