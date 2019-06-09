/* Copyright (c) 2015-2018, EPFL/Blue Brain Project
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille.favreau@epfl.ch>
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

#include "Scene.h"

#include <brayns/common/Transformation.h>
#include <brayns/common/log.h>
#include <brayns/common/scene/ClipPlane.h>
#include <brayns/common/utils/utils.h>
#include <brayns/engine/Material.h>
#include <brayns/engine/Model.h>

#include <brayns/parameters/GeometryParameters.h>

#include <fstream>

#include <boost/filesystem.hpp>

namespace
{
template <typename T, typename U = T> // U seems to be needed when getID is a
                                      // member function of a base of T.
std::shared_ptr<T> _find(const std::vector<std::shared_ptr<T>>& list,
                         const size_t id,
                         size_t (U::*getID)() const = &T::getID)
{
    auto i = std::find_if(list.begin(), list.end(), [id, getID](auto x) {
        return id == ((*x).*getID)();
    });
    return i == list.end() ? std::shared_ptr<T>{} : *i;
}

template <typename T, typename U = T>
std::shared_ptr<T> _remove(std::vector<std::shared_ptr<T>>& list,
                           const size_t id,
                           size_t (U::*getID)() const = &T::getID)
{
    auto i = std::find_if(list.begin(), list.end(), [id, getID](auto x) {
        return id == ((*x).*getID)();
    });
    if (i == list.end())
        return std::shared_ptr<T>{};
    auto result = *i;
    list.erase(i);
    return result;
}
} // namespace

namespace brayns
{
Scene::Scene(AnimationParameters& animationParameters,
             GeometryParameters& geometryParameters,
             VolumeParameters& volumeParameters)
    : _animationParameters(animationParameters)
    , _geometryParameters(geometryParameters)
    , _volumeParameters(volumeParameters)
{
}

void Scene::copyFrom(const Scene& rhs)
{
    if (this == &rhs)
        return;

    {
        std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
        std::shared_lock<std::shared_timed_mutex> rhsLock(rhs._modelMutex);

        _modelDescriptors.clear();
        _modelDescriptors.reserve(rhs._modelDescriptors.size());
        for (const auto& modelDesc : rhs._modelDescriptors)
            _modelDescriptors.push_back(modelDesc->clone(createModel()));
    }
    _computeBounds();

    *_backgroundMaterial = *rhs._backgroundMaterial;
    _backgroundMaterial->markModified();

    _lightManager = rhs._lightManager;
    _clipPlanes = rhs._clipPlanes;

    markModified();
}

void Scene::commit()
{
}

size_t Scene::getSizeInBytes() const
{
    auto lock = acquireReadAccess();
    size_t sizeInBytes = 0;
    for (auto modelDescriptor : _modelDescriptors)
        sizeInBytes += modelDescriptor->getModel().getSizeInBytes();
    return sizeInBytes;
}

size_t Scene::getNumModels() const
{
    auto lock = acquireReadAccess();
    return _modelDescriptors.size();
}

size_t Scene::addModel(ModelDescriptorPtr modelDescriptor)
{
    auto& model = modelDescriptor->getModel();
    if (model.empty())
        throw std::runtime_error("Empty models not supported.");

    const auto defaultBVHFlags = _geometryParameters.getDefaultBVHFlags();

    model.setBVHFlags(defaultBVHFlags);
    model.buildBoundingBox();

    // Since models can be added concurrently we check if that is supported
    if (supportsConcurrentSceneUpdates())
        model.commitGeometry();

    {
        std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
        modelDescriptor->setModelID(_modelID++);
        _modelDescriptors.push_back(modelDescriptor);

        // add default instance of this model to render something
        if (modelDescriptor->getInstances().empty())
            modelDescriptor->addInstance(
                {true, true, modelDescriptor->getTransformation()});
    }

    markModified();

    return modelDescriptor->getModelID();
}

bool Scene::removeModel(const size_t id)
{
    ModelDescriptorPtr model = nullptr;

    if (supportsConcurrentSceneUpdates())
    {
        {
            std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
            model =
                _remove(_modelDescriptors, id, &ModelDescriptor::getModelID);
        }
        if (model)
            model->callOnRemoved();
    }
    else
    {
        {
            std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
            model = _find(_modelDescriptors, id, &ModelDescriptor::getModelID);
        }
        if (model)
            model->markForRemoval();
    }

    if (model)
    {
        markModified();
        return true;
    }

    return false;
}

ModelDescriptorPtr Scene::getModel(const size_t id) const
{
    auto lock = acquireReadAccess();
    return _find(_modelDescriptors, id, &ModelDescriptor::getModelID);
}

bool Scene::empty() const
{
    auto lock = acquireReadAccess();
    for (auto modelDescriptor : _modelDescriptors)
        if (!modelDescriptor->getModel().empty())
            return false;
    return true;
}

size_t Scene::addClipPlane(const Plane& plane)
{
    auto clipPlane = std::make_shared<ClipPlane>(plane);
    clipPlane->onModified([&](const BaseObject&) { markModified(); });
    _clipPlanes.emplace_back(std::move(clipPlane));
    markModified();
    return _clipPlanes.back()->getID();
}

ClipPlanePtr Scene::getClipPlane(const size_t id) const
{
    return _find(_clipPlanes, id);
}

void Scene::removeClipPlane(const size_t id)
{
    if (_remove(_clipPlanes, id))
        markModified();
}

ModelDescriptorPtr Scene::loadModel(Blob&& blob, const ModelParams& params,
                                    LoaderProgress cb)
{
    const auto& loader =
        _loaderRegistry.getSuitableLoader("", blob.type,
                                          params.getLoaderName());

    // HACK: Add loader name in properties for archive loader
    auto propCopy = params.getLoaderProperties();
    propCopy.setProperty({"loaderName", params.getLoaderName()});
    auto modelDescriptor = loader.importFromBlob(std::move(blob), cb, propCopy);
    if (!modelDescriptor)
        throw std::runtime_error("No model returned by loader");
    *modelDescriptor = params;
    addModel(modelDescriptor);
    return modelDescriptor;
}

ModelDescriptorPtr Scene::loadModel(const std::string& path,
                                    const ModelParams& params,
                                    LoaderProgress cb)
{
    const auto& loader =
        _loaderRegistry.getSuitableLoader(path, "", params.getLoaderName());
    // HACK: Add loader name in properties for archive loader
    auto propCopy = params.getLoaderProperties();
    propCopy.setProperty({"loaderName", params.getLoaderName()});
    auto modelDescriptor = loader.importFromFile(path, cb, propCopy);
    if (!modelDescriptor)
        throw std::runtime_error("No model returned by loader");
    *modelDescriptor = params;
    addModel(modelDescriptor);
    return modelDescriptor;
}

void Scene::visitModels(const std::function<void(Model&)>& functor)
{
    std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
    for (const auto& modelDescriptor : _modelDescriptors)
        functor(modelDescriptor->getModel());
}

void Scene::buildDefault()
{
    BRAYNS_INFO << "Building default Cornell Box scene" << std::endl;

    // const Vector3f WHITE = {1.f, 1.f, 1.f};
    const Vector3f GREEN = {1.f, 1.0f, 1.f};

// SDF types

#if 0
    auto pill = createSDFConePill({0, 0, 0}, {1.5, 0, 0}, 0.4, 0.1);
    auto pillSigmoid =
        createSDFConePillSigmoid({0, 2, 0}, {1.5, 2, 0}, 0.4, 0.1);
    auto sphere = createSDFSphere({1, 4, 0}, 0.5);

    model->addSDFGeometry(0, pill, {});
    model->addSDFGeometry(0, pillSigmoid, {});
    model->addSDFGeometry(0, sphere, {});
#endif

// Blending ON
#if 1
    {
        auto model = createModel();

        const auto materialId = 0;
        auto material =
            model->createMaterial(materialId,
                                  "wall_" + std::to_string(materialId));
        material->setDiffuseColor(GREEN);
        material->setSpecularColor(GREEN);
        material->setSpecularExponent(10.f);
        material->setOpacity(1.f);

        auto sphere0 = createSDFSphere({1, 4, 0}, 0.2);
        auto sphere1 = createSDFSphere({1.3, 4, 0}, 0.2);

        model->addSDFGeometry(0, sphere0, {1});
        model->addSDFGeometry(0, sphere1, {0});

        addModel(
            std::make_shared<ModelDescriptor>(std::move(model), "Blending on"));
    }
#endif

// Blending OFF
#if 1
    {
        auto model = createModel();

        const auto materialId = 0;
        auto material =
            model->createMaterial(materialId,
                                  "wall_" + std::to_string(materialId));
        material->setDiffuseColor(GREEN);
        material->setSpecularColor(GREEN);
        material->setSpecularExponent(10.f);
        material->setOpacity(1.f);

        auto sphere0 = createSDFSphere({1, 4, 0}, 0.2);
        auto sphere1 = createSDFSphere({1.3, 4, 0}, 0.2);

        model->addSDFGeometry(0, sphere0, {});
        model->addSDFGeometry(0, sphere1, {});

        addModel(std::make_shared<ModelDescriptor>(std::move(model),
                                                   "Blending off"));
    }
#endif
}

void Scene::setMaterialsColorMap(MaterialsColorMap colorMap)
{
    {
        auto lock = acquireReadAccess();
        for (auto modelDescriptors : _modelDescriptors)
            modelDescriptors->getModel().setMaterialsColorMap(colorMap);
    }
    markModified();
}

bool Scene::setEnvironmentMap(const std::string& envMap)
{
    bool success = true;
    if (envMap.empty())
        _backgroundMaterial->clearTextures();
    else
    {
        try
        {
            _backgroundMaterial->setTexture(envMap, TextureType::diffuse);
        }
        catch (const std::runtime_error& e)
        {
            BRAYNS_DEBUG << "Cannot load environment map: " << e.what()
                         << std::endl;
            _backgroundMaterial->clearTextures();
            success = false;
        }

        _loadIBLMaps(envMap);
    }

    _updateValue(_environmentMap, success ? envMap : "");
    if (_backgroundMaterial->isModified())
        markModified();
    return success;
}

bool Scene::hasEnvironmentMap() const
{
    return !_environmentMap.empty();
}

void Scene::_computeBounds()
{
    std::unique_lock<std::shared_timed_mutex> lock(_modelMutex);
    _bounds.reset();
    for (auto modelDescriptor : _modelDescriptors)
    {
        modelDescriptor->computeBounds();
        _bounds.merge(modelDescriptor->getBounds());
    }

    if (_bounds.isEmpty())
        // If no model is enabled. return empty bounding box
        _bounds.merge({0, 0, 0});
}

void Scene::_loadIBLMaps(const std::string& envMap)
{
    try
    {
        auto tex = _backgroundMaterial->getTexture(TextureType::diffuse);

        namespace fs = boost::filesystem;
        const auto path = fs::path(envMap).parent_path();
        const auto basename = (path / fs::basename(envMap)).string();

        const std::string irradianceMap = basename + IRRADIANCE_MAP + ".hdr";
        const std::string radianceMap = basename + RADIANCE_MAP + ".hdr";
        const std::string brdfLUT = basename + BRDF_LUT + ".hdr";

        if (fs::exists(irradianceMap) && fs::exists(radianceMap) &&
            fs::exists(brdfLUT))
        {
            _backgroundMaterial->setTexture(irradianceMap,
                                            TextureType::irradiance);
            _backgroundMaterial->setTexture(radianceMap, TextureType::radiance);
            _backgroundMaterial->setTexture(brdfLUT, TextureType::brdf_lut);
        }
    }
    catch (...)
    {
    }
}
} // namespace brayns
