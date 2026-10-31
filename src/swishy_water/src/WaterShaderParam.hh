/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified for Swishy (2026): adapted from gz-sim 8.9.0
 * src/systems/shader_param/ShaderParam.hh.
 */
#ifndef SWISHY_WATER_WATERSHADERPARAM_HH_
#define SWISHY_WATER_WATERSHADERPARAM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace swishy
{
  class WaterShaderParamPrivate;

  /// \brief gz::sim::systems::ShaderParam with configurable depth writes.
  ///
  /// Takes the same SDF as ShaderParam (<shader>, <param>) plus:
  ///   <depth_write>  Whether the material writes depth. Default false, so
  ///                  overlapping transparent shader visuals all show.
  class WaterShaderParam
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    public: WaterShaderParam();

    public: ~WaterShaderParam() override;

    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<WaterShaderParamPrivate> dataPtr;
  };
}

#endif
