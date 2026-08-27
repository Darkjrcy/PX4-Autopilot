/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
*/
#ifndef GZ_SIM_SYSTEMS_NAVSAT_FAULT_INJECTION_HH_
#define GZ_SIM_SYSTEMS_NAVSAT_FAULT_INJECTION_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace gz
{
    namespace sim
    {
        // Inline bracket to help doxygen filtering.
        inline namespace GZ_SIM_VERSION_NAMESPACE {
        namespace systems
        {
            /// \class NavSatFaultInjection
            /// \brief Custom NavSat simulation system with runtime GPS fault injection.
            ///
            /// This system is based on Gazebo Sim's NavSat system, but adds the
            /// capability to inject GPS failures such as:
            ///
            /// - GPS dropout
            /// - Position bias
            /// - Increased noise
            /// - Position drift
            /// - GPS spoofing
            ///
            /// The system keeps the standard Gazebo NavSat sensor interface so that
            /// existing consumers, such as the PX4 GZBridge, can continue subscribing
            /// to the normal NavSat Gazebo Transport topic.
            ///
            /// The NavSat sensors rely on the world origin's spherical coordinates
            /// being set, for example through SDF's <spherical_coordinates> tag
            /// or the /world/world_name/set_spherical_coordinates service.
            class NavSatFaultInjection:
                public System,
                public ISystemConfigure,
                public ISystemPreUpdate,
                public ISystemPostUpdate
            {
                /// \brief Constructor
                public: explicit NavSatFaultInjection();

                // Documentation inherited
                public: void Configure(const Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    EntityComponentManager &_ecm,
                    EventManager &_eventMgr) final;

                // Documentation inherited
                public: void PreUpdate(const UpdateInfo &_info,
                                    EntityComponentManager &_ecm) final;

                // Documentation inherited
                public: void PostUpdate(const UpdateInfo &_info,
                                        const EntityComponentManager &_ecm) final;

                /// \brief Private data pointer.
                GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
            };
            }
        }
    }
}
#endif // GZ_SIM_SYSTEMS_NAVSAT_FAULT_INJECTION_HH_
