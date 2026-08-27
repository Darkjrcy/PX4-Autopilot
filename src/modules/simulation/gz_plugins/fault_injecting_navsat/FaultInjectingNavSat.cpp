/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "FaultInjectingNavSat.hpp"

#include <gz/msgs/navsat.pb.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <sdf/Sensor.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/NavSatSensor.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/int32.pb.h>

#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/NavSat.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private NavSatFaultInjection data class.
class gz::sim::systems::NavSatFaultInjection::Implementation
{
    /// \brief A map of NavSatFaultInjection entity to its sensor
    public: std::unordered_map<Entity,
        std::unique_ptr<sensors::NavSatSensor>> entitySensorMap;

    /// \brief gz-sensors sensor factory for creating sensors
    public: sensors::SensorFactory sensorFactory;

    /// \brief Keep list of sensors that were created during the previous
    /// `PostUpdate`, so that components can be created during the next
    /// `PreUpdate`.
    public: std::unordered_set<Entity> newSensors;

    /// When the system is just loaded, we loop over all entities to create
    /// sensors. After this initialization, we only check inserted entities.
    public: bool initialized = false;

    /// \brief Create sensors in gz-sensors
    /// \param[in] _ecm Immutable reference to ECM.
    public: void CreateSensors(const EntityComponentManager &_ecm);

    /// \brief Update sensor data based on physics data
    /// \param[in] _ecm Immutable reference to ECM.
    public: void Update(const EntityComponentManager &_ecm);

    /// \brief Advance time-dependent fault state by one simulation step.
    /// \param[in] _dtSeconds Simulation time elapsed during the step.
    public: void AdvanceFaultState(const double _dtSeconds);

    /// \brief Remove sensors if their entities have been removed from simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    public: void RemoveSensors(const EntityComponentManager &_ecm);

    /// \brief Create sensor
    /// \param[in] _ecm Immutable reference to ECM.
    /// \param[in] _entity Sensor entity
    /// \param[in] _navsat NavSat component.
    /// \param[in] _parent Parent entity component.
    public: void AddSensor(
        const EntityComponentManager &_ecm,
        const Entity _entity,
        const components::NavSat *_navSat,
        const components::ParentEntity *_parent);


    public:
        // Fault mode number:
        enum class FaultMode : int
        {
            NORMAL = 0,
            DROPOUT = 1,
            BIAS = 2,
            NOISE = 3,
            DRIFT = 4,
        };

        // Current GPS mode:
        std::atomic<FaultMode> currentFaultMode{FaultMode::NORMAL};

        // Gazebo transport node to publish the GPS fault mode:
        gz::transport::Node node;

        // Service names
        std::string enableService;
        std::string faultModeService;
        // Position bias in case of BIAS (m):
        double biasNorth{0.0};
        double biasEast{0.0};
        double biasAltitude{0.0};

        // Gaussian noise in case of Noise (m):
        double horizontalNoiseStdDev{0.0};
        double verticalNoiseStdDev{0.0};

        // Drift rate (m/s):
        double driftNorth{0.0};
        double driftEast{0.0};

        // Accumulated drift offset (m). Only accessed by the simulation thread.
        double accumulatedDriftNorth{0.0};
        double accumulatedDriftEast{0.0};
        FaultMode appliedFaultMode{FaultMode::NORMAL};

        // Persistent generator avoids reseeding for every NavSat sample.
        std::mt19937 randomGenerator{std::random_device{}()};

        // Callback functions for enabling GPS and selecting its fault mode.
        public: bool SetGpsEnabled(const gz::msgs::Boolean &_request,
            gz::msgs::Boolean &_response);
        public: bool SetFaultMode(const gz::msgs::Int32 &_request,
            gz::msgs::Boolean &_response);

};



//////////////////////////////////////////////////
NavSatFaultInjection::NavSatFaultInjection()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}



//////////////////////////////////////////////////
void NavSatFaultInjection::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
    // Check if the SDF is configured for specific service names.
    std::string serviceName = "/gps_fault/set_enabled";
    if (_sdf && _sdf->HasElement("enable_service")) {
        serviceName = _sdf->Get<std::string>("enable_service");
    }

    std::string modeService = "/gps_fault/set_mode";
    if (_sdf && _sdf->HasElement("fault_mode_service")) {
        modeService = _sdf->Get<std::string>("fault_mode_service");
    }

    // Get the noise and bias from the SDF.
    if (_sdf && _sdf->HasElement("bias_north")) {
        this->dataPtr->biasNorth = _sdf->Get<double>("bias_north");
    }
    if (_sdf && _sdf->HasElement("bias_east")) {
        this->dataPtr->biasEast = _sdf->Get<double>("bias_east");
    }
    if (_sdf && _sdf->HasElement("bias_altitude")) {
        this->dataPtr->biasAltitude = _sdf->Get<double>("bias_altitude");
    }
    if (_sdf && _sdf->HasElement("horizontal_noise_stddev")) {
        this->dataPtr->horizontalNoiseStdDev = _sdf->Get<double>("horizontal_noise_stddev");
    }
    if (_sdf && _sdf->HasElement("vertical_noise_stddev")) {
        this->dataPtr->verticalNoiseStdDev = _sdf->Get<double>("vertical_noise_stddev");
    }

    // Get the drift rates from the SDF.
    if (_sdf && _sdf->HasElement("drift_north")) {
        this->dataPtr->driftNorth = _sdf->Get<double>("drift_north");
    }
    if (_sdf && _sdf->HasElement("drift_east")) {
        this->dataPtr->driftEast = _sdf->Get<double>("drift_east");
    }

    this->dataPtr->enableService = serviceName;
    this->dataPtr->faultModeService = modeService;

    // Node to see if the GPS service is enabled:
    this->dataPtr->node.Advertise(serviceName,
        &NavSatFaultInjection::Implementation::SetGpsEnabled,
        this->dataPtr.get());
    // Node to change the service mode of the GPS:
    this->dataPtr->node.Advertise(modeService,
        &Implementation::SetFaultMode,
        this->dataPtr.get());
}



//////////////////////////////////////////////////
// Subscriber that turns out the GPS sensor if the service is called with false, and turns it on if true.
bool NavSatFaultInjection::Implementation::SetGpsEnabled(
    const gz::msgs::Boolean &_req, gz::msgs::Boolean &_rep)
{
    this->currentFaultMode.store(_req.data() ? FaultMode::NORMAL : FaultMode::DROPOUT);
    _rep.set_data(true);
    return true;
}


// Function used to get teh failure mode of the GPS sensor:
bool NavSatFaultInjection::Implementation::SetFaultMode(
    const gz::msgs::Int32 &_req, gz::msgs::Boolean &_rep)
{
    const int mode = _req.data();

    if (mode < static_cast<int>(Implementation::FaultMode::NORMAL) ||
        mode > static_cast<int>(Implementation::FaultMode::DRIFT)) {
        _rep.set_data(false);
        return true;
    }

    this->currentFaultMode.store(static_cast<FaultMode>(mode));
    _rep.set_data(true);
    return true;
}



//////////////////////////////////////////////////
void NavSatFaultInjection::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSat::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}



//////////////////////////////////////////////////
void NavSatFaultInjection::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSat::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    const double dtSeconds = std::chrono::duration<double>(_info.dt).count();
    this->dataPtr->AdvanceFaultState(dtSeconds);

    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the NavSat::Implementation::Update function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    const auto faultMode = this->dataPtr->currentFaultMode.load();
    this->dataPtr->Update(_ecm);

    // A dropout must suppress publication, not just position updates. Otherwise
    // Sensor::Update would republish the last valid sample.
    if (faultMode != Implementation::FaultMode::DROPOUT)
    {
      for (auto &it : this->dataPtr->entitySensorMap)
      {
        it.second.get()->sensors::Sensor::Update(_info.simTime, false);
      }
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

//////////////////////////////////////////////////
void NavSatFaultInjection::Implementation::AddSensor(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::NavSat *_navsat,
  const components::ParentEntity *_parent)
{
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _navsat->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/navsat";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::NavSatSensor> sensor =
      this->sensorFactory.CreateSensor<sensors::NavSatSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void NavSatFaultInjection::Implementation::CreateSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSat::CreateSensors");
  if (!this->initialized)
  {
    _ecm.Each<components::NavSat, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::NavSat *_navSat,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _navSat, _parent);
          return true;
        });
      this->initialized = true;
  }
  else
  {
    _ecm.EachNew<components::NavSat, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::NavSat *_navSat,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _navSat, _parent);
          return true;
      });
  }
}

//////////////////////////////////////////////////
void NavSatFaultInjection::Implementation::AdvanceFaultState(
    const double _dtSeconds)
{
  const FaultMode faultMode = this->currentFaultMode.load();

  // Start a new drift at zero whenever drift mode is selected.
  if (faultMode != this->appliedFaultMode)
  {
    if (faultMode == FaultMode::DRIFT)
    {
      this->accumulatedDriftNorth = 0.0;
      this->accumulatedDriftEast = 0.0;
    }
    this->appliedFaultMode = faultMode;
  }

  if (faultMode == FaultMode::DRIFT && _dtSeconds > 0.0)
  {
    this->accumulatedDriftNorth += this->driftNorth * _dtSeconds;
    this->accumulatedDriftEast += this->driftEast * _dtSeconds;
  }
}

//////////////////////////////////////////////////
void NavSatFaultInjection::Implementation::Update(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSat::Update");

  const FaultMode faultMode = this->currentFaultMode.load();

  _ecm.Each<components::NavSat, components::WorldLinearVelocity>(
    [&](const Entity &_entity,
        const components::NavSat * /*_navsat*/,
        const components::WorldLinearVelocity *_worldLinearVel
        )->bool
      {
        auto it = this->entitySensorMap.find(_entity);

        if (it == this->entitySensorMap.end())
        {
          gzerr << "Failed to update NavSat sensor entity [" << _entity
                 << "]. Entity not found." << std::endl;
          return true;
        }

        // Position
        auto latLonEle = sphericalCoordinates(_entity, _ecm);
        if (!latLonEle)
        {
          gzwarn << "Failed to update NavSat sensor enity [" << _entity
                  << "]. Spherical coordinates not set." << std::endl;
          return true;
        }

        // Set sensor position and add failure modes:
        // Real position  in radians and altitude:
        double latitudeRad = GZ_DTOR(latLonEle.value().X());
        double longitudeRad = GZ_DTOR(latLonEle.value().Y());
        double altitude = latLonEle.value().Z();

        // Define the adding offset for each case:
        double offsetNorth = 0.0;
        double offsetEast = 0.0;
        double offsetAltitude = 0.0;

        // Earth radius in meters:
        constexpr double earthRadius = 6378137.0;

        if (faultMode == FaultMode::DROPOUT) {
            return true; // Do not update the sensor, simulating a dropout
        } else {
            // Add the different biases:
            if (faultMode == FaultMode::BIAS) {
                offsetNorth = this->biasNorth / earthRadius;
                offsetEast = this->biasEast / (earthRadius * std::cos(latitudeRad));
                offsetAltitude = this->biasAltitude;
            } else if (faultMode == FaultMode::NOISE) {
                // Add Gaussian noise to the position
                std::normal_distribution<> d(0, 1);

                offsetNorth = d(this->randomGenerator) *
                    this->horizontalNoiseStdDev / earthRadius;
                offsetEast = d(this->randomGenerator) *
                    this->horizontalNoiseStdDev /
                    (earthRadius * std::cos(latitudeRad));
                offsetAltitude = d(this->randomGenerator) *
                    this->verticalNoiseStdDev;
            } else if (faultMode == FaultMode::DRIFT) {
                offsetNorth = this->accumulatedDriftNorth / earthRadius;
                offsetEast = this->accumulatedDriftEast /
                    (earthRadius * std::cos(latitudeRad));
            }
        }


        it->second->SetLatitude(latitudeRad + offsetNorth);
        it->second->SetLongitude(longitudeRad + offsetEast);
        it->second->SetAltitude(altitude + offsetAltitude);

        // Velocity in ENU frame
        it->second->SetVelocity(_worldLinearVel->Data());

        return true;
      });
}

//////////////////////////////////////////////////
void NavSatFaultInjection::Implementation::RemoveSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("NavSat::RemoveSensors");
  _ecm.EachRemoved<components::NavSat>(
    [&](const Entity &_entity,
        const components::NavSat *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing NavSat sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(NavSatFaultInjection, System,
  NavSatFaultInjection::ISystemConfigure,
  NavSatFaultInjection::ISystemPreUpdate,
  NavSatFaultInjection::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  NavSatFaultInjection,
  "gz::sim::systems::NavSatFaultInjection")
