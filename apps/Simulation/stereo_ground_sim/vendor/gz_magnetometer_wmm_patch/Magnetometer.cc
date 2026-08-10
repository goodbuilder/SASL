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

#include "Magnetometer.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/common/Profiler.hh>

#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/MagnetometerSensor.hh>

#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/** set this always to the sampling in degrees for the table below */
static constexpr const float SAMPLING_RES     =  10.0f;
// Latitude range widened from the stock -60..60 to -90..90 to match the
// PX4-sourced table below (see px4_wmm_geo_magnetic_tables.hh).
static constexpr const float SAMPLING_MIN_LAT = -90.0f;
static constexpr const float SAMPLING_MAX_LAT =  90.0f;
static constexpr const float SAMPLING_MIN_LON = -180.0f;
static constexpr const float SAMPLING_MAX_LON = 180.0f;

/// Returns scalar value constrained by (min_val, max_val)
template<typename Scalar>
static inline constexpr const Scalar &constrain(
  const Scalar &val, const Scalar &min_val, const Scalar &max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

// declination_table, inclination_table, totalintensity_table (all int16_t[19][37],
// scaled fixed-point) plus their WMM_*_SCALE_TO_* constants come from PX4-Autopilot's
// own WMM-2020 snapshot -- see px4_wmm_geo_magnetic_tables.hh for provenance. This
// replaces gz-sim's stock -60..60 lat, plain-degree/centi-gauss table so the simulated
// magnetometer signal and PX4's independently-computed EKF2_MAG_DECL reference the
// same declination/inclination/strength data for a given lat/lon.
#include "px4_wmm_geo_magnetic_tables.hh"

/// \brief Private Magnetometer data class.
class gz::sim::systems::MagnetometerPrivate
{
  /// \brief A map of magnetometer entity to its sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::MagnetometerSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief True if the magnetic field is reported in gauss rather than tesla.
  public: bool useUnitsGauss = true;

  /// \brief True if the magnetic field earth frame is NED rather than ENU.
  public: bool useEarthFrameNED = true;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _magnetometer Magnetometer component.
  /// \param[in] _worldField MagneticField component.
  /// \param[in] _parent Parent entity component.
  public: void AddMagnetometer(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Magnetometer *_magnetometer,
    const components::MagneticField *_worldField,
    const components::ParentEntity *_parent);

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove magnetometer sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveMagnetometerEntities(const EntityComponentManager &_ecm);

  unsigned get_lookup_table_index(float &val, float min, float max)
  {
    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    // limit to table bounds - required for maxima even when table spans full
    // globe range
    // limit to (table bounds - 1) because bilinear interpolation requires
    // checking (index + 1)
    val = constrain(val, min, max - SAMPLING_RES);

    return static_cast<unsigned>((-(min) + val) / SAMPLING_RES);
  }

  float get_table_data(float lat, float lon, const int16_t table[19][37])
  {
    /*
     * If the values exceed valid ranges, return zero as default
     * as we have no way of knowing what the closest real value
     * would be.
     */
    if (lat < -90.0f || lat > 90.0f ||
        lon < -180.0f || lon > 180.0f) {
      return 0.0f;
    }

    /* round down to nearest sampling resolution */
    float min_lat = static_cast<int>(lat / SAMPLING_RES) * SAMPLING_RES;
    float min_lon = static_cast<int>(lon / SAMPLING_RES) * SAMPLING_RES;

    /* find index of nearest low sampling point */
    unsigned min_lat_index =
      get_lookup_table_index(min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
    unsigned min_lon_index =
      get_lookup_table_index(min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

    const float data_sw = table[min_lat_index][min_lon_index];
    const float data_se = table[min_lat_index][min_lon_index + 1];
    const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
    const float data_nw = table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */
    const float lat_scale = constrain(
      (lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
    const float lon_scale = constrain(
      (lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);

    const float data_min = lon_scale * (data_se - data_sw) + data_sw;
    const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

    return lat_scale * (data_max - data_min) + data_min;
  }

  // return magnetic declination in degrees
  float get_mag_declination(float lat, float lon)
  {
    return get_table_data(lat, lon, declination_table) *
      WMM_DECLINATION_SCALE_TO_DEGREES;
  }

  // return magnetic field inclination in degrees
  float get_mag_inclination(float lat, float lon)
  {
    return get_table_data(lat, lon, inclination_table) *
      WMM_INCLINATION_SCALE_TO_DEGREES;
  }

  // return magnetic field strength in centi-Gauss (callers apply the
  // existing 0.01f-to-gauss / 1e-4f-to-tesla conversions unchanged; the
  // PX4-sourced table is in nanoTesla, so convert nT -> centi-Gauss here:
  // 1 nT = 1e-9 T = 1e-5 G = 1e-3 cG).
  float get_mag_strength(float lat, float lon)
  {
    const float nano_tesla =
      get_table_data(lat, lon, totalintensity_table) *
      WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA;
    return nano_tesla * 1.0e-3f;
  }
};

//////////////////////////////////////////////////
Magnetometer::Magnetometer() : System(), dataPtr(
    std::make_unique<MagnetometerPrivate>())
{
}

//////////////////////////////////////////////////
Magnetometer::~Magnetometer() = default;

//////////////////////////////////////////////////
void Magnetometer::Configure(const Entity &/*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &/*_ecm*/,
                           EventManager &/*_eventMgr*/)
{
    if (_sdf->HasElement("use_units_gauss"))
    {
      this->dataPtr->useUnitsGauss = _sdf->Get<bool>("use_units_gauss");
    }
    gzdbg << "Magnetometer: using param [use_units_gauss: "
          << this->dataPtr->useUnitsGauss << "]."
          << std::endl;

    if (_sdf->HasElement("use_earth_frame_ned"))
    {
      this->dataPtr->useEarthFrameNED = _sdf->Get<bool>("use_earth_frame_ned");
    }
    gzdbg << "Magnetometer: using param [use_earth_frame_ned: "
          << this->dataPtr->useEarthFrameNED << "]."
          << std::endl;
}

//////////////////////////////////////////////////
void Magnetometer::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Magnetometer::PreUpdate");

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
void Magnetometer::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Magnetometer::PostUpdate");

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
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the MagnetometerPrivate::Update function
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

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::AddMagnetometer(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Magnetometer *_magnetometer,
  const components::MagneticField *_worldField,
  const components::ParentEntity *_parent)
{
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _magnetometer->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/magnetometer";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::MagnetometerSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::MagnetometerSensor>(data);
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

  sensor->SetWorldMagneticField(_worldField->Data());

  // Get initial pose of sensor and set the reference z pos
  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
  math::Pose3d p = worldPose(_entity, _ecm);
  sensor->SetWorldPose(p);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::CreateMagnetometerEntities");
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world magnetic field (defined in world frame)
  auto worldField = _ecm.Component<components::MagneticField>(worldEntity);
  if (nullptr == worldField)
  {
    gzerr << "World missing magnetic field." << std::endl;
    return;
  }

  if (!this->initialized)
  {
    // Create magnetometers
    _ecm.Each<components::Magnetometer, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Magnetometer *_magnetometer,
          const components::ParentEntity *_parent)->bool
        {
          this->AddMagnetometer(_ecm, _entity, _magnetometer, worldField,
              _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create magnetometers
    _ecm.EachNew<components::Magnetometer, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Magnetometer *_magnetometer,
          const components::ParentEntity *_parent)->bool
        {
          this->AddMagnetometer(_ecm, _entity, _magnetometer, worldField,
              _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void MagnetometerPrivate::Update(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::Update");
  _ecm.Each<components::Magnetometer,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::Magnetometer * /*_magnetometer*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Get the magnetometer physical position
          const math::Pose3d &magnetometerWorldPose = _worldPose->Data();
          it->second->SetWorldPose(magnetometerWorldPose);

          // Position
          auto latLonEle = sphericalCoordinates(_entity, _ecm);
          if (!latLonEle)
          {
            gzwarn << "Failed to update Magnetometer sensor enity [" << _entity
                    << "]. Spherical coordinates not set." << std::endl;
            return true;
          }

          auto lat_rad = GZ_DTOR(latLonEle.value().X());
          auto lon_rad = GZ_DTOR(latLonEle.value().Y());

          // Magnetic declination and inclination (radians)
          float declination_rad =
            get_mag_declination(
              lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI) * GZ_PI / 180;
          float inclination_rad =
            get_mag_inclination(
              lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI) * GZ_PI / 180;

          // Magnetic strength in gauss (10^5 nano tesla = 10^-2 centi gauss)
          float strength_ga =
            0.01f *
            get_mag_strength(lat_rad * 180 / GZ_PI, lon_rad * 180 / GZ_PI);

          // Magnetic intensity measured in telsa
          float strength_tesla = 1.0E-4 * strength_ga;

          // Magnetic field components are calculated in world NED frame using:
          // http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
          float H = cosf(inclination_rad);
          H *= this->useUnitsGauss ? strength_ga : strength_tesla;
          float Z_ned = tanf(inclination_rad) * H;
          float X_ned = H * cosf(declination_rad);
          float Y_ned = H * sinf(declination_rad);

          float X = X_ned;
          float Y = Y_ned;
          float Z = Z_ned;
          if (!this->useEarthFrameNED)
          {
            // Use ENU convention for earth frame.
            X = Y_ned;
            Y = X_ned;
            Z = -1.0 * Z_ned;
          }

          math::Vector3d magnetic_field_I(X, Y, Z);
          it->second->SetWorldMagneticField(magnetic_field_I);
        }
        else
        {
          gzerr << "Failed to update magnetometer: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::RemoveMagnetometerEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MagnetometerPrivate::RemoveMagnetometerEntities");
  _ecm.EachRemoved<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing magnetometer sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemConfigure,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Magnetometer,
                          "gz::sim::systems::Magnetometer")
