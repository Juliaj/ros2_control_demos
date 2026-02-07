// Copyright (C) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#include "ros2_control_demo_example_18/duck_mini_mujoco_system_interface.hpp"

#include <hardware_interface/version.h>
#include <rclcpp/rclcpp.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace ros2_control_demo_example_18
{

DuckMiniMujocoSystemInterface::DuckMiniMujocoSystemInterface()
: mujoco_ros2_control::MujocoSystemInterface()
{
}

#if ROS_DISTRO_HUMBLE
hardware_interface::CallbackReturn DuckMiniMujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
#else
hardware_interface::CallbackReturn DuckMiniMujocoSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
#endif
{
  // Call parent on_init first
#if ROS_DISTRO_HUMBLE
  auto result = mujoco_ros2_control::MujocoSystemInterface::on_init(info);
#else
  auto result = mujoco_ros2_control::MujocoSystemInterface::on_init(params);
#endif
  if (result != hardware_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }

  // Register contact detection using getModel()
  register_contact_detection();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DuckMiniMujocoSystemInterface::register_contact_detection()
{
  mjModel * mj_model = nullptr;
  get_model(mj_model);

  if (mj_model == nullptr)
  {
    RCLCPP_WARN(get_logger(), "MuJoCo model not available for contact detection registration");
    if (mj_model != nullptr)
    {
      mj_deleteModel(mj_model);
    }
    return;
  }

  const auto & hardware_info = get_hardware_info();
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    if (sensor.parameters.count("mujoco_type") == 0)
    {
      continue;
    }
    const auto mujoco_type = sensor.parameters.at("mujoco_type");

    if (mujoco_type != "contact")
    {
      continue;
    }

    ContactDetectionData sensor_data;
    sensor_data.name = sensor_name;
    sensor_data.contact_value = 0.0;
    sensor_data.raw_contact_value = 0.0;

    // Get body names from parameters
    // Expected parameters: body1_name, body2_name
    // These should match body names in the MuJoCo model
    if (sensor.parameters.count("body1_name") == 0 || sensor.parameters.count("body2_name") == 0)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Contact detection '"
                        << sensor_name << "' requires 'body1_name' and 'body2_name' parameters");
      continue;
    }

    sensor_data.body1_name = sensor.parameters.at("body1_name");
    sensor_data.body2_name = sensor.parameters.at("body2_name");

    // Get body IDs from MuJoCo model
    int body1_id = mj_name2id(mj_model, mjOBJ_BODY, sensor_data.body1_name.c_str());
    int body2_id = mj_name2id(mj_model, mjOBJ_BODY, sensor_data.body2_name.c_str());

    if (body1_id == -1)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Failed to find body '" << sensor_data.body1_name
                                              << "' in MuJoCo model for contact detection '"
                                              << sensor_name << "'");
      continue;
    }
    if (body2_id == -1)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Failed to find body '" << sensor_data.body2_name
                                              << "' in MuJoCo model for contact detection '"
                                              << sensor_name << "'");
      continue;
    }

    sensor_data.body1_id = body1_id;
    sensor_data.body2_id = body2_id;

    // Optional consumer mode + debounce parameters
    // - collision: raw contact (default)
    // - gait: debounced/hysteresis contact for stable foot contact
    auto get_param_or = [&](const std::string & key, const std::string & def) -> std::string
    {
      if (sensor.parameters.count(key) == 0)
      {
        return def;
      }
      return sensor.parameters.at(key);
    };

    const std::string mode_str =
      get_param_or("contact_consumer", get_param_or("contact_mode", "collision"));
    if (mode_str == "gait")
    {
      sensor_data.mode = ContactDetectionData::ConsumerMode::GAIT;
      sensor_data.filtered_contact_state = false;
      sensor_data.on_counter = 0;
      sensor_data.off_counter = 0;

      // Defaults for gait: small debounce to avoid chatter
      sensor_data.debounce_on_steps = 2;
      sensor_data.debounce_off_steps = 2;
    }
    else
    {
      sensor_data.mode = ContactDetectionData::ConsumerMode::COLLISION;
      sensor_data.debounce_on_steps = 1;
      sensor_data.debounce_off_steps = 1;
    }

    auto try_parse_int_param = [&](const std::string & key, int & out_value)
    {
      if (sensor.parameters.count(key) == 0)
      {
        return;
      }
      try
      {
        out_value = std::max(1, std::stoi(sensor.parameters.at(key)));
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN_STREAM(
          get_logger(), "Contact detection '" << sensor_name << "': invalid integer for '" << key
                                              << "': '" << sensor.parameters.at(key) << "'");
      }
    };
    try_parse_int_param("debounce_on_steps", sensor_data.debounce_on_steps);
    try_parse_int_param("debounce_off_steps", sensor_data.debounce_off_steps);

    // Optional debug: print first matching contact geom pair when a match is found.
    auto parse_bool = [&](const std::string & key, bool & out_value)
    {
      if (sensor.parameters.count(key) == 0)
      {
        return;
      }
      std::string v = sensor.parameters.at(key);
      std::transform(
        v.begin(), v.end(), v.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      out_value = (v == "1" || v == "true" || v == "yes" || v == "on");
    };
    parse_bool("debug_contact_geoms", sensor_data.debug_contact_geoms);

    contact_detection_data_.push_back(sensor_data);
    RCLCPP_INFO_STREAM(
      get_logger(), "Registered contact detection '"
                      << sensor_name << "' checking contact between '" << sensor_data.body1_name
                      << "' and '" << sensor_data.body2_name << "'");
  }

  // Clean up allocated memory
  if (mj_model != nullptr)
  {
    mj_deleteModel(mj_model);
  }
}

std::vector<hardware_interface::StateInterface>
DuckMiniMujocoSystemInterface::export_state_interfaces()
{
  // Call parent to get all standard state interfaces
  auto state_interfaces = mujoco_ros2_control::MujocoSystemInterface::export_state_interfaces();

  // Add state interfaces for contact detection
  const auto & hardware_info = get_hardware_info();
  for (auto & sensor : contact_detection_data_)
  {
    // Find sensor info in hardware_info
    for (const auto & sensor_info : hardware_info.sensors)
    {
      if (sensor_info.name != sensor.name)
      {
        continue;
      }

      // Add state interfaces for this contact detection
      for (const auto & state_if : sensor_info.state_interfaces)
      {
        if (state_if.name == "contact" || state_if.name == "value")
        {
          state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.contact_value);
        }
        else if (state_if.name == "contact_raw")
        {
          state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.raw_contact_value);
        }
      }
      break;
    }
  }

  return state_interfaces;
}

hardware_interface::return_type DuckMiniMujocoSystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Call parent read to handle standard MuJoCo operations
  auto result = mujoco_ros2_control::MujocoSystemInterface::read(time, period);
  if (result != hardware_interface::return_type::OK)
  {
    return result;
  }

  // Update contact detection data using getData() and getModel()
  update_contact_detection();

  return hardware_interface::return_type::OK;
}

void DuckMiniMujocoSystemInterface::update_contact_detection()
{
  mjModel * mj_model = nullptr;
  mjData * mj_data = nullptr;

  // Get MuJoCo model and data using inherited methods from MujocoSystemInterface
  get_model(mj_model);
  get_data(mj_data);

  if (mj_model == nullptr || mj_data == nullptr)
  {
    RCLCPP_WARN(get_logger(), "MuJoCo model or data not available for contact detection");
    return;
  }

  // Contact detection data - check collisions using MuJoCo contact detection
  // Uses MuJoCo's collision detection system to determine if two bodies are in contact
  // Note: We check mj_data_control_ first (thread-safe copy updated after mj_step()),
  // then mj_data_ with mutex lock as fallback for most up-to-date information.
  // In general, data->ncon > mj_model_->nconmax should not happen, checking it here for safety.
  struct ContactMatchInfo
  {
    int contact_index{-1};
    int geom1{-1};
    int geom2{-1};
    int geom1_body_id{-1};
    int geom2_body_id{-1};
  };

  auto raw_contact_between_bodies =
    [&](const mjData * data, int body1_id, int body2_id, ContactMatchInfo * match_info) -> bool
  {
    if (!data || !mj_model || data->ncon <= 0 || data->ncon > mj_model->nconmax)
    {
      return false;
    }

    for (int i = 0; i < data->ncon; ++i)
    {
      const mjContact & contact = data->contact[i];
      const int geom1_body_id = mj_model->geom_bodyid[contact.geom1];
      const int geom2_body_id = mj_model->geom_bodyid[contact.geom2];

      // Contact pair has no guaranteed ordering so check both directions.
      if (
        (geom1_body_id == body1_id && geom2_body_id == body2_id) ||
        (geom1_body_id == body2_id && geom2_body_id == body1_id))
      {
        if (match_info)
        {
          match_info->contact_index = i;
          match_info->geom1 = contact.geom1;
          match_info->geom2 = contact.geom2;
          match_info->geom1_body_id = geom1_body_id;
          match_info->geom2_body_id = geom2_body_id;
        }
        return true;
      }
    }

    return false;
  };

  for (auto & sensor : contact_detection_data_)
  {
    ContactMatchInfo match{};
    const bool want_debug = sensor.debug_contact_geoms;

    // Primitive: raw contact boolean (unfiltered)
    bool raw_in_contact = raw_contact_between_bodies(
      mj_data, sensor.body1_id, sensor.body2_id, want_debug ? &match : nullptr);

    sensor.raw_contact_value = raw_in_contact ? 1.0 : 0.0;

    // Debug: print first matching contact geom pair (throttled) when a match is found.
    if (want_debug && raw_in_contact && match.contact_index >= 0)
    {
      const char * geom1_name = mj_id2name(mj_model, mjOBJ_GEOM, match.geom1);
      const char * geom2_name = mj_id2name(mj_model, mjOBJ_GEOM, match.geom2);
      const char * body1_name = mj_id2name(mj_model, mjOBJ_BODY, match.geom1_body_id);
      const char * body2_name = mj_id2name(mj_model, mjOBJ_BODY, match.geom2_body_id);

      static rclcpp::Clock clock;
      RCLCPP_INFO_THROTTLE(
        get_logger(), clock, 1000,
        "Contact debug '%s': matched contact[%d] geom1='%s'(id=%d, body_id=%d '%s') "
        "geom2='%s'(id=%d, body_id=%d "
        "'%s') | sensor bodies: '%s'(id=%d) vs '%s'(id=%d)",
        sensor.name.c_str(), match.contact_index, geom1_name ? geom1_name : "unknown", match.geom1,
        match.geom1_body_id, body1_name ? body1_name : "unknown",
        geom2_name ? geom2_name : "unknown", match.geom2, match.geom2_body_id,
        body2_name ? body2_name : "unknown", sensor.body1_name.c_str(), sensor.body1_id,
        sensor.body2_name.c_str(), sensor.body2_id);
    }

    // Consumer: collision (raw) vs gait (debounced/hysteresis)
    if (sensor.mode == ContactDetectionData::ConsumerMode::COLLISION)
    {
      sensor.filtered_contact_state = raw_in_contact;
    }
    else
    {
      // Debounce/hysteresis in steps:
      // - to turn ON: require N consecutive raw contacts
      // - to turn OFF: require M consecutive raw no-contacts
      if (raw_in_contact)
      {
        sensor.on_counter++;
        sensor.off_counter = 0;
        if (!sensor.filtered_contact_state && sensor.on_counter >= sensor.debounce_on_steps)
        {
          sensor.filtered_contact_state = true;
        }
      }
      else
      {
        sensor.off_counter++;
        sensor.on_counter = 0;
        if (sensor.filtered_contact_state && sensor.off_counter >= sensor.debounce_off_steps)
        {
          sensor.filtered_contact_state = false;
        }
      }
    }

    sensor.contact_value = sensor.filtered_contact_state ? 1.0 : 0.0;
  }

  // Clean up allocated memory
  if (mj_model != nullptr)
  {
    mj_deleteModel(mj_model);
  }
  if (mj_data != nullptr)
  {
    mj_deleteData(mj_data);
  }
}

}  // namespace ros2_control_demo_example_18

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_18::DuckMiniMujocoSystemInterface, hardware_interface::SystemInterface)
