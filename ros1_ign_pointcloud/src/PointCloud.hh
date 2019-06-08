// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS1_IGN_POINTCLOUD__POINTCLOUD_HPP_
#define ROS1_IGN_POINTCLOUD__POINTCLOUD_HPP_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ros1_ign_pointcloud
{
  // Forward declarations.
  class PointCloudPrivate;

  /// \brief Sample system that implemente the ISystemPostUpdate system
  /// plugin interface.
  class PointCloud:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: PointCloud();

    /// \brief Destructor
    public: ~PointCloud() override = default;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<PointCloudPrivate> dataPtr;
  };
}

#endif
