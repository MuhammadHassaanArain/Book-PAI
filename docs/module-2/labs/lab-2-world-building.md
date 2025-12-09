# Lab 2: Build a Custom Gazebo World

## Objective

In this lab, you will learn to create custom Gazebo worlds with terrain, objects, and lighting. You will build a complete environment from scratch and spawn objects dynamically using ROS 2.

## Prerequisites

- Basic understanding of Gazebo and ROS 2
- Completed Lab 1: Configure Gazebo Physics
- Working ROS 2 Humble/Iron installation
- Gazebo Fortress/Harmonic installed

## Lab Tasks

### Task 1: Create a Basic World File

Create a new world file with basic elements:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_lab_world">
    <!-- Physics configuration -->
    <physics type="bullet">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Environment elements -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Create a simple building -->
    <model name="building_1">
      <pose>5 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>4 4 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>4 4 3</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a table -->
    <model name="table">
      <pose>-3 2 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1.5 0.8 0.75</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.5 0.8 0.75</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Task 2: Add Terrain Elements

Add terrain elements to your world:

```xml
<!-- Add a simple terrain model -->
<model name="simple_terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://terrain_models/meshes/simple_terrain.dae</uri>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://terrain_models/meshes/simple_terrain.dae</uri>
        </mesh>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Dirt</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

### Task 3: Implement Dynamic Object Spawning

Create a ROS 2 node to spawn objects dynamically:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time

class DynamicObjectSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_object_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /spawn_entity...')

        self.spawn_objects()

    def spawn_objects(self):
        # Define objects to spawn
        objects = [
            {
                'name': 'dynamic_box_1',
                'xml': self.create_box_sdf('dynamic_box_1'),
                'x': 2.0, 'y': 2.0, 'z': 0.5
            },
            {
                'name': 'dynamic_sphere_1',
                'xml': self.create_sphere_sdf('dynamic_sphere_1'),
                'x': -2.0, 'y': -2.0, 'z': 0.5
            }
        ]

        for obj in objects:
            self.spawn_single_object(obj['name'], obj['xml'], obj['x'], obj['y'], obj['z'])
            time.sleep(1)  # Wait between spawns

    def create_box_sdf(self, name):
        return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.083</iyy>
          <iyz>0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.5 0.5 0.5</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.5 0.5 0.5</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>1.0 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def create_sphere_sdf(self, name):
        return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.025</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.025</iyy>
          <iyz>0</iyz>
          <izz>0.025</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>0.25</radius></sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>0.25</radius></sphere>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.3 1.0 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def spawn_single_object(self, name, xml, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.w = 1.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned {name}')
            else:
                self.get_logger().error(f'Failed to spawn {name}: {response.status_message}')
        else:
            self.get_logger().error(f'Exception while spawning {name}: {future.exception()}')

def main(args=None):
    rclpy.init(args=args)
    spawner = DynamicObjectSpawner()

    # Keep the node alive to maintain service connections
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 4: Launch the World and Spawning Node

Create a launch file to start the world and spawner:

```python
# launch/custom_world_lab.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('your_robot_package'),
                'worlds',
                'custom_lab_world.world'
            ])
        }.items()
    )

    # Launch the dynamic spawner node
    dynamic_spawner = Node(
        package='your_robot_package',
        executable='dynamic_object_spawner',
        name='dynamic_object_spawner',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        dynamic_spawner
    ])
```

## Lab Exercises

1. **Custom Terrain Creation**: Create a world with custom terrain using heightmaps or mesh files
2. **Lighting Configuration**: Add custom lighting to your world and experiment with different light types
3. **Multi-Object Spawning**: Extend the spawner to handle multiple object types and positions
4. **Environment Interaction**: Test how spawned objects interact with the static environment

## Validation Steps

1. Launch your custom world:
   ```bash
   ros2 launch your_robot_package custom_world_lab.launch.py
   ```

2. Verify that the static objects (building, table) are properly placed

3. Check that dynamic objects are successfully spawned

4. Test physics interactions between objects

5. Validate that the simulation runs stably

## Expected Outcomes

- Custom world file with multiple static objects
- Working dynamic object spawning system
- Proper ROS 2 integration for object spawning
- Stable physics simulation with all objects

## References

1. Gazebo Documentation. (2023). World File Format. http://gazebosim.org/tutorials?tut=build_world
2. ROS 2 Documentation. (2023). Gazebo Integration. https://docs.ros.org/en/humble/p/gazebo_ros_pkgs/
3. Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo. Proceedings of the 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems.