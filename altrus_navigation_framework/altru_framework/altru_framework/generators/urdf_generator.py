#!/usr/bin/env python3
"""
URDF Generator for ALTRUS Framework
Generates robot URDF based on configuration
"""

from pathlib import Path
from jinja2 import Template
from ..config.manager import RobotConfig


class URDFGenerator:
    """Generates URDF files from robot configuration"""

    URDF_TEMPLATE = '''<?xml version="1.0"?>
<robot name="{{ robot_name }}"
       xmlns:xacro="http://www.ros.org/wiki/xacro"
       xmlns:gazebo="http://gazebosim.org/gazebo">

  <!-- ============================
       COMMON PROPERTIES
  ============================= -->
  <xacro:property name="base_length"  value="{{ dimensions.length }}"/>
  <xacro:property name="base_width"   value="{{ dimensions.width }}"/>
  <xacro:property name="base_height"  value="{{ dimensions.height }}"/>

  <xacro:property name="wheel_radius" value="{{ dimensions.wheel_radius }}"/>
  <xacro:property name="wheel_width"  value="{{ dimensions.wheel_width }}"/>

  <!-- Tracked -->
  <xacro:property name="track_length"       value="{{ dimensions.track_length }}"/>
  <xacro:property name="track_width"        value="{{ dimensions.track_width }}"/>
  <xacro:property name="track_height"       value="{{ dimensions.track_height }}"/>
  <xacro:property name="track_separation"   value="{{ dimensions.track_separation }}"/>
  <xacro:property name="drive_wheel_radius" value="{{ dimensions.drive_wheel_radius }}"/>
  <xacro:property name="drive_wheel_width"  value="{{ dimensions.drive_wheel_width }}"/>

  {% if sensors.lidar.enabled %}
  <xacro:property name="lidar_radius" value="0.05"/>
  <xacro:property name="lidar_height" value="0.06"/>
  {% endif %}

  <!-- ============================
       INERTIAL HELPERS
  ============================= -->
  <xacro:macro name="inertial_box" params="mass x y z">
    <inertial>
      <mass value="${mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="${mass/12.0 * (y*y + z*z)}"
        iyy="${mass/12.0 * (x*x + z*z)}"
        izz="${mass/12.0 * (x*x + y*y)}"
        ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="inertial_cylinder" params="mass radius length">
    <inertial>
      <mass value="${mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="${mass/12.0 * (3*radius*radius + length*length)}"
        iyy="${mass/12.0 * (3*radius*radius + length*length)}"
        izz="${mass/2.0 * radius*radius}"
        ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

  <!-- ============================
       BASE LINK
       Wheeled lift uses wheel_radius
       Tracked lift uses track_height
  ============================= -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      {% if base_type == 'tracked' %}
      <origin xyz="0 0 ${track_height + base_height/2.0}" rpy="0 0 0"/>
      {% else %}
      <origin xyz="0 0 ${wheel_radius + base_height/2.0}" rpy="0 0 0"/>
      {% endif %}
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      {% if base_type == 'tracked' %}
      <origin xyz="0 0 ${track_height + base_height/2.0}" rpy="0 0 0"/>
      {% else %}
      <origin xyz="0 0 ${wheel_radius + base_height/2.0}" rpy="0 0 0"/>
      {% endif %}
    </collision>

    <xacro:inertial_box mass="8.0"
                        x="${base_length}"
                        y="${base_width}"
                        z="${base_height}"/>
  </link>

  <link name="base_footprint"/>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- ============================
       WHEEL MACRO (wheeled)
  ============================= -->
  <xacro:macro name="wheel" params="name parent xyz rpy">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      </collision>

      <xacro:inertial_cylinder mass="0.3"
                               radius="${wheel_radius}"
                               length="${wheel_width}"/>
    </link>

    <joint name="${name}_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  {% if base_type == 'wheeled' %}
  <xacro:wheel name="rear_left_wheel"
               parent="base_link"
               xyz="${-base_length/4.0} ${ base_width/2.0 - wheel_width/2.0} ${wheel_radius}"
               rpy="0 0 0"/>

  <xacro:wheel name="rear_right_wheel"
               parent="base_link"
               xyz="${-base_length/4.0} ${-base_width/2.0 + wheel_width/2.0} ${wheel_radius}"
               rpy="0 0 0"/>

  <xacro:wheel name="front_left_support_wheel"
               parent="base_link"
               xyz="${base_length/4.0}  ${ base_width/2.0 - wheel_width/2.0} ${wheel_radius}"
               rpy="0 0 0"/>

  <xacro:wheel name="front_right_support_wheel"
               parent="base_link"
               xyz="${base_length/4.0}  ${-base_width/2.0 + wheel_width/2.0} ${wheel_radius}"
               rpy="0 0 0"/>

  <gazebo reference="rear_left_wheel"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
  <gazebo reference="rear_right_wheel"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
  <gazebo reference="front_left_support_wheel"><mu1>0.01</mu1><mu2>0.01</mu2></gazebo>
  <gazebo reference="front_right_support_wheel"><mu1>0.01</mu1><mu2>0.01</mu2></gazebo>
  {% endif %}

  <!-- ============================
       TRACKED BASE
       - Track visuals fixed
       - Hidden drive wheels continuous
  ============================= -->
  {% if base_type == 'tracked' %}

  <link name="left_track">
    <visual>
      <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="black"><color rgba="0.05 0.05 0.05 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <xacro:inertial_box mass="1.0" x="${track_length}" y="${track_width}" z="${track_height}"/>
  </link>

  <joint name="left_track_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_track"/>
    <origin xyz="0 ${track_separation/2.0} ${track_height/2.0}" rpy="0 0 0"/>
  </joint>

  <link name="right_track">
    <visual>
      <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="black"><color rgba="0.05 0.05 0.05 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <xacro:inertial_box mass="1.0" x="${track_length}" y="${track_width}" z="${track_height}"/>
  </link>

  <joint name="right_track_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_track"/>
    <origin xyz="0 ${-track_separation/2.0} ${track_height/2.0}" rpy="0 0 0"/>
  </joint>

  <!-- Hidden drive wheels -->
  <link name="left_drive_wheel">
    <visual>
      <geometry><cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="invisible"><color rgba="0 0 0 0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <xacro:inertial_cylinder mass="0.3" radius="${drive_wheel_radius}" length="${drive_wheel_width}"/>
  </link>

  <joint name="left_drive_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_drive_wheel"/>
    <origin xyz="0 ${track_separation/2.0} ${drive_wheel_radius}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_drive_wheel">
    <visual>
      <geometry><cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="invisible"><color rgba="0 0 0 0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${drive_wheel_radius}" length="${drive_wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <xacro:inertial_cylinder mass="0.3" radius="${drive_wheel_radius}" length="${drive_wheel_width}"/>
  </link>

  <joint name="right_drive_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_drive_wheel"/>
    <origin xyz="0 ${-track_separation/2.0} ${drive_wheel_radius}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <gazebo reference="left_track"><mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
  <gazebo reference="right_track"><mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
  <gazebo reference="left_drive_wheel"><mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
  <gazebo reference="right_drive_wheel"><mu1>2.0</mu1><mu2>2.0</mu2></gazebo>

  {% endif %}

  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  {% if sensors.lidar.enabled %}
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="${lidar_radius}" length="${lidar_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue"><color rgba="0.2 0.2 0.9 1.0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${lidar_radius}" length="${lidar_height}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <xacro:inertial_cylinder mass="0.3" radius="${lidar_radius}" length="${lidar_height}"/>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    {% if base_type == 'tracked' %}
    <origin xyz="{{ sensors.lidar.position.x }} {{ sensors.lidar.position.y }} ${track_height + base_height + lidar_height/2.0 + 0.02}" rpy="0 0 0"/>
    {% else %}
    <origin xyz="{{ sensors.lidar.position.x }} {{ sensors.lidar.position.y }} ${wheel_radius + base_height + lidar_height/2.0 + 0.02}" rpy="0 0 0"/>
    {% endif %}
  </joint>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar">
      <always_on>true</always_on>
      <update_rate>{{ sensors.lidar.update_rate }}</update_rate>
      <visualize>true</visualize>
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>{{ sensors.lidar.samples }}</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>{{ sensors.lidar.min_range }}</min>
          <max>{{ sensors.lidar.max_range }}</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
        <ros><namespace>/</namespace><remapping>~/out:={{ sensors.lidar.topic }}</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>{{ sensors.lidar.frame }}</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  {% endif %}

  <!-- =========================================================
       CAMERA (NEW) - Only if sensors.camera.enabled == True
       This makes Gazebo publish /camera/image_raw (or your topic)
  ========================================================== -->
  {% if sensors.camera.enabled %}
  <link name="camera_link">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </collision>
    <xacro:inertial_box mass="0.2" x="0.05" y="0.05" z="0.05"/>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    {% if base_type == 'tracked' %}
    <origin xyz="{{ sensors.camera.position.x }} {{ sensors.camera.position.y }} ${track_height + base_height + 0.10}" rpy="0 0 0"/>
    {% else %}
    <origin xyz="{{ sensors.camera.position.x }} {{ sensors.camera.position.y }} ${wheel_radius + base_height + 0.10}" rpy="0 0 0"/>
    {% endif %}
  </joint>

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <always_on>true</always_on>
      <update_rate>{{ sensors.camera.update_rate }}</update_rate>
      <camera>
        <horizontal_fov>{{ sensors.camera.fov }}</horizontal_fov>
        <image>
          <width>{{ sensors.camera.width }}</width>
          <height>{{ sensors.camera.height }}</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>20.0</far>
        </clip>
      </camera>

      <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/</namespace>
          <remapping>image_raw:={{ sensors.camera.topic }}</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>{{ sensors.camera.frame }}</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  {% endif %}

  {% if actuators.differential_drive.enabled %}
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
      </ros>

      <update_rate>50</update_rate>

      <left_joint>{{ actuators.differential_drive.left_joint }}</left_joint>
      <right_joint>{{ actuators.differential_drive.right_joint }}</right_joint>

      <wheel_separation>{{ actuators.differential_drive.wheel_separation }}</wheel_separation>
      <wheel_diameter>{{ actuators.differential_drive.wheel_diameter }}</wheel_diameter>

      <max_wheel_torque>{{ actuators.differential_drive.max_wheel_torque }}</max_wheel_torque>
      <max_wheel_acceleration>{{ actuators.differential_drive.max_wheel_acceleration }}</max_wheel_acceleration>

      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>

      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>
  {% endif %}

</robot>
'''

    def __init__(self, template_str: str = None):
        self.template = Template(template_str if template_str else self.URDF_TEMPLATE)

    def generate(self, config: RobotConfig) -> str:
        context = config.to_dict()
        return self.template.render(**context)

    def generate_to_file(self, config: RobotConfig, output_path: Path):
        urdf_content = self.generate(config)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write(urdf_content)

    @staticmethod
    def generate_launch_file(robot_name: str) -> str:
        launch_template = '''# {{ robot_name }}_description/launch/robot_state_publisher.launch.py
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_xacro = os.path.join(
        get_package_share_directory('{{ robot_name }}_description'),
        'urdf',
        '{{ robot_name }}.urdf.xacro'
    )
    robot_description = Command(['xacro', ' ', robot_description_xacro])
    robot_description_param = ParameterValue(robot_description, value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param
        }]
    )
    return LaunchDescription([robot_state_publisher])
'''
        return Template(launch_template).render(robot_name=robot_name)

