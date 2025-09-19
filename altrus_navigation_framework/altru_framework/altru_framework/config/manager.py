#!/usr/bin/env python3
"""
Configuration Manager for ALTRUS Framework
Handles loading, validation, and storage of robot configurations
"""

import yaml
import json
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime


class RobotConfig:
    """Robot configuration data structure"""

    DEFAULT_CONFIG = {
        'robot_name': 'altrus_robot',
        'version': '1.0.0',
        'base_type': 'wheeled',
        'description': 'Assistive robot with multimodal control',
        'created_at': None,
        'modified_at': None,

        'dimensions': {
            'length': 0.40,      # meters
            'width': 0.30,
            'height': 0.12,

            # Wheeled base params
            'wheel_radius': 0.05,
            'wheel_width': 0.02,

            # Tracked base params
            'track_length': 0.35,
            'track_width': 0.06,
            'track_height': 0.06,
            'track_separation': 0.24,
            'drive_wheel_radius': 0.03,
            'drive_wheel_width': 0.02,

            'base_clearance': 0.05,
        },

        'sensors': {
            'lidar': {
                'enabled': True,
                'model': 'rplidar_a1',
                'topic': '/scan',
                'frame': 'lidar_link',
                'update_rate': 10.0,
                'min_range': 0.12,
                'max_range': 6.0,
                'samples': 360,
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.2}
            },
            'camera': {
                'enabled': False,
                'model': 'generic_camera',
                'topic': '/camera/image_raw',
                'frame': 'camera_link',
                'update_rate': 30.0,
                'width': 640,
                'height': 480,
                'fov': 1.396,
                'position': {'x': 0.2, 'y': 0.0, 'z': 0.2}
            },
            'imu': {
                'enabled': False,
                'model': 'generic_imu',
                'topic': '/imu/data',
                'frame': 'imu_link',
                'update_rate': 100.0,
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.1}
            },
            'depth_camera': {
                'enabled': False,
                'model': 'realsense_d435',
                'topic': '/depth/image_raw',
                'frame': 'depth_camera_link',
                'position': {'x': 0.2, 'y': 0.0, 'z': 0.2}
            }
        },

        'actuators': {
            'differential_drive': {
                'enabled': True,

                # Default wheeled joints
                'left_joint': 'rear_left_wheel_joint',
                'right_joint': 'rear_right_wheel_joint',

                # Will be recalculated depending on base_type
                'wheel_separation': 0.28,
                'wheel_diameter': 0.10,

                'max_wheel_torque': 20.0,
                'max_wheel_acceleration': 2.0
            }
        },

        'navigation': {
            'max_linear_velocity': 0.3,
            'max_angular_velocity': 1.0,
            'max_linear_acceleration': 0.5,
            'max_angular_acceleration': 2.0,
            'obstacle_padding': 0.22,
            'costmap_resolution': 0.05,
            'local_costmap_size': {'width': 10, 'height': 10},
            'global_costmap_size': {'width': 50, 'height': 50},
            'controller': 'regulated_pure_pursuit',

            # ✅ Planner selection:
            # - navfn: NavFn planner (supports Dijkstra or A* via use_astar)
            # - smac_2d: SmacPlanner2D (A* based)
            'planner': 'navfn',
            'use_astar': False,  # ✅ NEW (used only for navfn)

            'goal_tolerance': {'xy': 0.4, 'yaw': 0.8}
        },

        'slam': {
            'mode': 'mapping',
            'resolution': 0.05,
            'update_interval': 5.0,
            'min_travel_distance': 0.2,
            'min_travel_heading': 0.2,
        },

        'multimodal': {
            'voice': {
                'enabled': True,
                'engine': 'simulated',
                'language': 'en-US',
                'confidence_threshold': 0.7,
                'topic': '/voice/command'
            },
            'gesture': {
                'enabled': False,
                'engine': 'mediapipe',
                'confidence_threshold': 0.8,
                'topic': '/gesture/command',
                'camera_source': '/camera/image_raw'
            },
            'locations': {
                'kitchen': {'x': 5.0, 'y': 3.0, 'yaw': 1.57},
                'bedroom': {'x': -5.0, 'y': -3.0, 'yaw': 3.14},
                'living_room': {'x': -5.0, 'y': 3.0, 'yaw': 0.0},
                'bathroom': {'x': 5.0, 'y': -3.0, 'yaw': -1.57},
                'hallway': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            }
        },

        'simulation': {
            'world': 'multi_room_house',
            'spawn_position': {'x': 0.0, 'y': 0.0, 'z': 0.01},
            'spawn_orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },

        'safety': {
            'emergency_stop_enabled': True,
            'max_runtime_hours': 8.0,
            'low_battery_threshold': 20.0,
            'obstacle_stop_distance': 0.3
        }
    }

    def __init__(self, config_dict: Optional[Dict[str, Any]] = None):
        # Shallow copy of root, then deep-update dicts
        self.config = self.DEFAULT_CONFIG.copy()
        if config_dict:
            self._deep_update(self.config, config_dict)

        if not self.config.get('created_at'):
            self.config['created_at'] = datetime.now().isoformat()
        self.config['modified_at'] = datetime.now().isoformat()

        self._calculate_derived_values()

    def _deep_update(self, base: Dict, update: Dict) -> Dict:
        for key, value in update.items():
            if isinstance(value, dict) and key in base and isinstance(base[key], dict):
                self._deep_update(base[key], value)
            else:
                base[key] = value
        return base

    def _calculate_derived_values(self):
        """Calculate derived configuration values based on base_type."""
        dims = self.config['dimensions']
        base_type = self.config.get('base_type', 'wheeled')
        dd = self.config['actuators']['differential_drive']

        if base_type == 'tracked':
            # diff_drive must drive rotating joints (hidden drive wheels)
            dd['left_joint'] = 'left_drive_wheel_joint'
            dd['right_joint'] = 'right_drive_wheel_joint'

            dd['wheel_separation'] = float(dims.get('track_separation', 0.24))
            dd['wheel_diameter'] = float(dims.get('drive_wheel_radius', 0.03)) * 2.0

            if dd.get('max_wheel_torque', 20.0) < 30.0:
                dd['max_wheel_torque'] = 40.0
            if dd.get('max_wheel_acceleration', 2.0) < 3.0:
                dd['max_wheel_acceleration'] = 3.0

        else:
            dd['left_joint'] = dd.get('left_joint', 'rear_left_wheel_joint')
            dd['right_joint'] = dd.get('right_joint', 'rear_right_wheel_joint')
            dd['wheel_separation'] = float(dims['width']) - float(dims['wheel_width'])
            dd['wheel_diameter'] = float(dims['wheel_radius']) * 2.0

        max_dimension = max(float(dims['length']), float(dims['width']))
        self.config['navigation']['obstacle_padding'] = max_dimension / 2.0 + 0.05

    @classmethod
    def from_file(cls, filepath: Path) -> 'RobotConfig':
        with open(filepath, 'r') as f:
            config_dict = yaml.safe_load(f)
        return cls(config_dict)

    def to_file(self, filepath: Path):
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)

    def to_dict(self) -> Dict[str, Any]:
        return self.config.copy()

    def get(self, key_path: str, default=None):
        keys = key_path.split('.')
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

    def set(self, key_path: str, value: Any):
        keys = key_path.split('.')
        config = self.config
        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]
        config[keys[-1]] = value
        self.config['modified_at'] = datetime.now().isoformat()

    def validate(self) -> tuple[bool, list[str]]:
        errors = []
        if not self.config.get('robot_name'):
            errors.append("robot_name is required")

        if self.config['base_type'] not in ['wheeled', 'tracked', 'legged']:
            errors.append(f"Invalid base_type: {self.config['base_type']}")

        dims = self.config['dimensions']
        if dims['length'] <= 0 or dims['width'] <= 0 or dims['height'] <= 0:
            errors.append("Dimensions must be positive values")

        if not self.config['sensors']['lidar']['enabled']:
            errors.append("LIDAR sensor is required for navigation")

        if self.config['multimodal']['gesture']['enabled'] and \
           not self.config['sensors']['camera']['enabled']:
            errors.append("Camera must be enabled for gesture control")

        nav = self.config['navigation']
        if nav['max_linear_velocity'] <= 0 or nav['max_angular_velocity'] <= 0:
            errors.append("Velocity limits must be positive")

        if self.config['base_type'] == 'tracked':
            if dims.get('track_separation', 0.0) <= 0:
                errors.append("track_separation must be > 0 for tracked base")
            if dims.get('drive_wheel_radius', 0.0) <= 0:
                errors.append("drive_wheel_radius must be > 0 for tracked base")

        # ✅ Validate planner value
        planner = (nav.get('planner') or 'navfn').lower()
        if planner not in ('navfn', 'smac_2d', 'smac'):
            errors.append("navigation.planner must be 'navfn' or 'smac_2d'")

        return (len(errors) == 0, errors)

    def get_package_names(self) -> Dict[str, str]:
        robot_name = self.config['robot_name']
        return {
            'description': f"{robot_name}_description",
            'navigation': f"{robot_name}_navigation",
            'gazebo': f"{robot_name}_gazebo",
            'bringup': f"{robot_name}_bringup",
            'voice': f"{robot_name}_voice",
            'gesture': f"{robot_name}_gesture",
            'exploration': f"{robot_name}_exploration"
        }

    def __str__(self) -> str:
        return yaml.dump(self.config, default_flow_style=False)


class ConfigManager:
    """Manages robot configurations"""

    def __init__(self, config_dir: Optional[Path] = None):
        if config_dir is None:
            config_dir = Path.home() / '.altrus' / 'robots'
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)

    def create_config(self, robot_name: str, **kwargs) -> RobotConfig:
        config_dict = {'robot_name': robot_name}
        config_dict.update(kwargs)
        config = RobotConfig(config_dict)
        config_path = self.config_dir / robot_name / 'config.yaml'
        config.to_file(config_path)
        return config

    def load_config(self, robot_name: str) -> Optional[RobotConfig]:
        config_path = self.config_dir / robot_name / 'config.yaml'
        if not config_path.exists():
            return None
        return RobotConfig.from_file(config_path)

    def save_config(self, config: RobotConfig):
        robot_name = config.config['robot_name']
        config_path = self.config_dir / robot_name / 'config.yaml'
        config.to_file(config_path)

    def list_configs(self) -> list[str]:
        robots = []
        for item in self.config_dir.iterdir():
            if item.is_dir():
                config_file = item / 'config.yaml'
                if config_file.exists():
                    robots.append(item.name)
        return sorted(robots)

    def delete_config(self, robot_name: str) -> bool:
        config_dir = self.config_dir / robot_name
        if config_dir.exists():
            import shutil
            shutil.rmtree(config_dir)
            return True
        return False

    def export_config(self, robot_name: str, output_path: Path, format: str = 'yaml'):
        config = self.load_config(robot_name)
        if not config:
            raise ValueError(f"Robot '{robot_name}' not found")

        if format == 'yaml':
            config.to_file(output_path)
        elif format == 'json':
            with open(output_path, 'w') as f:
                json.dump(config.to_dict(), f, indent=2)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def import_config(self, filepath: Path) -> RobotConfig:
        config = RobotConfig.from_file(filepath)
        self.save_config(config)
        return config

