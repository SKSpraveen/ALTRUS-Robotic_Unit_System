#!/usr/bin/env python3
"""
Navigation Configuration Generator for ALTRUS Framework
Generates Nav2 parameters based on robot configuration
"""

import yaml
from jinja2 import Template
from ..config.manager import RobotConfig


class NavigationGenerator:
    """Generates Nav2 configuration files"""

    @staticmethod
    def generate_nav2_params(config: RobotConfig, slam_mode: bool = False) -> str:
        """Generate nav2_params.yaml based on robot config"""

        nav = config.get('navigation')
        robot_radius = config.get('navigation.obstacle_padding')

        # ✅ NEW: planner selection from config.yaml
        planner = (config.get('navigation.planner', 'navfn') or 'navfn').lower()
        use_astar = bool(config.get('navigation.use_astar', False))

        params = {
            'amcl': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'alpha1': 0.2,
                    'alpha2': 0.2,
                    'alpha3': 0.2,
                    'alpha4': 0.2,
                    'alpha5': 0.2,
                    'min_particles': 2000,
                    'max_particles': 8000,
                    'kld_err': 0.05,
                    'kld_z': 0.99,
                    'pf_err': 0.05,
                    'pf_z': 0.99,
                    'laser_model_type': 'likelihood_field',
                    'max_beams': 60,
                    'z_hit': 0.95,
                    'z_rand': 0.05,
                    'sigma_hit': 0.2,
                    'lambda_short': 0.1,
                    'laser_likelihood_max_dist': 2.0,
                    'update_min_d': 0.1,
                    'update_min_a': 0.15,
                    'resample_interval': 1,
                    'transform_tolerance': 0.5,
                    'tf_broadcast': True,
                    'recovery_alpha_fast': 0.0,
                    'recovery_alpha_slow': 0.0,
                    'odom_frame_id': 'odom',
                    'base_frame_id': 'base_footprint',
                    'global_frame_id': 'map',
                    'scan_topic': config.get('sensors.lidar.topic', '/scan'),
                    'set_initial_pose': True,
                    'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
                }
            },

            'map_server': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'yaml_filename': '',
                    'frame_id': 'map'
                }
            },

            # ✅ UPDATED: planner_server no longer hard-coded NavFn
            'planner_server': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'expected_planner_frequency': 5.0,
                    'planner_plugins': ['GridBased'],
                }
            },

            'controller_server': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'controller_frequency': 10.0,
                    'progress_checker_plugins': ['progress_checker'],
                    'goal_checker_plugins': ['stopped_goal_checker'],
                    'controller_plugins': ['FollowPath'],

                    'progress_checker': {
                        'plugin': 'nav2_controller::SimpleProgressChecker',
                        'required_movement_radius': 0.3,
                        'movement_time_allowance': 20.0 if not slam_mode else 60.0
                    },

                    'stopped_goal_checker': {
                        'plugin': 'nav2_controller::StoppedGoalChecker',
                        'xy_goal_tolerance': nav['goal_tolerance']['xy'],
                        'yaw_goal_tolerance': nav['goal_tolerance']['yaw'],
                        'trans_stopped_velocity': 0.05,
                        'rot_stopped_velocity': 0.05,
                        'stateful': True
                    },

                    'FollowPath': {
                        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                        'desired_linear_vel': nav['max_linear_velocity'] * 0.8,
                        'max_linear_accel': nav['max_linear_acceleration'],
                        'max_linear_decel': nav['max_linear_acceleration'] * 2,
                        'min_linear_vel': 0.0,
                        'min_angular_vel': 0.0,
                        'approach_velocity_scaling_dist': 0.5 if not slam_mode else 0.8,
                        'min_approach_linear_velocity': 0.0,
                        'lookahead_dist': 0.6 if not slam_mode else 0.5,
                        'min_lookahead_dist': 0.3 if not slam_mode else 0.25,
                        'max_lookahead_dist': 0.9 if not slam_mode else 0.7,
                        'lookahead_time': 1.5,
                        'transform_tolerance': 0.5,
                        'use_velocity_scaled_lookahead_dist': True,
                        'use_cost_regulated_linear_velocity_scaling': True,
                        'use_rotate_to_heading': True,
                        'rotate_to_heading_angular_vel': nav['max_angular_velocity'] * 0.8,
                        'rotate_to_heading_min_angle': 0.5 if not slam_mode else 0.4,
                        'max_angular_vel': nav['max_angular_velocity'],
                        'cost_scaling_dist': 0.6 if not slam_mode else 0.4,
                        'cost_scaling_gain': 1.0 if not slam_mode else 0.5,
                        'inflation_cost_scaling_factor': 3.0 if not slam_mode else 2.0,
                        'allow_reversing': False
                    }
                }
            },

            'bt_navigator': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'global_frame': 'map',
                    'robot_base_frame': 'base_footprint',
                    'transform_tolerance': 0.2 if not slam_mode else 0.5,
                    'default_nav_to_pose_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
                    'default_nav_through_poses_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml'
                }
            },

            'behavior_server': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'global_frame': 'map',
                    'robot_base_frame': 'base_footprint',
                    'transform_tolerance': 0.2,
                    'behavior_plugins': ['spin', 'backup', 'drive_on_heading', 'wait'],

                    'spin': {
                        'plugin': 'nav2_behaviors/Spin',
                        'simulate_ahead_time': 2.0,
                        'max_rotational_vel': nav['max_angular_velocity'] * 0.5,
                        'min_rotational_vel': nav['max_angular_velocity'] * 0.2,
                        'rotational_acc_lim': nav['max_angular_acceleration']
                    },

                    'backup': {
                        'plugin': 'nav2_behaviors/BackUp',
                        'simulate_ahead_time': 2.0,
                        'backup_dist': 0.3 if not slam_mode else 0.8,
                        'backup_speed': nav['max_linear_velocity'] * 0.3
                    },

                    'drive_on_heading': {'plugin': 'nav2_behaviors/DriveOnHeading'},
                    'wait': {'plugin': 'nav2_behaviors/Wait'}
                }
            },

            'waypoint_follower': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'loop_rate': 20,
                    'stop_on_failure': False,
                    'waypoint_task_executor_plugin': 'wait_at_waypoint',
                    'wait_at_waypoint': {
                        'plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
                        'enabled': True,
                        'waypoint_pause_duration': 0
                    }
                }
            },

            'global_costmap': {
                'global_costmap': {
                    'ros__parameters': {
                        'use_sim_time': True,
                        'global_frame': 'map',
                        'robot_base_frame': 'base_footprint',
                        'update_frequency': 2.0 if not slam_mode else 5.0,
                        'publish_frequency': 1.0 if not slam_mode else 2.0,
                        'resolution': nav['costmap_resolution'],
                        'track_unknown_space': True,
                        'rolling_window': False,
                        'width': nav['global_costmap_size']['width'],
                        'height': nav['global_costmap_size']['height'],
                        'origin_x': -nav['global_costmap_size']['width'] / 2 if not slam_mode else 0.0,
                        'origin_y': -nav['global_costmap_size']['height'] / 2 if not slam_mode else 0.0,
                        'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],

                        'static_layer': {
                            'plugin': 'nav2_costmap_2d::StaticLayer',
                            'map_subscribe_transient_local': True
                        },

                        'obstacle_layer': {
                            'plugin': 'nav2_costmap_2d::ObstacleLayer',
                            'observation_sources': 'scan',
                            'scan': {
                                'topic': config.get('sensors.lidar.topic', '/scan'),
                                'max_obstacle_height': 2.0,
                                'clearing': True,
                                'marking': True,
                                'data_type': 'LaserScan',
                                'obstacle_range': config.get('sensors.lidar.max_range', 6.0) - 0.5,
                                'raytrace_range': config.get('sensors.lidar.max_range', 6.0) if not slam_mode else 10.0,
                                'expected_update_rate': 0.0
                            }
                        },

                        'inflation_layer': {
                            'plugin': 'nav2_costmap_2d::InflationLayer',
                            'inflation_radius': robot_radius * 1.5 if not slam_mode else robot_radius * 0.75,
                            'cost_scaling_factor': 3.0 if not slam_mode else 2.0
                        }
                    }
                }
            },

            'local_costmap': {
                'local_costmap': {
                    'ros__parameters': {
                        'use_sim_time': True,
                        'global_frame': 'odom',
                        'robot_base_frame': 'base_footprint',
                        'update_frequency': 10.0,
                        'publish_frequency': 5.0,
                        'resolution': nav['costmap_resolution'],
                        'robot_radius': robot_radius,
                        'footprint_padding': 0.03,
                        'rolling_window': True,
                        'width': nav['local_costmap_size']['width'],
                        'height': nav['local_costmap_size']['height'],
                        'origin_x': 0.0,
                        'origin_y': 0.0,
                        'always_send_full_costmap': True,
                        'plugins': ['obstacle_layer', 'inflation_layer'],

                        'obstacle_layer': {
                            'plugin': 'nav2_costmap_2d::ObstacleLayer',
                            'observation_sources': 'scan',
                            'scan': {
                                'topic': config.get('sensors.lidar.topic', '/scan'),
                                'max_obstacle_height': 2.0,
                                'clearing': True,
                                'marking': True,
                                'data_type': 'LaserScan',
                                'obstacle_range': 3.5 if not slam_mode else 6.0,
                                'raytrace_range': 4.0 if not slam_mode else 10.0
                            }
                        },

                        'inflation_layer': {
                            'plugin': 'nav2_costmap_2d::InflationLayer',
                            'inflation_radius': robot_radius if not slam_mode else robot_radius * 0.75,
                            'cost_scaling_factor': 3.0 if not slam_mode else 2.0
                        }
                    }
                }
            },

            'lifecycle_manager': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'map_server' if not slam_mode else None,
                        'amcl' if not slam_mode else None,
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower'
                    ]
                }
            }
        }

        # Remove None values from lifecycle manager
        params['lifecycle_manager']['ros__parameters']['node_names'] = [
            n for n in params['lifecycle_manager']['ros__parameters']['node_names'] if n
        ]

        # If SLAM mode, remove AMCL and map_server
        if slam_mode:
            del params['amcl']
            del params['map_server']

        # ============================
        # ✅ PLANNER SELECTION (NEW)
        # ============================
        if planner in ('navfn', 'navfn_planner'):
            params['planner_server']['ros__parameters']['GridBased'] = {
                'plugin': 'nav2_navfn_planner/NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': use_astar,
                'allow_unknown': True
            }

        elif planner in ('smac', 'smac_2d'):
            params['planner_server']['ros__parameters']['GridBased'] = {
                'plugin': 'nav2_smac_planner/SmacPlanner2D',
                'tolerance': 0.5,
                'allow_unknown': True,

                # safe defaults
                'motion_model_for_search': 'MOORE',
                'angle_quantization_bins': 1,
                'minimum_turning_radius': 0.0,
                'reverse_penalty': 2.0,
                'change_penalty': 0.0,
                'non_straight_penalty': 1.0,
                'cost_penalty': 2.0,
                'smooth_path': True
            }
        else:
            # fallback
            params['planner_server']['ros__parameters']['GridBased'] = {
                'plugin': 'nav2_navfn_planner/NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True
            }

        return yaml.dump(params, default_flow_style=False, sort_keys=False)

    @staticmethod
    def generate_slam_config(config: RobotConfig) -> str:
        """Generate SLAM Toolbox configuration"""
        slam_cfg = config.get('slam')

        params = {
            'slam_toolbox': {
                'ros__parameters': {
                    'use_sim_time': True,
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'scan_topic': config.get('sensors.lidar.topic', '/scan'),
                    'mode': slam_cfg['mode'],
                    'resolution': slam_cfg['resolution'],
                    'map_update_interval': slam_cfg['update_interval'],
                    'min_laser_range': config.get('sensors.lidar.min_range', 0.12),
                    'max_laser_range': config.get('sensors.lidar.max_range', 6.0),
                    'throttle_scans': 1,
                    'minimum_travel_distance': slam_cfg['min_travel_distance'],
                    'minimum_travel_heading': slam_cfg['min_travel_heading'],
                    'transform_publish_period': 0.02,
                    'tf_buffer_duration': 30.0,
                    'transform_timeout': 0.2,
                    'solver_plugin': 'solver_plugins::CeresSolver',
                    'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                    'ceres_preconditioner': 'SCHUR_JACOBI',
                    'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                    'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                    'ceres_loss_function': 'None',
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'scan_buffer_size': 10,
                    'scan_buffer_maximum_scan_distance': 10.0,
                    'link_match_minimum_response_fine': 0.1,
                    'link_scan_maximum_distance': 1.5,
                    'loop_search_maximum_distance': 3.0,
                    'do_loop_closing': True,
                    'loop_match_minimum_chain_size': 10,
                    'loop_match_maximum_variance_coarse': 3.0,
                    'loop_match_minimum_response_coarse': 0.35,
                    'loop_match_minimum_response_fine': 0.45,
                    'correlation_search_space_dimension': 0.5,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.1,
                    'minimum_angle_penalty': 0.9,
                    'minimum_distance_penalty': 0.5,
                    'use_response_expansion': True
                }
            }
        }

        return yaml.dump(params, default_flow_style=False, sort_keys=False)

    @staticmethod
    def generate_launch_file(robot_name: str, slam_mode: bool = False) -> str:
        """Generate navigation launch file"""

        if slam_mode:
            launch_template = '''# {{ robot_name }}_navigation/launch/slam.launch.py
# Auto-generated by ALTRUS Framework

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_config = os.path.join(
        get_package_share_directory('{{ robot_name }}_navigation'),
        'config',
        'slam_toolbox.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('{{ robot_name }}_navigation'),
        'config',
        'slam.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        slam_node,
        rviz_node
    ])
'''
        else:
            launch_template = '''# {{ robot_name }}_navigation/launch/nav2_bringup.launch.py
# Auto-generated by ALTRUS Framework

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    pkg_share = get_package_share_directory('{{ robot_name }}_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server', 'amcl', 'planner_server', 'controller_server',
                    'behavior_server', 'bt_navigator', 'waypoint_follower'
                ]
            }]
        ),
    ])
'''
        return Template(launch_template).render(robot_name=robot_name)

