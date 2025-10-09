#!/usr/bin/env python3
"""
ALTRUS Framework CLI - Main Entry Point
Complete command-line interface for robot configuration
"""

import click
import yaml
import subprocess
from pathlib import Path
import os

# Use absolute imports for installed package
from altru_framework.config.manager import ConfigManager, RobotConfig
from altru_framework.generators.urdf_generator import URDFGenerator
from altru_framework.generators.navigation_generator import NavigationGenerator


@click.group()
@click.version_option(version="1.0.0")
def cli():
    """
    🤖 ALTRUS Framework - Adaptive Navigation for Assistive Robots

    Get started:
      altrus-cli init my_robot --base wheeled
      altrus-cli configure my_robot
      altrus-cli build my_robot
      altrus-cli launch my_robot
    """
    pass


@cli.command()
@click.argument("robot_name")
@click.option("--base", type=click.Choice(["wheeled", "tracked", "legged"]), default="wheeled", help="Robot base type")
@click.option("--template", help="Use existing robot as template")
def init(robot_name, base, template):
    """Initialize a new robot project"""
    manager = ConfigManager()

    click.echo(f"🤖 Initializing robot: {robot_name}")
    click.echo(f"📦 Base type: {base}")

    # Check if already exists
    if manager.load_config(robot_name):
        if not click.confirm(f"Robot '{robot_name}' already exists. Overwrite?"):
            return

    # Load template if specified
    template_config = {}
    if template:
        template_robot = manager.load_config(template)
        if template_robot:
            template_config = template_robot.to_dict()
            click.echo(f"📋 Using template from: {template}")
        else:
            click.echo(f"⚠️  Template '{template}' not found, using defaults")

    # Create config
    manager.create_config(robot_name, base_type=base, **template_config)

    click.echo("✅ Created robot configuration")
    click.echo(f"📁 Location: {manager.config_dir / robot_name / 'config.yaml'}")
    click.echo()
    click.echo("📝 Next steps:")
    click.echo(f"  1. altrus-cli configure {robot_name}  # Customize settings")
    click.echo(f"  2. altrus-cli build {robot_name}       # Generate ROS2 packages")
    click.echo(f"  3. altrus-cli launch {robot_name}      # Start simulation")


@cli.command()
@click.argument("robot_name")
@click.option("--gui", is_flag=True, help="Open GUI configurator")
def configure(robot_name, gui):
    """Interactively configure robot parameters"""
    manager = ConfigManager()
    config = manager.load_config(robot_name)

    if not config:
        click.echo(f"❌ Robot '{robot_name}' not found")
        click.echo(f"Run: altrus-cli init {robot_name}")
        return

    if gui:
        click.echo("🖥️  Opening GUI configurator...")
        click.echo("⚠️  GUI not yet implemented. Use CLI for now.")
        return

    click.echo(f"\n⚙️  Configuring: {robot_name}")
    click.echo("=" * 60)

    # Dimensions
    if click.confirm("\n📏 Configure robot dimensions?", default=False):
        dims = config.config["dimensions"]
        dims["length"] = click.prompt("  Length (m)", default=dims["length"], type=float)
        dims["width"] = click.prompt("  Width (m)", default=dims["width"], type=float)
        dims["height"] = click.prompt("  Height (m)", default=dims["height"], type=float)

        if config.config["base_type"] == "wheeled":
            dims["wheel_radius"] = click.prompt("  Wheel radius (m)", default=dims["wheel_radius"], type=float)
            dims["wheel_width"] = click.prompt("  Wheel width (m)", default=dims["wheel_width"], type=float)

        if config.config["base_type"] == "tracked":
            dims["track_length"] = click.prompt("  Track length (m)", default=dims.get("track_length", 0.35), type=float)
            dims["track_width"] = click.prompt("  Track width (m)", default=dims.get("track_width", 0.06), type=float)
            dims["track_height"] = click.prompt("  Track height (m)", default=dims.get("track_height", 0.06), type=float)
            dims["track_separation"] = click.prompt(
                "  Track separation (m)", default=dims.get("track_separation", 0.28), type=float
            )
            dims["drive_wheel_radius"] = click.prompt(
                "  Drive wheel radius (m)", default=dims.get("drive_wheel_radius", 0.05), type=float
            )
            dims["drive_wheel_width"] = click.prompt(
                "  Drive wheel width (m)", default=dims.get("drive_wheel_width", 0.02), type=float
            )

    # Sensors
    if click.confirm("\n📷 Configure sensors?", default=False):
        sensors = config.config["sensors"]
        sensors["camera"]["enabled"] = click.confirm("  Enable camera?", default=sensors["camera"]["enabled"])
        sensors["imu"]["enabled"] = click.confirm("  Enable IMU?", default=sensors["imu"]["enabled"])

        if click.confirm("  Configure LIDAR settings?", default=False):
            lidar = sensors["lidar"]
            lidar["max_range"] = click.prompt("    Max range (m)", default=lidar["max_range"], type=float)
            lidar["update_rate"] = click.prompt("    Update rate (Hz)", default=lidar["update_rate"], type=float)

    # Navigation (planner selection)
    if click.confirm("\n🗺️  Configure navigation?", default=False):
        nav = config.config["navigation"]

        nav["planner"] = click.prompt(
            "  Global planner", type=click.Choice(["navfn", "smac_2d"]), default=(nav.get("planner", "navfn") or "navfn")
        )

        if nav["planner"] == "navfn":
            nav["use_astar"] = click.confirm("  Use A* (NavFn)?", default=bool(nav.get("use_astar", False)))
        else:
            nav["use_astar"] = True  # SMAC is A*-based

        nav["max_linear_velocity"] = click.prompt(
            "  Max linear velocity (m/s)", default=nav["max_linear_velocity"], type=float
        )
        nav["max_angular_velocity"] = click.prompt(
            "  Max angular velocity (rad/s)", default=nav["max_angular_velocity"], type=float
        )
        nav["obstacle_padding"] = click.prompt("  Obstacle padding (m)", default=nav["obstacle_padding"], type=float)

    # Multimodal
    if click.confirm("\n🎤 Configure voice/gesture control?", default=False):
        mm = config.config["multimodal"]
        mm["voice"]["enabled"] = click.confirm("  Enable voice commands?", default=mm["voice"]["enabled"])
        mm["gesture"]["enabled"] = click.confirm("  Enable gesture control?", default=mm["gesture"]["enabled"])

        if mm["voice"]["enabled"]:
            mm["voice"]["language"] = click.prompt("  Voice language", default=mm["voice"]["language"])

        if mm["gesture"]["enabled"] and not config.config["sensors"]["camera"]["enabled"]:
            click.echo("  ⚠️  Gesture control requires camera. Enabling camera...")
            config.config["sensors"]["camera"]["enabled"] = True

    # Recalculate derived values
    config._calculate_derived_values()

    # Validate
    is_valid, errors = config.validate()
    if not is_valid:
        click.echo("\n❌ Configuration errors:")
        for error in errors:
            click.echo(f"  - {error}")
        if not click.confirm("\nSave anyway?", default=False):
            return

    # Save
    manager.save_config(config)
    click.echo("\n✅ Configuration saved")



