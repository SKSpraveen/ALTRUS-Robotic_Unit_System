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


@cli.command()
@click.argument("robot_name")
@click.option("--location-name", help="Location name (e.g., kitchen)")
@click.option("--x", type=float, help="X coordinate")
@click.option("--y", type=float, help="Y coordinate")
@click.option("--yaw", default=0.0, type=float, help="Orientation (radians)")
@click.option("--interactive", is_flag=True, help="Interactive mode")
def add_location(robot_name, location_name, x, y, yaw, interactive):
    """Add a named location for voice navigation"""
    manager = ConfigManager()
    config = manager.load_config(robot_name)

    if not config:
        click.echo(f"❌ Robot '{robot_name}' not found")
        return

    if interactive or not all([location_name, x is not None, y is not None]):
        location_name = click.prompt("Location name")
        x = click.prompt("X coordinate", type=float)
        y = click.prompt("Y coordinate", type=float)
        yaw = click.prompt("Yaw (radians)", default=0.0, type=float)

    config.config["multimodal"]["locations"][location_name] = {"x": x, "y": y, "yaw": yaw}

    manager.save_config(config)
    click.echo(f"✅ Added location: {location_name} at ({x}, {y}, {yaw})")


@cli.command()
@click.argument("robot_name")
@click.option("--sensor", type=click.Choice(["camera", "imu", "depth_camera"]))
@click.option("--model", help="Sensor model/driver")
def add_sensor(robot_name, sensor, model):
    """Add a sensor to the robot configuration"""
    manager = ConfigManager()
    config = manager.load_config(robot_name)

    if not config:
        click.echo(f"❌ Robot '{robot_name}' not found")
        return

    if not sensor:
        sensor = click.prompt("Sensor type", type=click.Choice(["camera", "imu", "depth_camera"]))

    config.config["sensors"][sensor]["enabled"] = True
    if model:
        config.config["sensors"][sensor]["model"] = model

    manager.save_config(config)
    click.echo(f"✅ Added sensor: {sensor}" + (f" ({model})" if model else ""))


@cli.command()
@click.argument("robot_name")
@click.option("--output-dir", type=click.Path(), help="Output directory")
@click.option("--skip-validation", is_flag=True, help="Skip config validation")
def build(robot_name, output_dir, skip_validation):
    """Generate ROS2 packages from robot configuration"""
    manager = ConfigManager()
    config = manager.load_config(robot_name)

    if not config:
        click.echo(f"❌ Robot '{robot_name}' not found")
        return

    # Validate
    if not skip_validation:
        is_valid, errors = config.validate()
        if not is_valid:
            click.echo("❌ Configuration errors:")
            for error in errors:
                click.echo(f"  - {error}")
            if not click.confirm("Build anyway?", default=False):
                return

    # Determine output directory
    if not output_dir:
        output_dir = Path.home() / "altrus_ws" / "src"
    else:
        output_dir = Path(output_dir).expanduser()

    output_dir.mkdir(parents=True, exist_ok=True)

    click.echo(f"🔨 Building packages for: {robot_name}")
    click.echo(f"📦 Output directory: {output_dir}")
    click.echo()

    pkg_names = config.get_package_names()

    # Generate description package
    desc_pkg = output_dir / pkg_names["description"]
    click.echo(f"📦 Generating {pkg_names['description']}...")
    _generate_description_package(config, desc_pkg)

    # Generate navigation package
    nav_pkg = output_dir / pkg_names["navigation"]
    click.echo(f"📦 Generating {pkg_names['navigation']}...")
    _generate_navigation_package(config, nav_pkg)

    # Generate gazebo package
    gz_pkg = output_dir / pkg_names["gazebo"]
    click.echo(f"📦 Generating {pkg_names['gazebo']}...")
    _generate_gazebo_package(config, gz_pkg)

    # Generate bringup package
    bringup_pkg = output_dir / pkg_names["bringup"]
    click.echo(f"📦 Generating {pkg_names['bringup']}...")
    _generate_bringup_package(config, bringup_pkg)

    # Voice package (if enabled)
    if config.get("multimodal.voice.enabled"):
        voice_pkg = output_dir / pkg_names["voice"]
        click.echo(f"📦 Generating {pkg_names['voice']}...")
        _generate_voice_package(config, voice_pkg)

    # Gesture package (if enabled)
    if config.get("multimodal.gesture.enabled"):
        gesture_pkg = output_dir / pkg_names["gesture"]
        click.echo(f"📦 Generating {pkg_names['gesture']}...")
        _generate_gesture_package(config, gesture_pkg)

    click.echo()
    click.echo("✅ Build complete!")
    click.echo()
    click.echo("📝 Next steps:")
    click.echo(f"  cd {output_dir.parent}")
    click.echo("  colcon build")
    click.echo("  source install/setup.bash")
    click.echo(f"  altrus-cli launch {robot_name}")


@cli.command()
@click.argument("robot_name")
@click.option("--mode", type=click.Choice(["simulation", "real"]), default="simulation")
@click.option("--slam", is_flag=True, help="Launch with SLAM")
def launch(robot_name, mode, slam):
    """Launch the robot"""
    manager = ConfigManager()
    config = manager.load_config(robot_name)

    if not config:
        click.echo(f"❌ Robot '{robot_name}' not found")
        return

    pkg_name = config.get_package_names()["bringup"]

    if mode == "simulation":
        launch_file = "simulation_slam.launch.py" if slam else "simulation.launch.py"
    else:
        launch_file = "real_robot.launch.py"

    cmd = ["ros2", "launch", pkg_name, launch_file]

    click.echo(f"🚀 Launching: {robot_name} ({'SLAM' if slam else 'Navigation'} mode)")
    click.echo(f"📦 Package: {pkg_name}")
    click.echo(f"🎯 Launch file: {launch_file}")
    click.echo()
    click.echo(f"Running: {' '.join(cmd)}")
    click.echo()

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        click.echo("\n❌ Launch failed. Make sure you've built the packages:")
        click.echo("  cd ~/altrus_ws && colcon build")
    except FileNotFoundError:
        click.echo("\n❌ ROS2 not found. Make sure ROS2 is installed and sourced.")


@cli.command()
def list():
    """List all configured robots"""
    manager = ConfigManager()
    robots = manager.list_configs()

    if not robots:
        click.echo("No robots configured yet.")
        click.echo("\nGet started:")
        click.echo("  altrus-cli init my_robot --base wheeled")
        return

    click.echo("\n🤖 Configured Robots:")
    click.echo("=" * 60)

    for robot_name in robots:
        config = manager.load_config(robot_name)
        if config:
            cfg = config.config
            click.echo(f"\n📦 {robot_name}")
            click.echo(f"   Base: {cfg['base_type']}")
            click.echo(f"   Dimensions: {cfg['dimensions']['length']}m × {cfg['dimensions']['width']}m")
            click.echo(f"   Planner: {cfg['navigation'].get('planner','navfn')} (A*: {cfg['navigation'].get('use_astar', False)})")
            click.echo(f"   Voice: {'✓' if cfg['multimodal']['voice']['enabled'] else '✗'}")
            click.echo(f"   Gesture: {'✓' if cfg['multimodal']['gesture']['enabled'] else '✗'}")
            sensors = [s for s, v in cfg["sensors"].items() if v["enabled"]]
            click.echo(f"   Sensors: {', '.join(sensors)}")



