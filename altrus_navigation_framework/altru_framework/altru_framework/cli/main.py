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



