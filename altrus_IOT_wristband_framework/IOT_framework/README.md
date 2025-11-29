# Altrus ML Model V2 CLI

This repository provides a Python CLI that scaffolds standalone wristband sensor fusion
projects for anomaly detection and activity classification. The CLI gathers sensor,
anomaly, and activity selections, then creates a ready-to-edit project folder with
configuration, simulation stubs, and model hooks.

## Installation

```bash
pip install -e .
```

## Usage

```bash
altrus init
```

Follow the prompts to pick sensors, anomalies, activities, target environments, and an
output location. A YAML configuration file will be generated so teams can update
selections later without rebuilding from scratch.

To run a live scanner inside a generated project:

```bash
cd <project-name>
altrus run
```

## Training workspace

Use `training_workspace/` to generate demo `.pkl` files for the default activity and
anomaly models. Copy the outputs into your generated project `models/` directory.

```bash
python training_workspace/train_models.py
```
