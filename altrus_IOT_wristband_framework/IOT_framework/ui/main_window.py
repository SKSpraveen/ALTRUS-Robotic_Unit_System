from __future__ import annotations

import pathlib
import shutil
from dataclasses import asdict

from PySide6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

from app.core.config import ProjectConfig
from app.core.generation import Generator, PreviewResult
from app.core.operations import IdfRunner
from app.displays.registry import DisplayRegistry
from app.sensors.registry import SensorRegistry
from app.utils.worker import WorkerThread


class MainWindow(QtWidgets.QMainWindow):
    _ESP32_USB_IDS = {
        (0x10C4, 0xEA60),  # Silicon Labs CP210x
        (0x1A86, 0x7523),  # QinHeng CH340
        (0x1A86, 0x55D4),  # QinHeng CH9102
        (0x0403, 0x6001),  # FTDI FT232
        (0x303A, 0x1001),  # Espressif USB JTAG/serial
    }
    _ESP32_DESCRIPTION_HINTS = (
        "cp210",
        "ch340",
        "ch910",
        "ftdi",
        "usb serial",
        "usb-serial",
        "espressif",
        "esp32",
    )

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP32 Wristband Project Generator")
        self.resize(1200, 800)

        self.registry = SensorRegistry.load_default()
        self.display_registry = DisplayRegistry.load_default()
        self.generator = Generator()
        self.idf_runner = IdfRunner()
        self.sensor_settings: dict[str, dict[str, str]] = {}
        self.workers: list[WorkerThread] = []

        self._setup_ui()
        self._load_sensors()

    def _setup_ui(self) -> None:
        central = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(central)

        form_layout = QtWidgets.QFormLayout()

        self.project_path_edit = QtWidgets.QLineEdit()
        self.project_path_edit.setPlaceholderText("Select output folder for ESP-IDF project...")
        self.browse_button = QtWidgets.QPushButton("Browse")
        self.browse_button.clicked.connect(self._browse_project)
        path_layout = QtWidgets.QHBoxLayout()
        path_layout.addWidget(self.project_path_edit)
        path_layout.addWidget(self.browse_button)
        form_layout.addRow("Project Folder", path_layout)

        self.transport_combo = QtWidgets.QComboBox()
        self.transport_combo.addItems(["BLE", "Wi-Fi"])
        form_layout.addRow("Transport", self.transport_combo)

        self.sample_rate_spin = QtWidgets.QSpinBox()
        self.sample_rate_spin.setRange(1, 2000)
        self.sample_rate_spin.setValue(50)
        form_layout.addRow("Default Sample Rate (Hz)", self.sample_rate_spin)

        main_layout.addLayout(form_layout)

        self.top_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.main_splitter.addWidget(self.top_splitter)
        self.main_splitter.addWidget(self.bottom_splitter)
        self.main_splitter.setStretchFactor(0, 2)
        self.main_splitter.setStretchFactor(1, 3)

        self._build_top_left()
        self._build_top_right()
        self._build_bottom()

        main_layout.addWidget(self.main_splitter, stretch=1)

        self.log_output = QtWidgets.QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setPlaceholderText("Logs and preview output...")
        dock = QtWidgets.QDockWidget("Logs / Preview", self)
        dock.setObjectName("logs_dock")
        dock.setWidget(self.log_output)
        dock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)

        button_layout = QtWidgets.QHBoxLayout()
        self.preview_button = QtWidgets.QPushButton("Preview")
        self.generate_button = QtWidgets.QPushButton("Generate")
        self.build_button = QtWidgets.QPushButton("Build + Upload")

        self.preview_button.clicked.connect(self._preview)
        self.generate_button.clicked.connect(self._generate)
        self.build_button.clicked.connect(self._build_and_upload)

        button_layout.addStretch(1)
        button_layout.addWidget(self.preview_button)
        button_layout.addWidget(self.generate_button)
        button_layout.addWidget(self.build_button)
        button_layout.addStretch(1)
        main_layout.addLayout(button_layout)

        self.setCentralWidget(central)

    def _build_top_left(self) -> None:
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)

        display_group = QtWidgets.QGroupBox("Display Selection")
        display_layout = QtWidgets.QFormLayout(display_group)
        display_label = QtWidgets.QLabel("Display")
        self.display_combo = QtWidgets.QComboBox()
        self.display_combo.addItem("No display", None)
        for display in self.display_registry.displays:
            self.display_combo.addItem(display.display_name, display.key)
        display_layout.addRow(display_label, self.display_combo)

        self.sensor_search = QtWidgets.QLineEdit()
        self.sensor_search.setPlaceholderText("Search sensors...")
        self.sensor_search.textChanged.connect(self._filter_sensors)

        self.sensor_tree = QtWidgets.QTreeWidget()
        self.sensor_tree.setHeaderLabel("Sensors")
        self.sensor_tree.itemChanged.connect(self._sensor_checked)
        self.sensor_tree.itemSelectionChanged.connect(self._sensor_selected)

        left_layout.addWidget(display_group)
        left_layout.addWidget(self.sensor_search)
        left_layout.addWidget(self.sensor_tree)
        left_layout.addStretch(1)
        self.top_splitter.addWidget(left_widget)

    def _build_top_right(self) -> None:
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)

        esp32_group = QtWidgets.QGroupBox("ESP32 Selection")
        esp32_layout = QtWidgets.QFormLayout(esp32_group)
        self.esp32_combo = QtWidgets.QComboBox()
        self.esp32_combo.currentIndexChanged.connect(self._update_esp32_status)
        self.esp32_status = QtWidgets.QLabel("No board detected.")
        self.esp32_detect_button = QtWidgets.QPushButton("Detect")
        self.esp32_detect_button.clicked.connect(self._detect_esp32)
        esp32_layout.addRow("Board", self.esp32_combo)
        esp32_layout.addRow(self.esp32_detect_button, self.esp32_status)
        right_layout.addWidget(esp32_group)

        title = QtWidgets.QLabel("Sensor Variables")
        title.setStyleSheet("font-weight: 600;")
        right_layout.addWidget(title)

        self.sensor_form = QtWidgets.QFormLayout()
        self.sensor_form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        right_layout.addLayout(self.sensor_form)
        right_layout.addStretch(1)

        self.top_splitter.addWidget(right_widget)
        self.top_splitter.setStretchFactor(0, 2)
        self.top_splitter.setStretchFactor(1, 3)

    def _build_bottom(self) -> None:
        tree_widget = QtWidgets.QWidget()
        tree_layout = QtWidgets.QVBoxLayout(tree_widget)
        tree_label = QtWidgets.QLabel("Project Structure")
        tree_label.setStyleSheet("font-weight: 600;")
        self.project_tree = QtWidgets.QTreeWidget()
        self.project_tree.setHeaderLabel("Files")
        self.project_tree.itemSelectionChanged.connect(self._load_selected_file)
        tree_layout.addWidget(tree_label)
        tree_layout.addWidget(self.project_tree)

        editor_widget = QtWidgets.QWidget()
        editor_layout = QtWidgets.QVBoxLayout(editor_widget)
        editor_label = QtWidgets.QLabel("Code Editor")
        editor_label.setStyleSheet("font-weight: 600;")
        self.code_editor = QtWidgets.QPlainTextEdit()
        self.code_editor.setPlaceholderText("Select a file to edit...")
        self.code_editor.setFont(QtGui.QFont("Consolas", 10))
        editor_layout.addWidget(editor_label)
        editor_layout.addWidget(self.code_editor)

        self.bottom_splitter.addWidget(tree_widget)
        self.bottom_splitter.addWidget(editor_widget)
        self.bottom_splitter.setStretchFactor(0, 1)
        self.bottom_splitter.setStretchFactor(1, 3)

    def _load_sensors(self) -> None:
        self.sensor_tree.clear()
        categories: dict[str, QtWidgets.QTreeWidgetItem] = {}

        for sensor in self.registry.sensors:
            category = sensor.definition.category or "Other"
            if category not in categories:
                category_item = QtWidgets.QTreeWidgetItem([category])
                category_item.setFlags(category_item.flags() & ~QtCore.Qt.ItemIsSelectable)
                categories[category] = category_item
                self.sensor_tree.addTopLevelItem(category_item)
            else:
                category_item = categories[category]

            sensor_item = QtWidgets.QTreeWidgetItem([sensor.display_name])
            sensor_item.setFlags(sensor_item.flags() | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsSelectable)
            sensor_item.setCheckState(0, QtCore.Qt.Unchecked)
            sensor_item.setData(0, QtCore.Qt.UserRole, sensor.key)
            category_item.addChild(sensor_item)
            category_item.setExpanded(True)

    def _filter_sensors(self, text: str) -> None:
        text = text.lower().strip()
        for index in range(self.sensor_tree.topLevelItemCount()):
            category_item = self.sensor_tree.topLevelItem(index)
            category_visible = False
            for child_index in range(category_item.childCount()):
                child = category_item.child(child_index)
                matches = text in child.text(0).lower()
                child.setHidden(not matches)
                category_visible = category_visible or matches
            category_item.setHidden(not category_visible if text else False)

    def _sensor_checked(self, item: QtWidgets.QTreeWidgetItem, _column: int) -> None:
        if item.childCount() > 0:
            return
        key = item.data(0, QtCore.Qt.UserRole)
        if not key:
            return
        if item.checkState(0) == QtCore.Qt.Checked:
            if key not in self.sensor_settings:
                sensor = self.registry.get(key)
                default_rate = sensor.definition.default_sample_rate_hz or self.sample_rate_spin.value()
                self.sensor_settings[key] = {
                    "sample_rate_hz": str(default_rate),
                    **{pin: str(value) for pin, value in sensor.definition.default_pins.items()},
                }
        else:
            self.sensor_settings.pop(key, None)

    def _sensor_selected(self) -> None:
        selected_items = self.sensor_tree.selectedItems()
        if not selected_items:
            return
        item = selected_items[0]
        if item.childCount() > 0:
            return
        sensor_key = item.data(0, QtCore.Qt.UserRole)
        if not sensor_key:
            return
        self._render_sensor_form(sensor_key)

    def _render_sensor_form(self, sensor_key: str) -> None:
        while self.sensor_form.rowCount():
            self.sensor_form.removeRow(0)

        sensor = self.registry.get(sensor_key)
        values = self.sensor_settings.get(sensor_key, {})
        self.sensor_form.addRow("Sensor", QtWidgets.QLabel(sensor.display_name))
        self.sensor_form.addRow("Bus", QtWidgets.QLabel(sensor.definition.bus))

        rate_edit = QtWidgets.QLineEdit(values.get("sample_rate_hz", str(self.sample_rate_spin.value())))
        rate_edit.textChanged.connect(lambda value, key=sensor_key: self._update_setting(key, "sample_rate_hz", value))
        self.sensor_form.addRow("Sample Rate (Hz)", rate_edit)

        for pin_name, default_value in sensor.definition.default_pins.items():
            pin_edit = QtWidgets.QLineEdit(values.get(pin_name, str(default_value)))
            pin_edit.textChanged.connect(lambda value, key=sensor_key, name=pin_name: self._update_setting(key, name, value))
            self.sensor_form.addRow(pin_name.upper(), pin_edit)

    def _update_setting(self, sensor_key: str, field: str, value: str) -> None:
        if sensor_key not in self.sensor_settings:
            self.sensor_settings[sensor_key] = {}
        self.sensor_settings[sensor_key][field] = value

    def _browse_project(self) -> None:
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Project Folder")
        if folder:
            self.project_path_edit.setText(folder)

    def _resolve_project_path(self) -> pathlib.Path | None:
        raw_text = self.project_path_edit.text().strip()
        if not raw_text:
            return None
        raw_path = pathlib.Path(raw_text)
        if (raw_path / "app" / "main.py").exists():
            return raw_path / (raw_path.name + "_esp32")
        return raw_path

    def _collect_config(self) -> ProjectConfig:
        selected = []
        for index in range(self.sensor_tree.topLevelItemCount()):
            category_item = self.sensor_tree.topLevelItem(index)
            for child_index in range(category_item.childCount()):
                item = category_item.child(child_index)
                if item.checkState(0) == QtCore.Qt.Checked:
                    selected.append(item.data(0, QtCore.Qt.UserRole))

        project_path = self._resolve_project_path() or pathlib.Path()
        return ProjectConfig(
            project_name=project_path.name if project_path.name else "esp32_wristband",
            project_path=project_path,
            transport=self.transport_combo.currentText().lower(),
            sample_rate_hz=self.sample_rate_spin.value(),
            display=self.display_combo.currentData(),
            sensors=selected,
            sensor_settings={key: self.sensor_settings[key] for key in selected if key in self.sensor_settings},
        )

    def _append_preview(self, text: str) -> None:
        self.log_output.appendPlainText(text)

    def _preview(self) -> None:
        self.log_output.clear()
        resolved_path = self._resolve_project_path()
        if not resolved_path:
            self._append_preview("Please select a project folder.")
            return
        config = self._collect_config()
        config.project_path = resolved_path

        try:
            preview = self.generator.preview(config, self.registry)
        except Exception as exc:  # noqa: BLE001 - UI boundary
            self._append_preview(f"Preview failed: {exc}")
            return

        self._render_preview(preview)

    def _render_preview(self, preview: PreviewResult) -> None:
        self._append_preview("Files to be generated/updated:")
        for path in preview.files:
            self._append_preview(f"- {path}")

        if preview.diff:
            self._append_preview("\nUnified diff:\n")
            self._append_preview(preview.diff)
        else:
            self._append_preview("\nNo changes detected.")

    def _generate(self) -> None:
        self.log_output.clear()
        resolved_path = self._resolve_project_path()
        if not resolved_path:
            self._append_preview("Please select a project folder.")
            return
        config = self._collect_config()
        config.project_path = resolved_path

        worker = WorkerThread(self._generate_project, config)
        worker.signals.finished.connect(lambda: self._on_generate_finished(config.project_path, worker))
        worker.signals.error.connect(lambda message: self._append_preview(message))
        worker.signals.log.connect(self._append_preview)
        worker.finished.connect(lambda: self._cleanup_worker(worker))
        self._track_worker(worker)

    def _generate_project(self, config: ProjectConfig, log: QtCore.SignalInstance) -> None:
        log.emit("Generating project...")
        self.generator.generate(config, self.registry, log.emit)
        log.emit("Project configuration saved.")

    def _build_and_upload(self) -> None:
        project_path = self._resolve_project_path()
        if not project_path:
            self._append_preview("Please select a project folder.")
            return
        if not self.idf_runner.is_available():
            self._append_preview("idf.py not found. Installing ESP-IDF in the background...")
            worker = WorkerThread(self._install_idf)
            worker.signals.log.connect(self._append_preview)
            worker.signals.error.connect(lambda message: self._append_preview(message))
            worker.signals.finished.connect(lambda: self._append_preview("ESP-IDF install task finished."))
            worker.finished.connect(lambda: self._cleanup_worker(worker))
            self._track_worker(worker)
            return
        port = self.esp32_combo.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, "ESP32", "No boards detected.")
            return
        self._run_idf("flash", project_path, port)

    def _run_idf(self, command: str, project_path: pathlib.Path, port: str | None = None) -> None:
        self.log_output.clear()
        worker = WorkerThread(self._idf_task, command, project_path, port)
        worker.signals.log.connect(self._append_preview)
        worker.signals.error.connect(lambda message: self._append_preview(message))
        worker.signals.finished.connect(lambda: self._append_preview(f"IDF {command} finished."))
        worker.finished.connect(lambda: self._cleanup_worker(worker))
        self._track_worker(worker)

    def _idf_task(self, command: str, project_path: pathlib.Path, port: str | None, log: QtCore.SignalInstance) -> None:
        for line in self.idf_runner.run(command, project_path, port):
            log.emit(line)

    def _install_idf(self, log: QtCore.SignalInstance) -> None:
        install_root = pathlib.Path.home() / ".esp-idf-tools"
        for line in self.idf_runner.install(install_root):
            log.emit(line)

    def _detect_esp32(self) -> None:
        self.esp32_combo.clear()
        all_ports = list_ports.comports()
        esp32_ports = [port for port in all_ports if self._is_esp32_port(port)]
        for port in all_ports:
            description = port.description or port.device
            if port in esp32_ports:
                display = f"ESP32: {description} ({port.device})"
            else:
                display = f"{description} ({port.device})"
            self.esp32_combo.addItem(display, port.device)
        if not all_ports:
            self.esp32_status.setText("No board detected.")
            QtWidgets.QMessageBox.information(self, "ESP32", "No boards detected.")
            return
        if esp32_ports:
            self.esp32_status.setText("ESP32 detected. Select a port to flash.")
        else:
            self.esp32_status.setText("Select the correct serial port to flash.")
        self._update_esp32_status()

    def _update_esp32_status(self) -> None:
        current = self.esp32_combo.currentText()
        if current:
            if current.lower().startswith("esp32:"):
                self.esp32_status.setText(f"ESP32 detected: {current[6:].strip()}")
            else:
                self.esp32_status.setText(f"Selected port: {current}")
        else:
            self.esp32_status.setText("No board detected.")

    def _is_esp32_port(self, port) -> bool:
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        if vid is not None and pid is not None:
            if (vid, pid) in self._ESP32_USB_IDS:
                return True
        description = (port.description or "").lower()
        return any(hint in description for hint in self._ESP32_DESCRIPTION_HINTS)

    def _restore_config(self, config: ProjectConfig) -> None:
        self.project_path_edit.setText(str(config.project_path))
        self.transport_combo.setCurrentText(config.transport.upper())
        self.sample_rate_spin.setValue(config.sample_rate_hz)
        self.sensor_settings = config.sensor_settings or {}
        if config.display:
            index = self.display_combo.findData(config.display)
            if index >= 0:
                self.display_combo.setCurrentIndex(index)
        selected = set(config.sensors)
        for index in range(self.sensor_tree.topLevelItemCount()):
            category_item = self.sensor_tree.topLevelItem(index)
            for child_index in range(category_item.childCount()):
                item = category_item.child(child_index)
                item.setCheckState(
                    0,
                    QtCore.Qt.Checked if item.data(0, QtCore.Qt.UserRole) in selected else QtCore.Qt.Unchecked,
                )

        self.log_output.appendPlainText("Loaded existing project_config.yaml")

    def showEvent(self, event) -> None:  # noqa: N802 - Qt naming
        project_path = self._resolve_project_path()
        if project_path:
            config_path = project_path / "project_config.yaml"
            if config_path.exists():
                config = self._collect_config()
                config.project_path = project_path
                config.update_from_file(config_path)
                self._restore_config(config)
                self._refresh_project_tree(project_path)
        super().showEvent(event)

    def _debug_config_dump(self) -> None:
        config = self._collect_config()
        self._append_preview(str(asdict(config)))

    def _track_worker(self, worker: WorkerThread) -> None:
        self.workers.append(worker)
        worker.start()

    def _cleanup_worker(self, worker: WorkerThread) -> None:
        if worker in self.workers:
            self.workers.remove(worker)

    def _on_generate_finished(self, project_path: pathlib.Path, worker: WorkerThread) -> None:
        self._append_preview("Generation complete.")
        self._refresh_project_tree(project_path)

    def _refresh_project_tree(self, project_path: pathlib.Path) -> None:
        self.project_tree.clear()
        if not project_path.exists():
            return
        root_item = QtWidgets.QTreeWidgetItem([project_path.name])
        root_item.setData(0, QtCore.Qt.UserRole, project_path)
        self.project_tree.addTopLevelItem(root_item)
        self._populate_tree(root_item, project_path)
        root_item.setExpanded(True)

    def _populate_tree(self, parent_item: QtWidgets.QTreeWidgetItem, path: pathlib.Path) -> None:
        for child in sorted(path.iterdir(), key=lambda p: (p.is_file(), p.name.lower())):
            if child.name.startswith("."):
                continue
            item = QtWidgets.QTreeWidgetItem([child.name])
            item.setData(0, QtCore.Qt.UserRole, child)
            parent_item.addChild(item)
            if child.is_dir():
                self._populate_tree(item, child)

    def _load_selected_file(self) -> None:
        selected = self.project_tree.selectedItems()
        if not selected:
            return
        item = selected[0]
        path = item.data(0, QtCore.Qt.UserRole)
        if not isinstance(path, pathlib.Path) or not path.is_file():
            return
        try:
            content = path.read_text(encoding="utf-8")
        except Exception as exc:  # noqa: BLE001 - UI boundary
            self._append_preview(f"Failed to open {path.name}: {exc}")
            return
        self.code_editor.setPlainText(content)
        self.code_editor.document().setModified(False)
