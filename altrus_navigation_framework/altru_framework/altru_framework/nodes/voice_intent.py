#!/usr/bin/env python3
import json
import queue
import re
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceIntent(Node):
    """
    Publishes voice intents to /voice/command as std_msgs/String.

    Supported intents:
      - "go"
      - "come"
      - "stop"

    Modes:
      engine = "simulated"  -> (NOT reliable under ros2 launch) reads typed commands
      engine = "vosk"       -> offline speech recognition (vosk + sounddevice)
    """

    def __init__(self):
        super().__init__('altrus_voice_intent')

        self.declare_parameter('engine', 'simulated')  # simulated | vosk
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('out_topic', '/voice/command')

        # vosk parameters
        self.declare_parameter('vosk_model_path', '')
        # Use -1 for default device (avoids rclpy "None type" warning)
        self.declare_parameter('device', -1)
        self.declare_parameter('sample_rate', 16000)

        self.engine = str(self.get_parameter('engine').value)
        self.out_topic = str(self.get_parameter('out_topic').value)

        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.get_logger().info(f"VoiceIntent engine={self.engine} publishing to {self.out_topic}")

        self._stop_flag = False

        if self.engine == 'simulated':
            self.thread = threading.Thread(target=self._typed_loop, daemon=True)
            self.thread.start()
        elif self.engine == 'vosk':
            self.thread = threading.Thread(target=self._vosk_loop, daemon=True)
            self.thread.start()
        else:
            self.get_logger().error("Unknown engine. Use 'simulated' or 'vosk'.")

    def publish_intent(self, text: str):
        text = (text or '').strip().lower()
        if not text:
            return

        # Normalize to intents
        intent = self._normalize(text)
        if not intent:
            self.get_logger().info(f"Ignored voice text: {text}")
            return

        msg = String()
        msg.data = intent
        self.pub.publish(msg)
        self.get_logger().info(f"Published voice intent: {intent}")

    def _normalize(self, text: str) -> Optional[str]:
        t = text.lower()
        if re.search(r'\b(stop|halt|freeze)\b', t):
            return 'stop'
        if re.search(r'\b(come|forward|closer)\b', t):
            return 'come'
        if re.search(r'\b(go|back|away|return)\b', t):
            return 'go'
        # allow exact commands
        if t in ('go', 'come', 'stop'):
            return t
        return None

    def _typed_loop(self):
        # NOTE: input() is often NOT usable when launched with ros2 launch
        self.get_logger().info("Typed voice mode: type (go/come/stop) then Enter.")
        self.get_logger().warn("If typing doesn't work under ros2 launch, use: ros2 topic pub --once /voice/command ...")
        while rclpy.ok() and not self._stop_flag:
            try:
                line = input("> ")
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            self.publish_intent(line)

    def _vosk_loop(self):
        try:
            from vosk import Model, KaldiRecognizer
            import sounddevice as sd
        except Exception as e:
            self.get_logger().error("Vosk mode requires: pip3 install vosk sounddevice")
            self.get_logger().error(str(e))
            return

        model_path = str(self.get_parameter('vosk_model_path').value).strip()
        if not model_path:
            self.get_logger().error("vosk_model_path is empty. Download a Vosk model and set this param.")
            return

        device_param = int(self.get_parameter('device').value)
        device = None if device_param < 0 else device_param
        sample_rate = int(self.get_parameter('sample_rate').value)

        self.get_logger().info(f"Loading Vosk model from: {model_path}")
        model = Model(model_path)
        rec = KaldiRecognizer(model, sample_rate)
        rec.SetWords(True)

        audio_q: "queue.Queue[bytes]" = queue.Queue(maxsize=50)

        def callback(indata, frames, time_info, status):
            if status:
                self.get_logger().warn(str(status))
            try:
                audio_q.put_nowait(bytes(indata))
            except queue.Full:
                # Drop audio if system lags
                pass

        self.get_logger().info("Listening on microphone... say: go / come / stop")

        try:
            with sd.RawInputStream(
                samplerate=sample_rate,
                blocksize=8000,
                dtype='int16',
                channels=1,
                callback=callback,
                device=device
            ):
                while rclpy.ok() and not self._stop_flag:
                    try:
                        data = audio_q.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if rec.AcceptWaveform(data):
                        result = json.loads(rec.Result())
                        text = (result.get('text') or '').strip()
                        if text:
                            self.get_logger().info(f"Heard: {text}")
                            self.publish_intent(text)

        except Exception as e:
            self.get_logger().error(f"Mic error: {e}")

    def destroy_node(self):
        self._stop_flag = True
        super().destroy_node()


def main():
    rclpy.init()
    node = VoiceIntent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

