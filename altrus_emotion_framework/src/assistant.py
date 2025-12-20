import time
import queue
import threading
import random
import numpy as np
import cv2
import sounddevice as sd
import tensorflow as tf
import librosa

from .config import AppConfig
from .fusion import fuse_probs
from .prompt import build_prompt, label_to_text
from .utils.ffmpeg import ensure_ffmpeg_available

HAPPY_RESPONSES = [
    "You seem happy today, that's wonderful!",
    "I'm glad you're feeling good!",
    "You sound cheerful. Nice to see that!",
]
SAD_RESPONSES = [
    "You seem a bit down. I'm here for you.",
    "I'm sorry you're feeling sad. You're not alone.",
    "It's okay to feel sad sometimes. I'm here for you.",
]

def fallback_response(label: int) -> str:
    return random.choice(HAPPY_RESPONSES) if label == 0 else random.choice(SAD_RESPONSES)

class MultimodalAssistant:
    def __init__(self, cfg: AppConfig, llm, asr, tts):
        self.cfg = cfg
        self.llm = llm
        self.asr = asr
        self.tts = tts

        self.stop_event = threading.Event()
        self.robot_busy_event = threading.Event()
        self.utterance_queue: "queue.Queue[tuple[np.ndarray, np.ndarray | None]]" = queue.Queue()

        self.state_lock = threading.Lock()
        self.is_talking = False
        self.face_probs_buffer = []
        self.last_face_box = None
        self.last_face_label = None

        ensure_ffmpeg_available(cfg.FFMPEG_EXE)

        print("Loading models...")
        self.face_model = tf.keras.models.load_model(str(cfg.FACE_MODEL_PATH))
        self.voice_model = tf.keras.models.load_model(str(cfg.VOICE_MODEL_PATH))
        print("Models loaded.\n")

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    def _get_face_roi(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(60, 60))
        if len(faces) == 0:
            return None, None

        x, y, w, h = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)[0]
        face_gray = gray[y : y + h, x : x + w]
        face_resized = cv2.resize(face_gray, (48, 48))

        face_norm = face_resized.astype("float32") / 255.0
        face_norm = np.expand_dims(face_norm, axis=-1)
        face_norm = np.expand_dims(face_norm, axis=0)
        return face_norm, (x, y, w, h)

    def _predict_face_probs(self, frame):
        face_input, box = self._get_face_roi(frame)
        if face_input is None:
            return None, None, None
        preds = self.face_model.predict(face_input, verbose=0)
        probs = preds[0]
        label = int(np.argmax(probs))
        return probs, box, label

    def _extract_mfcc(self, y):
        mfcc = librosa.feature.mfcc(y=y, sr=self.cfg.SR, n_mfcc=self.cfg.N_MFCC)
        mfcc = (mfcc - np.mean(mfcc)) / (np.std(mfcc) + 1e-9)

        if mfcc.shape[1] < self.cfg.MAX_LEN:
            pad_width = self.cfg.MAX_LEN - mfcc.shape[1]
            mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)), mode="constant")
        else:
            mfcc = mfcc[:, : self.cfg.MAX_LEN]

        return mfcc[np.newaxis, ..., np.newaxis]

    def _predict_voice_emotion(self, y):
        mfcc_input = self._extract_mfcc(y)
        preds = self.voice_model.predict(mfcc_input, verbose=0)
        probs = preds[0]
        label = int(np.argmax(probs))
        return probs, label

    def camera_loop(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: could not open webcam.")
            self.stop_event.set()
            return

        frame_count = 0
        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            do_predict = (frame_count % 3 == 0)

            probs, box, label = (None, None, None)
            if do_predict:
                probs, box, label = self._predict_face_probs(frame)

            with self.state_lock:
                if probs is not None:
                    self.last_face_box = box
                    self.last_face_label = label
                    if self.is_talking:
                        self.face_probs_buffer.append(probs)

                box_draw = self.last_face_box
                label_draw = self.last_face_label

            cv2.putText(frame, "Talk normally. Press 'q' to quit.", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if box_draw is not None:
                x, y, w, h = box_draw
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                if label_draw is not None:
                    cv2.putText(frame, f"Face: {label_to_text(label_draw, self.cfg.LABELS)}",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Multimodal Assistant (Ollama)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stop_event.set()
                break

        cap.release()
        cv2.destroyAllWindows()
        self.stop_event.set()

    def audio_loop(self):
        blocksize = int(self.cfg.BLOCK_DUR * self.cfg.SR)
        silence_time = 0.0
        speech_start_time = None
        speech_chunks = []

        def callback(indata, frames, time_info, status):
            nonlocal silence_time, speech_start_time, speech_chunks

            if self.stop_event.is_set():
                raise sd.CallbackStop()

            if self.tts.speaking_event.is_set() or self.robot_busy_event.is_set():
                silence_time = 0.0
                speech_start_time = None
                speech_chunks.clear()
                with self.state_lock:
                    self.is_talking = False
                    self.face_probs_buffer = []
                return

            audio = indata[:, 0].copy()
            rms = np.sqrt(np.mean(audio ** 2))

            if rms > self.cfg.VAD_THRESHOLD:
                speech_chunks.append(audio)

                with self.state_lock:
                    if not self.is_talking:
                        self.is_talking = True
                        self.face_probs_buffer = []

                if speech_start_time is None:
                    speech_start_time = time.time()
                    print("\n[INFO] Speech detected...")
                silence_time = 0.0
            else:
                if speech_start_time is not None:
                    silence_time += self.cfg.BLOCK_DUR

                    if silence_time >= self.cfg.SILENCE_LIMIT:
                        duration = time.time() - speech_start_time

                        with self.state_lock:
                            self.is_talking = False
                            face_copy = np.array(self.face_probs_buffer) if len(self.face_probs_buffer) > 0 else None
                            self.face_probs_buffer = []

                        if duration >= self.cfg.MIN_UTTERANCE and len(speech_chunks) > 0:
                            full_signal = np.concatenate(speech_chunks)
                            self.utterance_queue.put((full_signal, face_copy))
                            print(f"[INFO] Utterance finished (~{duration:.2f}s).")
                        else:
                            print("[INFO] Short sound ignored (too brief).")

                        speech_chunks = []
                        speech_start_time = None
                        silence_time = 0.0

        with sd.InputStream(samplerate=self.cfg.SR, channels=1, blocksize=blocksize, callback=callback):
            while not self.stop_event.is_set():
                time.sleep(0.05)

    def run(self):
        print("Multimodal Emotion Framework")
        print("Talk normally. When you stop, it will respond.")
        print("Press 'q' in the video window to quit.\n")

        t_cam = threading.Thread(target=self.camera_loop, daemon=True)
        t_aud = threading.Thread(target=self.audio_loop, daemon=True)
        t_cam.start()
        t_aud.start()

        while not self.stop_event.is_set():
            try:
                full_signal, face_probs_segment = self.utterance_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self.robot_busy_event.set()
            try:
                voice_probs, voice_label = self._predict_voice_emotion(full_signal)
                print(f"[VOICE] probs {list(self.cfg.LABELS)}: {voice_probs} => {label_to_text(voice_label, self.cfg.LABELS)}")

                if face_probs_segment is not None and len(face_probs_segment) > 0:
                    face_avg = np.mean(face_probs_segment, axis=0)
                    face_label = int(np.argmax(face_avg))
                    print(f"[FACE]  avg  {list(self.cfg.LABELS)}: {face_avg} => {label_to_text(face_label, self.cfg.LABELS)}")
                else:
                    face_avg = np.array([0.5, 0.5], dtype=np.float32)
                    print("[FACE]  no face during speech => neutral [0.5, 0.5]")

                fused_probs, fused_label = fuse_probs(face_avg, voice_probs, self.cfg.W_FACE, self.cfg.W_VOICE)
                print(f"[FUSED] probs {list(self.cfg.LABELS)}: {fused_probs} => {label_to_text(fused_label, self.cfg.LABELS)}")

                text = self.asr.transcribe(full_signal, self.cfg.SR) if self.asr else ""
                if text:
                    print(f"[USER TEXT] {text}")
                else:
                    print("[USER TEXT] (no transcription)")

                prompt = build_prompt(
                    user_text=text,
                    fused_label=fused_label,
                    fused_probs=fused_probs,
                    labels=self.cfg.LABELS,
                )

                try:
                    reply = self.llm.generate(prompt)
                    if not reply:
                        reply = fallback_response(fused_label)
                except Exception:
                    print("[WARN] LLM call failed. Using fallback.")
                    reply = fallback_response(fused_label)

            finally:
                self.robot_busy_event.clear()

            self.tts.speak_async(reply)

        print("\n[INFO] Exiting...")
