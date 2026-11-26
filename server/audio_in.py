"""
audio_in.py - Voice input, Voice Activity Detection (VAD), Wake Word, and Emergency STOP Spotter.
Provides push-to-talk, typed-text fallback, and STT pluggability.
Audio never blocks control loops: pushes structured events to orchestrator queue.
"""
import time
import queue
import threading
import tempfile
import os
from typing import Optional, Callable
from events import bus, Event, EventType
from intents import IntentParser, Intent

class AudioInput:
    MAX_AUDIO_BYTES = 15 * 1024 * 1024

    def __init__(self, parser: IntentParser, wake_word: str = "rover", wake_word_enabled: bool = False,
                 sim: bool = True, stt_backend: str = "faster-whisper", whisper_model: str = "tiny.en"):
        self.parser = parser
        self.wake_word = wake_word.lower()
        self.wake_word_enabled = wake_word_enabled
        self.sim = sim
        self.stt_backend = stt_backend
        self.whisper_model_name = whisper_model
        self._whisper_model = None
        self._stt_lock = threading.Lock()

        self.muted = False # Half-duplex muting when robot is speaking
        self.is_listening = False
        self.event_queue = queue.Queue()

        self._running = True

    def transcribe_audio(self, audio_bytes: bytes, content_type: str = "audio/webm") -> str:
        """Transcribe a short browser recording locally with Faster-Whisper."""
        if self.stt_backend != "faster-whisper":
            raise RuntimeError(f"Unsupported speech-to-text backend: {self.stt_backend}")
        if not audio_bytes:
            raise ValueError("No audio data was received.")
        if len(audio_bytes) > self.MAX_AUDIO_BYTES:
            raise ValueError("Audio recording is too large; keep push-to-talk clips under 15 MB.")

        media_type = content_type.split(";", maxsplit=1)[0].strip().lower()
        suffix = {
            "audio/mp4": ".mp4",
            "audio/m4a": ".m4a",
            "audio/wav": ".wav",
            "audio/x-wav": ".wav",
            "audio/ogg": ".ogg",
            "audio/mpeg": ".mp3",
        }.get(media_type, ".webm")
        with self._stt_lock:
            if self._whisper_model is None:
                try:
                    from faster_whisper import WhisperModel
                except ImportError as exc:
                    raise RuntimeError("Local transcription is not installed. Install project requirements first.") from exc
                self._whisper_model = WhisperModel(
                    self.whisper_model_name,
                    device="cpu",
                    compute_type="int8"
                )

            with tempfile.TemporaryDirectory() as temp_dir:
                audio_path = os.path.join(temp_dir, f"recording{suffix}")
                with open(audio_path, "wb") as audio_file:
                    audio_file.write(audio_bytes)
                # Telling Whisper which words to expect makes it hear medicine names much better
                vocab = ("Rover commands: fetch paracetamol, fetch cetirizine, fetch omeprazole, panadol, "
                         "move forward, move back, turn left, turn right, stop, go home, arm home, "
                         "open gripper, close gripper, centimeters, meters, degrees.")
                kwargs = dict(language="en", beam_size=3, vad_filter=True, initial_prompt=vocab,
                              condition_on_previous_text=False)
                try:
                    segments, _ = self._whisper_model.transcribe(
                        audio_path, hotwords="paracetamol cetirizine omeprazole forward backward left right stop", **kwargs)
                except TypeError:                     # older faster-whisper without hotwords
                    segments, _ = self._whisper_model.transcribe(audio_path, **kwargs)
                return " ".join(segment.text.strip() for segment in segments).strip()

    def set_muted(self, muted: bool):
        """Half-duplex: mute mic while TTS is speaking so robot does not hear itself."""
        self.muted = muted

    def process_text_command(self, text: str) -> Intent:
        """Typed command fallback or transcribed speech input."""
        # Commands are ALWAYS accepted (push-to-talk / typed), even while the robot is
        # talking - "stop" must never be ignored.

        t = text.strip()
        print(f"[AudioIn] Processing command: '{t}'")

        # Wake word check (if enabled)
        if self.wake_word_enabled:
            if not t.lower().startswith(self.wake_word):
                print(f"[AudioIn] Ignored: speech does not start with wake word '{self.wake_word}'.")
                return Intent("UNKNOWN", {}, t, 0.0, "Wake word missing")
            # Strip wake word
            t = t[len(self.wake_word):].strip()

        # Parse intent
        intent = self.parser.parse(t)

        # Publish event
        bus.publish(Event(
            type=EventType.INTENT_RECEIVED,
            source="AudioIn",
            data={"intent": intent.name, "params": intent.params, "raw_text": t, "spoken_reply": intent.spoken_reply}
        ))

        # Push to internal queue
        self.event_queue.put(intent)
        return intent

    def spot_stop_keyword(self, chunk_text: str) -> bool:
        """Emergency keyword spotter: check immediately for STOP/HALT."""
        words = ["stop", "halt", "freeze", "abort"]
        if any(w in chunk_text.lower() for w in words):
            intent = Intent("STOP", {}, chunk_text, 1.0, "Stopping immediately.")
            bus.publish(Event(type=EventType.STOP_TRIGGERED, source="AudioInStopSpotter", data={}))
            self.event_queue.put(intent)
            return True
        return False

    def close(self):
        self._running = False
