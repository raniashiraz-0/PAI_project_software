"""
audio_out.py - Text-to-speech output with half-duplex microphone muting.
Mutes mic while robot speaks to prevent self-hearing / feedback loops.
"""
import time
import threading
from typing import Optional, List
from events import bus, Event, EventType

class AudioOutput:
    def __init__(self, audio_in=None, tts_backend: str = "offline"):
        self.audio_in = audio_in
        self.tts_backend = tts_backend
        self._lock = threading.Lock()
        self.speech_history: List[str] = []
        self.last_spoken: str = ""
        self._engine = None

        if tts_backend == "pyttsx3":
            try:
                import pyttsx3
                self._engine = pyttsx3.init()
                self._engine.setProperty('rate', 150)
            except Exception as e:
                print(f"[AudioOut] pyttsx3 init failed ({e}), using console/web audio mode.")
                self._engine = None

    def speak(self, text: str, non_blocking: bool = True):
        """Speak phrase with half-duplex mic mute."""
        if not text:
            return

        with self._lock:
            self.last_spoken = text
            self.speech_id = getattr(self, "speech_id", 0) + 1     # lets the browser repeat the same sentence
            self.speech_history.append(text)
            if len(self.speech_history) > 50:
                self.speech_history.pop(0)

        print(f"[AudioOut] $SAY: '{text}'")
        bus.publish(Event(type=EventType.TTS_SPEAK, source="AudioOut", data={"text": text}))

        if non_blocking:
            threading.Thread(target=self._speak_sync, args=(text,), daemon=True).start()
        else:
            self._speak_sync(text)

    def _speak_sync(self, text: str):
        # 1. Half-duplex: mute microphone
        if self.audio_in:
            self.audio_in.set_muted(True)

        try:
            if self._engine:
                self._engine.say(text)
                self._engine.runAndWait()
            else:
                # Sim speech delay: ~60ms per character
                time.sleep(max(0.4, len(text) * 0.05))
        except Exception as e:
            print(f"[AudioOut] TTS speak error: {e}")
        finally:
            # 2. Unmute microphone after short buffer
            time.sleep(0.15)
            if self.audio_in:
                self.audio_in.set_muted(False)

    def get_latest_speech(self) -> str:
        with self._lock:
            return self.last_spoken
