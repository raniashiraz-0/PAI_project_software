"""
events.py - Event definitions, mission states, display states, and event bus for Pharma Rover.
"""
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Callable, List

class MissionState(str, Enum):
    IDLE = "IDLE"
    LISTENING = "LISTENING"
    THINKING = "THINKING"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    DOCKING = "DOCKING"
    ARM_SEARCH = "ARM_SEARCH"
    ARM_VERIFY = "ARM_VERIFY"
    ARM_APPROACH = "ARM_APPROACH"
    ARM_GRIP = "ARM_GRIP"
    ARM_LIFT = "ARM_LIFT"
    ARM_CARRY = "ARM_CARRY"
    NAVIGATING_RETURN = "NAVIGATING_RETURN"
    ARM_DELIVER = "ARM_DELIVER"
    SUCCESS = "SUCCESS"
    ERROR = "ERROR"
    STOPPED = "STOPPED"

class DisplayState(str, Enum):
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    SCANNING = "scanning"
    ALERT = "alert"
    OBSTACLE = "obstacle"
    HAPPY = "happy"
    SUCCESS = "success"
    ERROR = "error"

class EventType(str, Enum):
    # System & Mission
    MISSION_STARTED = "MISSION_STARTED"
    MISSION_STATE_CHANGED = "MISSION_STATE_CHANGED"
    MISSION_COMPLETED = "MISSION_COMPLETED"
    MISSION_CANCELLED = "MISSION_CANCELLED"
    MISSION_ERROR = "MISSION_ERROR"
    STOP_TRIGGERED = "STOP_TRIGGERED"
    ESTOP_CHANGED = "ESTOP_CHANGED"

    # Car
    CAR_ARRIVED = "CAR_ARRIVED"
    CAR_STOPPED = "CAR_STOPPED"
    CAR_OBSTACLE = "CAR_OBSTACLE"
    CAR_LINK_LOST = "CAR_LINK_LOST"
    CAR_POSE_UPDATED = "CAR_POSE_UPDATED"

    # Arm
    ARM_MOVE_STARTED = "ARM_MOVE_STARTED"
    ARM_MOVE_DONE = "ARM_MOVE_DONE"
    ARM_DOCKED = "ARM_DOCKED"
    ARM_VERDICT = "ARM_VERDICT"
    GRAB_AUTHORISED = "GRAB_AUTHORISED"
    GRAB_REJECTED = "GRAB_REJECTED"
    ARM_GRIP_DONE = "ARM_GRIP_DONE"
    ARM_CARRY_READY = "ARM_CARRY_READY"
    ARM_DELIVERED = "ARM_DELIVERED"
    ARM_ABORTED = "ARM_ABORTED"
    ARM_ERROR = "ARM_ERROR"

    # Vision
    TRACKER_POSE = "TRACKER_POSE"
    TRACKER_LOST = "TRACKER_LOST"
    TRACKER_CAMERA_MOVED = "TRACKER_CAMERA_MOVED"
    VERIFICATION_RESULT = "VERIFICATION_RESULT"

    # Audio & Intents
    AUDIO_VOICE_DETECTED = "AUDIO_VOICE_DETECTED"
    AUDIO_TRANSCRIPTION = "AUDIO_TRANSCRIPTION"
    INTENT_RECEIVED = "INTENT_RECEIVED"
    TTS_SPEAK = "TTS_SPEAK"

    # Display
    DISPLAY_STATE_UPDATE = "DISPLAY_STATE_UPDATE"
    DISPLAY_CARD_UPDATE = "DISPLAY_CARD_UPDATE"

@dataclass
class Event:
    type: EventType
    source: str
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    mission_id: Optional[str] = None

class EventBus:
    """Thread-safe synchronous/asynchronous in-memory event bus."""
    def __init__(self):
        self._subscribers: Dict[EventType, List[Callable[[Event], None]]] = {}
        self._all_subscribers: List[Callable[[Event], None]] = []

    def subscribe(self, event_type: EventType, callback: Callable[[Event], None]):
        if event_type not in self._subscribers:
            self._subscribers[event_type] = []
        self._subscribers[event_type].append(callback)

    def subscribe_all(self, callback: Callable[[Event], None]):
        self._all_subscribers.append(callback)

    def publish(self, event: Event):
        # Call specific subscribers
        if event.type in self._subscribers:
            for cb in self._subscribers[event.type]:
                try:
                    cb(event)
                except Exception as e:
                    print(f"[EventBus] Error in handler for {event.type}: {e}")

        # Call global subscribers
        for cb in self._all_subscribers:
            try:
                cb(event)
            except Exception as e:
                print(f"[EventBus] Error in global handler for {event.type}: {e}")

# Global singleton event bus
bus = EventBus()
