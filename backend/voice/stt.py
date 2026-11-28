"""Speech to text, offline, on the CPU: faster-whisper with the 'small' model from the shared Hugging Face cache.

  - Nothing is downloaded (local_files_only): the model must already be in ~/.cache/huggingface/hub
    (Systran/faster-whisper-small). Audio never leaves the laptop.
  - The model (~630 MB of RAM) is loaded on the first command and unloaded after IDLE_UNLOAD_S without use, so it
    doesn't sit in this laptop's tight RAM while nobody is talking.
  - A short list of the app's own words (take off, figure eight, box canyon, wingmen...) is given to Whisper as a
    prompt: it makes those words much more likely to be heard right.
  - Whisper "hears" words in silence (a famous quirk: "you", "thank you"). The voice-activity filter cuts silence
    first, and segments Whisper itself rates as probably-not-speech are dropped.
"""

from __future__ import annotations

import io
import threading
import time

MODEL_NAME = "small"
IDLE_UNLOAD_S = 120.0
NO_SPEECH_MAX = 0.6
MAX_AUDIO_BYTES = 5_000_000  # ~5 minutes of compressed audio: far more than a command needs
VOCABULARY = ("DroneLab drone commands: take off, land, hover, stop, go up, go down, go forward, go back, turn left, "
              "turn right, come home, figure eight, square loop, zig-zag, first flight, up and down, rings race, "
              "autopilot, box canyon, slalom, around the block, hidden walls, square patrol, wingmen, V formation, "
              "explore the building, the forest, lidar, SLAM, no GPS, wind, A star, farm, survey, north field, pad.")


class VoiceUnavailable(RuntimeError):
    """faster-whisper isn't installed, or the model isn't in the cache. The message says what to do."""


_lock = threading.Lock()
_model = None
_last_used = 0.0
_timer: threading.Timer | None = None


def available() -> tuple[bool, str]:
    try:
        import faster_whisper  # noqa: F401
    except ImportError:
        return False, "Voice is off: install it with  uv pip install --python .venv/bin/python -r backend/requirements-voice.txt"
    return True, f"faster-whisper '{MODEL_NAME}' (offline, CPU)"


def _load():
    global _model
    if _model is None:
        try:
            from faster_whisper import WhisperModel
        except ImportError as exc:
            raise VoiceUnavailable(available()[1]) from exc
        try:
            _model = WhisperModel(MODEL_NAME, device="cpu", compute_type="int8", cpu_threads=4, local_files_only=True)
        except Exception as exc:  # noqa: BLE001  (missing files show up as several different errors)
            raise VoiceUnavailable(f"The Whisper '{MODEL_NAME}' model isn't in the Hugging Face cache "
                                   f"(~/.cache/huggingface/hub/models--Systran--faster-whisper-{MODEL_NAME}).") from exc
    return _model


def _unload_if_idle() -> None:
    global _model, _timer
    with _lock:
        idle = time.monotonic() - _last_used
        if idle >= IDLE_UNLOAD_S - 1:
            _model = None  # the RAM goes back once Python drops it
            _timer = None
        else:  # used again since: check again when it would have been idle long enough
            _timer = threading.Timer(IDLE_UNLOAD_S - idle, _unload_if_idle)
            _timer.daemon = True
            _timer.start()


def transcribe(audio: bytes, extra_words: str = "") -> dict:
    """Audio file bytes (webm/ogg/wav, anything FFmpeg-in-PyAV reads) -> {text, seconds, audio_s}."""
    global _last_used, _timer
    if not audio:
        raise ValueError("No audio received.")
    if len(audio) > MAX_AUDIO_BYTES:
        raise ValueError("That recording is too long for a command.")
    start = time.monotonic()
    with _lock:
        model = _load()
        segments, info = model.transcribe(io.BytesIO(audio), language="en", vad_filter=True, beam_size=1,  # greedy: fastest; commands are short and the word list helps
                                         
                                          initial_prompt=f"{VOCABULARY} {extra_words}".strip(),
                                          condition_on_previous_text=False)
        text = " ".join(s.text.strip() for s in segments if s.no_speech_prob < NO_SPEECH_MAX).strip()
        _last_used = time.monotonic()
        if _timer is None:
            _timer = threading.Timer(IDLE_UNLOAD_S, _unload_if_idle)
            _timer.daemon = True
            _timer.start()
    return {"text": text, "seconds": round(time.monotonic() - start, 2), "audio_s": round(float(info.duration), 2)}


def warm() -> None:
    """Load the model now (in a thread, while the user is still speaking), so the first command isn't slow."""
    global _last_used
    with _lock:
        _load()
        _last_used = time.monotonic()


def loaded() -> bool:
    return _model is not None
