# 27 · Voice commands: talk to the drone

**Big idea:** say *"take off"*, *"go left 3 metres"* or *"race the figure eight with the autopilot"*, and the drone does it, everywhere in the app: the playground, the race games, wingmen, obstacle courses, the farm and exploration. Everything runs on this laptop: your voice never goes to the internet.

## How a sentence becomes a flight
```
 🎙 microphone ─> Whisper "small" (offline, CPU, ~1 s) ─> text
   "stop" anywhere? ───────────────> hover NOW (and cancel anything waiting)
   a fixed phrase? ────────────────> do it at once (no AI)          take off · land · hover · go left 3 m · turn right · come home
   anything else ──> local AI (Gemma) picks ACTIONS from a list of what exists right now
                     ─> code checks every name and number ─> small moves: do it
                                                           ─> routes / new worlds: "Say go to start"
 🔊 the answer is spoken back (the browser's own voices)
```

## Why it is built like this
- **Speech to text is its own small model.** Whisper turns sound into words; it doesn't fly anything. DroneLab reuses the `faster-whisper` *small* model already on this laptop (from another project), read from the shared model cache: nothing was downloaded. It uses about 630 MB of RAM while it's loaded, and it unloads after 2 minutes of quiet.
- **Fixed phrases skip the AI.** The commands a kid says most (*take off, land, stop, go up, go left 2 metres, turn around, come home*) are matched directly. They are instant and work even when the AI is busy or switched off. You can chain them: *"go forward 4 metres and then turn around"*.
- **The AI can only choose, not invent.** For bigger requests, Gemma gets a list of actions and the exact names that exist (race tracks, courses, worlds, wingmen shapes...), as an enum in a JSON schema. It can't make up a track or a coordinate. Code then checks every number (a move is 0.2–20 m, a turn 5–360°) and whether the action makes sense now (*"the autopilot needs a race loaded first"*).
- **Word rules for what a small model gets wrong.** In our tests Gemma heard *"explore the building without GPS with someone walking around"* but chose "dead reckoning" and no person. Simple word rules now fill in those details (*without GPS* = SLAM, *someone walking* = a person). They only correct details of an action the model already chose; they never add a new one.
- **Anything big waits for "go".** Flying a route by itself (a mission, the race autopilot, exploring, a farm survey) or jumping to another world while flying is said back first: *"I'll race the figure eight on autopilot. Say go to start."* Small moves happen at once, like pressing a key.
- **"Stop" always wins.** If the word *stop* (or *freeze*, *halt*) is anywhere in what was heard, the drone hovers at once and anything waiting is cancelled. There's also a ✋ Stop button.
- **On the farm**, the request goes to the farm's own AI mission planner (lesson 19), with its safety checks (no-fly zones, battery, geofence). A plan that fails a check is refused out loud.

## What we tested (local Gemma, live simulator)
| You say | It did |
|---|---|
| "go forward 4 m and then turn around" | quick: forward 4 m, turn 180° |
| "let's do the figure eight race" | loaded the track (on the ground: no "go" needed) |
| "let the autopilot race it" → "yes" | race autopilot flying |
| "bring three wingmen in a line" | 3 wingmen, line abreast |
| "I want to fly the slalom course by itself with A star" → "go" | slalom loaded, A* mission flying |
| "explore the building without GPS and with someone walking around" | building, lidar SLAM, a person walking (after the word-rule fix) |
| "make it windy from the north" | 5 m/s wind from the north |
| "survey the east field and land on pad B" → "go" | farm planner's plan, checked, flying |
| "fly to the moon" / "do a flip" | "Sorry, I can't do that" |

Speech recognition itself wasn't tested with a real voice on this machine (no microphone in the test setup). Try it, and if a word is often misheard, it can be added to Whisper's word list (`VOCABULARY` in `backend/voice/stt.py`).

## Try it
Fly view → **🎙 Talk** (top of the controls). Press, speak, press again (it stops by itself after 8 s), or type in the box. The first command takes a few seconds longer while Whisper loads. Untick 🔊 to stop it talking back.

Setup, if the button is greyed out: `uv pip install --python .venv/bin/python -r backend/requirements-voice.txt`.

## Self-check
1. Why are "take off" and "stop" handled without the AI?
2. The AI picked `race_autopilot` but no race is loaded. What happens?
3. Why does "explore the building" wait for "go", but "go left 2 metres" doesn't?

## Answers
1. They must work instantly and always, even when the AI is slow, busy or switched off. A fixed phrase can't be misunderstood by a model.
2. Code checks it before anything moves: "Load a race track first". If the same sentence also loaded a race ("race the figure eight with the autopilot"), the check runs in order and passes.
3. Exploring flies a whole route by itself and replaces the world around the drone. A 2 m move is what you'd do with one key press, and "stop" can always undo it.
