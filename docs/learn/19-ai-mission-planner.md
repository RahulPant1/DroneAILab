# 19 · The AI mission planner: plain English in, a checked flight plan out

**Big idea:** let people say what they want ("survey the north field, then land on pad B") and have a language model turn it into a flight plan, **without trusting the model with anything that matters for safety**. The AI writes a short task list; ordinary code does the maths and the checks; a person presses Fly.

## The pipeline
```
"Survey the north field at 25 m, then land on pad B"
   │  local LLM (Gemma via Ollama). Its reply must match a JSON schema whose place names come from the map.
   ▼
{"tasks": [{"type": "survey", "field": "north field"}, {"type": "land", "place": "pad B"}], "altitude_m": 25, ...}
   │  code: lawnmower lines, A* routing around tall buildings and no-fly zones, safety checks
   ▼
route + ✅ / ⚠️ / ❌ checks  →  you press ✈ Fly (DroneLab) or 🛰 Fly on ArduPilot
```

## The safety net (why the AI can't make it unsafe)
| Risk | What stops it |
|---|---|
| The model invents a place or coordinates | It can only use names from the map (the JSON schema lists them). Coordinates are looked up by code. |
| It asks for somewhere not on the map | It must list it in `not_on_map`, and that blocks the plan ❌. |
| It plans into a no-fly zone | No-fly zones aren't valid destinations; the route planner (A*) treats them as walls. |
| Unsafe numbers (150 m, 9 m/s) | It is told to write them down as asked; the checks then fail them ❌. |
| It quietly drops part of the request | It must list what it left out; you see a ⚠️. |
| It makes up numbers in its explanation | Every number is checked against your request, the map and the plan; unknown ones are flagged. |
| Not enough battery | Code estimates the energy (time × hover power, measured on the simulated drone) and wants 30 % left at landing. |
| Something changed since the check | The server compiles and checks the plan again before it flies, in DroneLab and on ArduPilot. |

The checks are the same with or without the AI: you can build the task list by hand in the panel too.

## What the real local model did (gemma4 e4b, 4-bit, on this laptop's GPU)
| You say | It planned | Checks |
|---|---|---|
| "Survey the north field, then land on pad B" | survey north field → land on pad B (camera) | ✅ all pass |
| "Map the east field from 12 m and hover over the gate for 10 seconds on the way back" | survey east field at 12 m → gate → hover 10 s → home | ✅ all pass |
| "Check the orchard at 25 m, then fly over the horse paddock" | survey orchard at 25 m; paddock **left out** ("it is a No-Fly Zone") | ⚠️ left out |
| "Fly to the school and back" | `not_on_map: ["school"]` | ❌ Places |
| "Survey the north field at 150 m" | survey at 150 m (as asked) | ❌ Altitude (ceiling 60 m) |
| "Survey all three fields at 8 m" | three surveys at 8 m | ❌ Battery: ~79 % of the pack, too little reserve |

After the first model loads (~50 s), each plan takes about 2 s. Two early mistakes (ignoring "from 12 m", adding a needless "fly to the field" before each survey) were fixed by sharpening the instructions and by dropping that step in code.

## Try it
**Fly tab** → course **🌾 Hillside farm** → **🧭 Mission planner**. Type a request or click an example. Read the checks and the map, then ✈ Fly. Use ⏩ to speed up time. With the 🛰 ArduPilot panel started, **Fly on ArduPilot too** uploads the same plan to real ArduCopter as an AUTO mission (orange photo footprints).

## Self-check
1. Why let the model choose only from names, instead of letting it write coordinates?
2. A request asks for 150 m. Why should the model write 150 instead of quietly using a safe 60?
3. Why does the server check the plan again when you press Fly?

## Answers
1. A wrong name is easy to catch (it isn't on the map); a wrong coordinate looks just like a right one. Names also mean the model can't send the drone somewhere nobody chose.
2. So the person sees that the request was unsafe and why (a ❌ with the reason). If the model silently changed it, the plan would no longer be what was asked, and nobody would know.
3. Never trust what comes back from the browser: the plan could have been edited, or the battery or drone changed. The check that counts is the one right before flight.
