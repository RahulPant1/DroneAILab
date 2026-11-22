# 17 · Surveys: mapping a field from the air

**Big idea:** most paid drone work (farms, construction sites, solar parks, insurance) is **surveying**: photograph a whole area from above so software can stitch the photos into one map. Three numbers decide everything: **height**, **overlap** and **coverage**.

## The camera footprint
A camera pointing straight down sees a rectangle of ground. Its size grows with height:
- across = 2 × height × tan(70° ÷ 2) ≈ **1.4 × height**; along = across × 3/4 (a 4:3 picture)
- **Ground sample distance (GSD)** = across ÷ 4000 pixels: how much ground one pixel covers. Lower = sharper.

| Height | One photo covers | GSD | North field (40 × 28 m): lines, photos, time |
|---|---|---|---|
| 10 m | 14 × 10.5 m | 0.35 cm/pixel | 7 lines, 113 photos, 203 s |
| 20 m | 28 × 21 m | 0.70 cm/pixel | 4 lines, 36 photos, 142 s |
| 40 m | 56 × 42 m | 1.40 cm/pixel | 2 lines, 10 photos, 114 s |

Higher means fewer photos and a shorter flight, but less detail. You choose the height from the detail you need (counting plants, finding a crack, or just an overview).

## The lawnmower pattern
Fly parallel lines back and forth across the field, along its **longest side** (fewer turns). Neighbouring photos must **overlap**:
- **side overlap** between lines (70 % here): lines are footprint × (1 − 0.7) apart;
- **front overlap** along a line (75 %): a photo every footprint × (1 − 0.75) metres.
Stitching software needs every spot in **two or more photos** to line them up and measure heights.

## The camera trigger
Like ArduPilot's `CAM_TRIGG_DIST`: on a survey line, take a photo every *photo spacing* metres the drone **thinks** it has travelled. The picture is taken where the drone **really** is. So wind and GPS error move the photos away from their planned spots.

## What we measured (north field, 20 m, 3 seeds)
| Side overlap | Calm, perfect GPS | Gusty 6 m/s crosswind + real GPS (1.5 m drift) |
|---|---|---|
| 0 % | 100 % covered | **94.8 %, 97.6 %, 98.8 %**: gaps between lines |
| 10 % | 100 % | 100 % |
| 70 % | 100 % | 100 % |

With no overlap there's no margin: the wind and the GPS error push a line a metre or two sideways, and a strip between lines is never photographed. Even a little overlap hides that error. Real surveys use 60–80 % so the stitching works well, not just for coverage.

**Same survey on real ArduPilot** (`python -m backend.sitl.plan_flight --write`, the same plan uploaded as an AUTO mission): see [docs/results/survey-compare.md](../results/survey-compare.md). Both covered 100 % of the field with about the same number of photos.

## Try it
- **Lab → experiment 8:** change height, overlap and conditions; the map shows every photo's footprint and any gaps (red dots).
- **Fly tab:** course **🌾 Hillside farm** → **🧭 Mission planner** → survey a field → ✈ Fly. Use ⏩ to speed up time. Blue rectangles appear on the ground as photos are taken.

## What it doesn't model
Hills (the ground is flat here), the camera tilting with the drone (we assume a gimbal keeps it pointing down), motion blur, lighting, and the stitching itself.

## Self-check
1. You fly twice as high. What happens to the GSD and to the number of photos?
2. Why fly the lines along the field's longest side?
3. Overlap is 0 % and the drone is perfectly on its line. Why can a gap still appear on a windy day with real GPS?

## Answers
1. GSD doubles (each pixel sees twice as much ground, so less detail). Each photo covers 4× the area, so you need roughly a quarter of the photos (fewer lines and a bigger spacing along each).
2. Fewer, longer lines means fewer turns, and turns are wasted time (no photos while turning).
3. It is on its line **by its own estimate**. GPS drift means it is really a metre or two to the side, and wind pushes it about too. The photos land where it really was, so a strip next to the line can be missed.
