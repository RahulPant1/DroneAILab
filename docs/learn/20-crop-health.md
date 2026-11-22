# 20 · Crop-health maps: from photos to "go and look here"

**Big idea:** a farmer doesn't want 36 photos, they want an answer: *which parts of my field are in trouble, and where exactly?* That is the whole job of an agricultural drone: **fly → map → find → act**.

## 1. Seeing plant health: NDVI
Healthy leaves reflect a lot of **near-infrared** light (NIR, invisible to us) and absorb **red** light for photosynthesis. A multispectral camera measures both, and:

**NDVI = (NIR − red) ÷ (NIR + red)**

| Surface | NDVI |
|---|---|
| Healthy crop | about 0.8 |
| Stressed crop (dry, waterlogged, pests, disease) | 0.3 – 0.6 |
| Bare soil | about 0.15 |

Each farm field in DroneLab hides a few stressed patches: the **ground truth** we can check the drone against. The north field has a dry corner and some pest damage, the east field a waterlogged dip, and the orchard some diseased trees.

## 2. Stitching the photos (an orthomosaic)
Mapping software pastes every photo onto one big map at its **geotag**: where the drone *thought* it was when it took the picture. With perfect GPS the photos fit exactly. With real GPS (here: about 1.5 m of slow drift) they all land a metre or two off, so **the whole map is shifted** and a little smeared. Professional surveys use **RTK GPS** (centimetre accuracy) or **ground control points** (markers measured on the ground), plus image matching between photos.

## 3. Finding the problems
Smooth the map a little (so one noisy pixel isn't a "problem"), mark every spot with NDVI below 0.6, and group touching spots into **patches**. Ignore the 2 m strip along the edge (tractor tracks, and a shifted map puts soil there). Report each patch bigger than 2 m²: where, how big, how bad.

## 4. The farmer's report
The local AI turns the findings into a short note: where each patch is, how big, *possible* causes (labelled **[Hypothesis]**: a map can't tell drought from pests, someone has to walk there), and what to do first. It never sees the picture, only the numbers, and every number it writes is checked.

## What we measured (4 runs per row, 20 m height, 36 photos for the north field)
| Field | Conditions | Patches found | False alarms | How far off the found patches were |
|---|---|---|---|---|
| North (2 patches) | perfect GPS | 8 / 8 | 0 | 0.34 – 0.36 m |
| North | real GPS (1.5 m drift) | 8 / 8 | 1 | 0.4 – 1.6 m |
| East (1) | real GPS | 4 / 4 | 0 | 0.4 – 1.5 m |
| Orchard (1) | real GPS | 4 / 4 | 0 | 0.1 – 2.4 m |

The patches are found every time; GPS error decides **where on the map** they appear. A metre or two is fine for "walk over and look"; spraying exactly the right square metre needs better positioning.

## Try it
- **Lab → experiment 10:** pick a field and conditions. The NDVI map shows the found patches (numbered boxes) and the truth (dashed circles). Press **🤖 Farmer's report**.
- **Fly tab:** 🌾 farm → 🧭 Mission planner → survey a field → ✈ Fly (⏩ 8×). After landing, the photos are stitched and **🗺 Field map** appears.

## Self-check
1. Why does a healthy plant have a HIGH NDVI?
2. The map shows a problem patch 1.5 m east of where it really is. What caused that, and how would a professional survey avoid it?
3. Why does the report label causes as hypotheses?

## Answers
1. It reflects lots of near-infrared and absorbs most red light, so NIR − red is large compared with NIR + red.
2. The photos' geotags came from the drone's GPS, which was 1.5 m off, so every photo, and the whole map, was pasted 1.5 m east. RTK GPS or ground control points fix the map's position.
3. Low NDVI only says "these plants are struggling". Dry soil, too much water, pests and disease all look similar from above; someone has to go and look.
