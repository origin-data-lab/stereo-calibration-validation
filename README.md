# Checking a Stereo Calibration on Ordinary Footage

**Origin Data Lab · Technical Field Note #04 · 23 October 2026**

[![Field Note](https://img.shields.io/badge/Field_Note-%2304-0B1220?style=for-the-badge)](https://origindatalab.io/technical-field-notes/stereo-calibration-check.html)
![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB?style=for-the-badge)
![License](https://img.shields.io/badge/Code-MIT-16A34A?style=for-the-badge)

> **Short answer.** You can check whether a stereo calibration fits a device and capture mode using ordinary footage, without a target. On a 120 s, 1600×1200 head-mounted stereo recording, a Kalibr calibration from the same capture mode gave a **median angular epipolar error of 0.41 px** on 19,386 unfiltered matches; a calibration of the same camera family from a different (16:9) mode gave **1.83 px**. Two common checks can mislead on wide-angle lenses (see below).

**Full article:** [Technical Field Note #04 — Checking a Stereo Calibration on Ordinary Footage](https://origindatalab.io/technical-field-notes/stereo-calibration-check.html)

---

## Key results

| Result | Value |
|---|---:|
| Median angular epipolar error, mode-matched calibration, no adjustment | **0.41 px** (19,386 matches, 29 frame pairs) |
| Same matches, calibration from a different capture mode | **1.83 px** |
| Stereo-rotation correction requested by the data | **< 0.02°** per axis |
| Deliberate 0.1° pitch error | median rises to **0.76 px** |
| Rectified vertical residual, 0–20° vs 75–90° field angle | **0.50 px → 4.05 px** (angular metric stays **0.46 px**) |
| `cv2.fisheye.undistortPoints` (OpenCV 5.0, defaults) | exact to ~**74°**; **4.5°** wrong at 80°, **8.0°** at 85° for this lens |

All values are from the evaluated recording and configuration; they are not universal guarantees.

## Two traps on wide-angle stereo

**1. Rectified vertical residuals inflate toward the image edge.** Projecting wide-angle rays onto a flat rectified image stretches the periphery (roughly 1/cos² of the angle), so a correct calibration looks wrong at the edges. Use an angular metric, or restrict rectified checks to the central field of view.

**2. OpenCV's default fisheye un-projection clamps the distorted angle at π/2.** With a positive k₁, θd reaches π/2 before the true ray angle reaches 90°, and every point beyond returns at the wrong angle. Run `tools/fisheye_roundtrip_check.py` with your own coefficients to see where this happens for your lens.

## Tools

```bash
pip install opencv-python numpy scipy pyyaml

# 1. Validate a Kalibr pinhole-equidistant stereo calibration on ordinary stereo footage
python tools/validate_stereo_calib.py --left left.mp4 --right right.mp4 \
    --camchain camchain.yaml --scale 2 --out result.json
#   --scale = video resolution / calibration resolution

# 2. Find where OpenCV's fisheye un-projection breaks for your coefficients
python tools/fisheye_roundtrip_check.py --camchain camchain.yaml --cam cam0 --scale 2
python tools/fisheye_roundtrip_check.py --k 0.12 0 0 0 --f 480 --cx 800 --cy 600
```

## Repository contents

| Path | Contents |
|---|---|
| `tools/validate_stereo_calib.py` | Target-free stereo calibration check (angular epipolar error, field-angle breakdown, held-out rotation fit) |
| `tools/fisheye_roundtrip_check.py` | OpenCV fisheye round-trip test |
| `results/` | Measured results behind the article (JSON) |
| `figures/` | Figures used in the article |

The device calibration parameters and the source recording are not published in this repository.

## What this check does not verify

- **Baseline length / metric scale** — absent from the epipolar constraint.
- **Metric depth accuracy** — requires targets at measured distances.
- **Camera–IMU calibration** — time shift and extrinsics need a separate test.

## Cite

Use the **"Cite this repository"** button (from `CITATION.cff`), or:

```
Origin Data Lab (2026). Checking a Stereo Calibration on Ordinary Footage. Technical Field Note #04.
https://origindatalab.io/technical-field-notes/stereo-calibration-check.html
```

## License

- Code in `tools/`: [MIT](LICENSE)
- Text, results and figures: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/) — free to share and adapt for non-commercial use with attribution to Origin Data Lab.

## Related

- [All Technical Field Notes](https://origindatalab.io/technical-field-notes/)
- [한국어 — 스테레오 캘리브레이션 검증 (Field Note #04)](https://origindatalab.io/ko/technical-field-notes/stereo-calibration-check.html)
- [#03 Stereo + IMU Egocentric Capture](https://origindatalab.io/technical-field-notes/stereo-imu-egocentric-sample.html) · [repository](https://github.com/origin-data-lab/stereo-imu-egocentric-evaluation)

---

**Origin Data Lab** — real-world data production for Physical AI and robotics · [origindatalab.io](https://origindatalab.io) · [contact@origindatalab.io](mailto:contact@origindatalab.io)

Need per-device, per-capture-mode stereo calibration validated on your own footage? [Contact us](https://origindatalab.io/contact.html).
