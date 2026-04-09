# Visual Odometry — ORB-SLAM3 Monocular on AMtown02

## Group: X-AeroFusion-Group

**Course:** AAE5303 Robust Control Technology in Low-Altitude Aerial Vehicle (AY2025/2026 S2)

---

## Results Summary

| Metric | Value |
|--------|-------|
| ATE RMSE | **6.3168 m** |
| RPE Translation Drift | **2.10427 m/m** |
| Completeness | **98.77%** (3704 / 3750) |
| Leaderboard Rank | #6 (Total Score = 88.4) |
| Tracking Rate | 98.8% (7406 / 7499 frames) |
| Keyframes | 1288 |
| Loop Closure | Detected ✓ |
| Scale Correction | 1.4699 |

---

## Method

We use **ORB-SLAM3** in **pure Monocular offline mode** (`mono_euroc`) on the **AMtown02** dataset from the MARS-LVIG collection.

### Pipeline Overview

1. **Data Extraction** — Extract RGB images and IMU data from `AMtown02.bag` using `extract_images.py`. Original resolution 2448×2048 is downscaled to **1224×1024** (÷2) to reduce computation.
2. **ORB-SLAM3 Monocular** — Run the offline pipeline with calibrated camera intrinsics. The system performs feature extraction (ORB), tracking, local mapping, loop closing, and pose-graph optimization.
3. **Evaluation** — Align the estimated trajectory to ground truth (RTK) using Sim(3) alignment with scale correction, then compute ATE and RPE via the `evo` toolbox.

### Key Configuration (`AMtown02_Mono.yaml`)

```yaml
Camera.type: PinHole
Camera1.fx: 722.215
Camera1.fy: 722.17
Camera1.cx: 589.75
Camera1.cy: 522.45
Camera1.k1: -0.0560
Camera1.k2: 0.1180
Camera1.p1: 0.00122
Camera1.p2: 0.00064
Camera1.k3: -0.0627
Camera.width: 1224
Camera.height: 1024
Camera.fps: 10
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 7
```

Camera intrinsics are derived from the original MARS-LVIG calibration, scaled by 0.5 for the half-resolution images.

---

## Reproduce the Results

### Prerequisites

- **Docker image:** `liangyu99/orbslam3_ros1:latest`
- **Dataset:** `AMtown02.bag` (~17 GB) from [MARS-LVIG](https://mars.hku.hk/dataset.html)
- **Evaluation tool:** [evo](https://github.com/MichaelGrupp/evo) (install via `pip install evo`)

### Step 1: Extract images from the ROS bag

```bash
python3 extract_images.py
```

This extracts RGB images (downscaled to 1224×1024) and generates `timestamps.txt` into the `extracted/` directory.

### Step 2: Start the Docker container

```bash
docker run -it -d --name orbslam3 \
  -v D:\download:/data \
  liangyu99/orbslam3_ros1:latest bash
docker exec -it orbslam3 bash
```

### Step 3: Run ORB-SLAM3 (offline mode)

Inside the container:

```bash
cd /root/ORB_SLAM3
./Examples/Monocular/mono_euroc \
  ./Vocabulary/ORBvoc.txt \
  /data/extracted/AMtown02_Mono.yaml \
  /data/extracted \
  /data/extracted/timestamps.txt \
  AMtown02_Mono
```

Output files: `CameraTrajectory.txt` and `KeyFrameTrajectory.txt`.

### Step 4: Evaluate with evo

**Important:** The trajectory timestamps are in **nanoseconds** and must be converted to seconds before evaluation. The ground truth is already in seconds.

```bash
# Convert nanosecond timestamps to seconds
awk '{printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2, $3, $4, $5, $6, $7, $8}' \
  CameraTrajectory.txt > traj_sec.txt

# ATE (Absolute Trajectory Error)
evo_ape tum ground_truth.txt traj_sec.txt \
  --align --correct_scale --t_max_diff 0.1 -va

# RPE (Relative Pose Error)
evo_rpe tum ground_truth.txt traj_sec.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --pose_relation trans_part -va
```

---

## Hyperparameter Tuning

We performed several tuning experiments. The default configuration (v1) yielded the best result:

| Version | Changes | ATE RMSE | Notes |
|---------|---------|----------|-------|
| **v1 (default)** | nFeatures=2000, scaleFactor=1.2, nLevels=8 | **6.32 m** | ★ Best |
| run2 | Same as v1 (re-run) | 6.40 m | Non-deterministic variance |
| v2 | nFeatures=3000, iniThFAST=10, minThFAST=3, nLevels=10 | 6.58 m | Lower quality features |
| v3 | nFeatures=5000 | Failed | Tracking lost |
| CLAHE v1 | CLAHE preprocessing + scaleFactor=1.15 | 14.19 m | Scale estimation error |
| CLAHE v2 | CLAHE preprocessing + scaleFactor=1.2 | 7.37 m | Noise amplification |
| nL5 | nLevels=5 | 6.46 m | Slightly worse |

### Observations

- Increasing `nFeatures` beyond 2000 introduced lower-quality features that degraded matching accuracy.
- CLAHE histogram equalization amplified noise textures in outdoor scenes, hurting scale estimation.
- Reducing `nLevels` limited the scale pyramid and reduced robustness to scale changes.
- ORB-SLAM3 has inherent non-determinism (multi-threading), so repeated runs with identical settings produce slightly different results (~0.1 m variance).

---

## File Structure

```
visual_odometry/
├── AMtown02_Mono.yaml      # ORB-SLAM3 configuration file
├── README.md               # This file
└── extract_images.py       # Script to extract images from ROS bag (optional)
```

---

## Leaderboard Submission

```json
{
  "group_name": "X-AeroFusion-Group",
  "project_private_repo_url": "https://github.com/X-AeroFusion-Group/AAE5303",
  "metrics": {
    "ate_rmse_m": 6.3168,
    "rpe_trans_drift_m_per_m": 2.10427,
    "completeness_pct": 98.77
  }
}
```

Leaderboard: https://qian9921.github.io/leaderboard_web/

---

## References

- Campos, C., Elvira, R., Rodríguez, J.J.G., Montiel, J.M., & Tardós, J.D. (2021). ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multi-Map SLAM. *IEEE Transactions on Robotics*, 37(6), 1874–1890.
- MARS-LVIG Dataset: https://mars.hku.hk/dataset.html
- evo evaluation tool: https://github.com/MichaelGrupp/evo
