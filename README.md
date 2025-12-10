# ReckonSLAM: Mobile Dead Reckoning and Visual SLAM for Android

**ReckonSLAM** is a lightweight Android app that demonstrates camera-based **Dead Reckoning (DR)** and **basic Visual SLAM** (feature-based). The project is educational and focused on clarity — it shows VO/DR behavior and provides diagnostic tools to help evaluate and demonstrate tracking quality.

**Repository:** [https://github.com/AbhishekGuna/ReckonSLAM](https://github.com/AbhishekGuna/ReckonSLAM)
**Demo (unlisted):** [https://drive.google.com/drive/folders/1a8vefgEPwJSAnBktxZe_kVRn7sgvOSXw](https://drive.google.com/drive/folders/1a8vefgEPwJSAnBktxZe_kVRn7sgvOSXw)

---

## Key Features

### Core Tracking
* **Camera integration** (Camera2 / CameraX)
* **Feature detection & matching** (ORB / FAST)
* **Visual Odometry (VO)**: Essential Matrix / frame-to-frame pose estimation
* **Trajectory accumulation** (6-DOF pose tracking)
* **SLAM hooks**: pose-graph utilities and loop-closure readiness
* **Optional IMU integration** for better scale and robustness

### Diagnostic & Demo Enhancements (major differentiators)
* **Motion Stability Indicator** — real-time IMU-variance-based status (Green / Yellow / Red)
* **Leveler Tool** — pitch / roll readout showing **$0^\circ$** when perfectly level
* **Sensor Health Dashboard** — live IMU values, sampling rate, sparklines, motion/no-motion badge
* **Step Counter + Distance Estimator** — accelerometer peak detection $\times$ stride length

### Visualization
* Camera preview with overlaid ORB keypoints and tracking metrics
* On-screen DR metrics: X/Y displacement, distance, bearing, step count

---

## How it works (high-level VO pipeline)

1.  **Frame capture & preprocessing** — convert to grayscale, (optionally) downsample
2.  **Feature extraction & matching** — ORB keypoints + BFMatcher (Hamming)
3.  **Relative motion estimation** — Essential matrix / recoverPose / PnP
4.  **Trajectory accumulation** — apply relative transforms to update absolute pose
5.  **Diagnostics & visualization** — show IMU-based DR, SLAM overlays, and diagnostic widgets

---

## Project structure

```yaml
app/
├── camera/        # Camera2/CameraX pipeline & frame callbacks
├── processing/    # Preprocessing (grayscale, undistort, resize)
├── features/      # ORB / FAST detection & descriptor matching
├── vo/            # Visual Odometry (frame-to-frame)
├── slam/          # Pose graph & SLAM utilities (hooks)
├── dr/            # Dead Reckoning (IMU integration + step counter)
├── diagnostics/   # Stability indicator, leveler, sensor dashboard
├── logging/       # Pose logs, IMU logs, debug exports
└── ui/            # App UI, overlays, trajectory rendering
```

---

## Setup & Build

## 1. Clone the repository

```bash
git clone https://github.com/AbhishekGuna/ReckonSLAM.git
cd ReckonSLAM
```

## 2. Open in Android Studio

Open the project folder in Android Studio.  
Let Gradle sync and download dependencies (OpenCV is included via Maven).

---

## 3. Build & Run

Select the **app** module and run on a **physical Android device** (recommended for camera/IMU access).  
Prefer devices with good camera and IMU sampling.

---

# Exported Logs & Outputs

The app supports exporting:

- **6-DOF pose logs** (position + orientation)
- **IMU streams** (accelerometer, gyro)
- **Feature & match statistics** (feature count, matches, inliers)

These outputs can be used for offline analysis and plotting.

---

# Evaluation Notes (for demos / reports)

- Dead Reckoning (DR) accumulates drift quickly over distance; check **step-counter mode** for pedestrian tests.
- SLAM stabilizes pose where there is good visual texture and smooth motion.
- Use the **Motion Stability Indicator** to warn demonstrators about motions likely to break tracking.
- For quantitative evaluation, align DR and SLAM logs by timestamp and compute **per-sample correction magnitudes (drift)**.

---

# Implementation Details & Tips

- **Core CV APIs used:**  
  `ORB::create()`, `BFMatcher`, `findEssentialMat`, `recoverPose`, `solvePnP`
- **Frame conversion:**  
  Convert camera frames (YUV) → **OpenCV Mat (RGB/GRAY)** for feature extraction.
- **Performance optimization:**  
  - Tune ORB keypoint limits  
  - Use keyframe-based processing  
  - Avoid heavy on-device bundle adjustment

---

# Future Work

Planned enhancements include:

- Tightly-coupled **VIO (Visual-Inertial Odometry)**
- Loop-closure detection and **pose-graph optimization** (sparse BA)
- Map saving/loading and **cross-device relocalization**
- Automated **IMU & stride-length calibration**

---

# References & Acknowledgments

- OpenCV: https://github.com/opencv/opencv  
- OpenCV Android Documentation:  
  https://docs.opencv.org/4.x/d5/df8/tutorial_dev_with_OCV_on_Android.html  
- Project Repository: https://github.com/AbhishekGuna/ReckonSLAM  
- Demo Video:  
  https://drive.google.com/drive/folders/1a8vefgEPwJSAnBktxZe_kVRn7sgvOSXw  
