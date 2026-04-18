# 3D LiDAR Relocalization (ROS 2 Humble)

A high-performance global relocalization package for 3D LiDAR systems, designed to work alongside FAST-LIO or similar odometry frameworks.

This package enables robust global pose recovery using TEASER++ and continuous scan-to-map tracking using GICP, with proper TF fusion for consistent localization in ROS 2 navigation stacks.

---

## 🚀 Features

- 🔁 Global relocalization using **TEASER++**
- 📍 Continuous tracking via **GICP (scan-to-map)**
- 🧠 Robust to sparse and non-uniform LiDAR data (e.g., Livox MID-360)
- 🔧 Service-based relocalization trigger (`/relocalize_global`)
- 🌐 Proper TF fusion (`map → camera_init`) for compatibility with Nav2
- 📡 Works directly with live map streams (`/saved_map`)
- 🧩 Modular architecture (preprocessing, registration, refinement)

---

## 🏗️ System Architecture

```

Live Scan (/livox/lidar)
│
▼
Preprocessing (Voxel Grid)
│
▼
Global Registration (TEASER++)  ← triggered via service
│
▼
Local Refinement (GICP)
│
▼
Pose Estimation (map → body)
│
▼
TF Fusion
(map → camera_init)

```

---

## 🔁 TF Structure

This package integrates with FAST-LIO using:

```

map
↓
camera_init   ← published by this package
↓
body          ← FAST-LIO
↓
scan

````

---

## 📦 Topics

### Subscribed
- `/livox/lidar` → `sensor_msgs/PointCloud2`
- `/saved_map` → `sensor_msgs/PointCloud2`

### Published
- `/relocalized_pose` → `geometry_msgs/PoseStamped`
- `/aligned_scan` → debug aligned cloud
- `/tf` → `map → camera_init`

---

## ⚙️ Services

- `/relocalize_global` (`std_srvs/Trigger`)
  - Triggers global relocalization using TEASER++

---

## 🧪 Algorithms Used

### Global Registration
- FPFH feature extraction
- Feature correspondence matching
- **TEASER++ robust registration**

### Local Refinement
- **Generalized ICP (GICP)**

---

## 🔧 Build and Installation Intructions:

Please refer Installation.md
---

## ▶️ Run

```bash
ros2 run relocalization_3d relocalization_node
```

Trigger relocalization:

```bash
ros2 service call /relocalize_global std_srvs/srv/Trigger {}
```

---

## 📊 Visualization (RViz2)

Add:

* `/saved_map`
* `/aligned_scan`
* `/relocalized_pose`

Set Fixed Frame → `map`

---

## ⚠️ Notes

* Initial alignment requires sufficient overlap between scan and map
* Performance depends on map size (current implementation uses brute-force feature matching)
* Designed for LiDARs with non-uniform scan patterns (e.g., Livox)

---

## 🔮 Future Improvements

* KD-tree based feature matching (performance optimization)
* Map feature caching
* Loop closure integration
* Multi-threaded GICP
* GPU acceleration

---

## 📚 Dependencies

* ROS 2 Humble
* PCL
* Eigen
* TEASER++

---

## 🤝 Acknowledgements

* FAST-LIO for odometry backbone
* TEASER++ for robust global registration

