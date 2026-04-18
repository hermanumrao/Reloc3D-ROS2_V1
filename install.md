# 🔧 Installation Guide

## 1. System Requirements

* Ubuntu 22.04
* ROS 2 Humble
* C++17 compiler
* PCL (>= 1.10)
* Eigen3

---

## 2. Install ROS 2 Dependencies

```bash
sudo apt update

sudo apt install -y \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-srvs
```

---

## 3. Install System Dependencies

```bash
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  build-essential \
  cmake \
  git
```

---

## 4. Install TEASER++

```bash
cd ~
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

---

## 5. Verify TEASER++ Installation

```bash
ls /usr/local/lib | grep teaser
```

Expected:

```bash
libteaser_registration.a
libteaser_io.a
```

---

## 6. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

## 7. Clone the Repository

```bash
git clone <your_repo_url>
```

---

## 8. Build the Package

```bash
cd ~/ros2_ws

colcon build --packages-select relocalization_3d
source install/setup.bash
```

---

## 9. (Optional) Fix Library Path Issues

If TEASER++ is not found at runtime:

```bash
sudo ldconfig
```

Or manually:

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

---

# Map Publisher Setup (PCD Publisher)
Installing a PCD Publisher

You can use the following package to publish a saved map as a ROS 2 topic:

👉 pcd_publisher

1. Clone into Workspace
```bash
cd ~/ros2_ws/src

git clone https://github.com/omerssid/pcd_publisher.git

```

2. Build

```bash
cd ~/ros2_ws

colcon build --packages-select pcd_publisher
source install/setup.bash
3. Run PCD Publisher
ros2 run pcd_publisher pcd_publisher_node \
  --ros-args \
  -p file_path:=/path/to/your/map.pcd \
  -p topic_name:=/saved_map \
  -p frame_id:=map
```

4. Verify Output
`ros2 topic echo /saved_map`

You should see:

`sensor_msgs/msg/PointCloud2 `
🔗 Integration with Relocalization Pipeline

Your system expects:

Topic	          | Purpose
/saved_map	    | Global reference map
/livox/lidar	  | Live scan

# Running the Node

start with running you ros driver (livox in my case) which will be different based on the lidar you are using
```bash
ros2 run relocalization_3d relocalization_node
```

---

## Triggering Relocalization

```bash
ros2 service call /relocalize_global std_srvs/srv/Trigger {}
```

---

# ⚠️ Common Issues

### 1. TEASER++ linking error

```
cannot find -lteaserpp::teaserpp
```

✔ Fix:

* Ensure `/usr/local/lib/libteaser_registration.a` exists
* Use `find_library()` in CMake (already handled in repo)

---

### 2. TF not available

```
TF lookup failed
```

✔ Fix:

* Ensure FAST-LIO (or equivalent) is running
* Frames must exist: `camera_init → body`

---

### 3. Slow relocalization

✔ Expected (current version uses brute-force matching)
✔ Will be optimized later (KD-tree, caching)

---

# ✅ After Installation

Verify:

```bash
ros2 topic list
```

You should see:

* `/relocalized_pose`
* `/aligned_scan`
* `/relocalize_global`

---

# 🚀 Ready to Use

Once installed, the system:

* Tracks pose using GICP
* Relocalizes globally using TEASER++
* Publishes correct `map → camera_init` transform
