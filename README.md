# EyeGaze — ROS Eye Gaze Tracking

A ROS (Robot Operating System) catkin workspace for real-time eye gaze detection and tracking. The project uses computer vision techniques via Python and C++ to estimate a user's gaze direction, making it suitable for human-robot interaction, accessibility applications, and attention-aware systems.

---

## 📋 Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

---

## ✨ Features

- Real-time eye gaze estimation using a camera feed
- ROS-native design — publishes gaze data as ROS topics
- Python and C++ nodes for flexible integration
- Compatible with standard ROS visualization tools (e.g., RViz)
- Lightweight and suitable for embedded/robot platforms

---

## 🧰 Prerequisites

Make sure the following are installed on your system before proceeding:

| Dependency | Version |
|---|---|
| Ubuntu | 18.04 / 20.04 |
| ROS | Melodic / Noetic |
| Python | 3.6+ |
| OpenCV | 4.x |
| CMake | 3.0+ |

Install ROS dependencies:

```bash
sudo apt-get update
sudo apt-get install ros-<distro>-cv-bridge ros-<distro>-image-transport python3-opencv
```

> Replace `<distro>` with your ROS distribution (e.g., `noetic`).

---

## 📁 Repository Structure

```
eyegaze/
├── src/                   # ROS packages (source code)
├── build/                 # CMake build output (generated)
├── devel/                 # Catkin development space (generated)
└── .catkin_workspace      # Catkin workspace marker
```

---

## 🔧 Installation

**1. Clone the repository:**

```bash
git clone https://github.com/AIbyHimanshu/eyegaze.git
cd eyegaze
```

**2. Source your ROS environment:**

```bash
source /opt/ros/<distro>/setup.bash
```

**3. Install any additional Python dependencies:**

```bash
pip3 install -r src/<package_name>/requirements.txt
```

---

## 🏗️ Building the Workspace

From the root of the cloned repository, run:

```bash
catkin_make
```

Then source the workspace:

```bash
source devel/setup.bash
```

To make this permanent, add it to your `.bashrc`:

```bash
echo "source ~/eyegaze/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Usage

**Launch the eye gaze tracking node:**

```bash
roslaunch eyegaze eyegaze.launch
```

**With a specific camera device:**

```bash
roslaunch eyegaze eyegaze.launch camera_id:=0
```

**Visualize gaze output in RViz:**

```bash
rviz
```

Then add the relevant ROS topics (e.g., `/gaze_direction`, `/eye_region`) to the display.

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

**Made with ❤️ by [AIbyHimanshu](https://github.com/AIbyHimanshu)**
