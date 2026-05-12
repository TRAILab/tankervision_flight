# tankervision_flight
Dev Location for the tankervision flight software 


# TankerVision Flight System

Aerial wildfire detection system running on Jetson-based flight computers.
Each unit runs Ubuntu 22.04 + ROS2 Humble, detects fire using a YOLO model,
and saves raw imagery and rosbags to a local mergerfs storage pool.

---

## Flight Units

| Unit   | Plane   | Province        |
|--------|---------|-----------------|
| helios | BD-124  | Alberta         |
| argus  | BD-122  | British Columbia|
| atlas  | BD-126  | Alberta         |

Unit identity is set per-machine in `config/<unit name>.yaml`.
Global Config (true for all flight units) is set in `config/tankervision.yaml`.
---

## Repository Layout

```
tankervision_flight/
├── config/
│   └── tankervision.yaml          ← single config file for everything
│   └──  argus.yaml                ← unit-specific config - only one is read by the unit
│   └──  atlas.yaml
│   └──  helios.yaml   
├── scripts/
│   ├── configure-dfg-camera.sh    ← applies NTSC/PAL settings to DFG2USB Pro
│   └── tankervision_login.sh      ← shows flight log on terminal login
├── src/
│   ├── arena_camera_node/         ← Lucid Triton2 C++ driver node
│   ├── tanker_vision/
│   │   ├── tanker_vision/
│   │   │   ├── status_node.py     ← startup banner, session folder, flight log, email
│   │   │   ├── record_data_node.py← YOLO fire detection + rosbag recording
│   │   │   └── fire_detection.pt  ← YOLO model (class 2 = fire, conf 0.3)
│   │   └── launch/
│   │       └── flight.launch.py   ← top-level launch file
│   ├── gnss_imu/
│   │   ├── save_gnss.sh           ← records raw GNSS serial to .ubx chunks
│   │   └── save_imu.sh            ← records IMU rosbag via ROS2
│   └── im19_ros2/
│       └── launch/sensors.launch.py ← launches im19_mems_node
├── startup_scripts/
│   ├── tankervision.service       ← main systemd service
│   ├── ptp4l.service              ← PTP master
│   ├── gpsd-chrony.service        ← GPS time sync
│   └── gnss-record.service        ← GNSS recording (continuous)
├── install.sh                     ← first-time machine setup
└── update.sh                      ← deploy repo changes to this machine
```

---

## Configuration — `config/tankervision.yaml` and `config/<unit_name>.yaml`

This is the **single source of truth** for all system parameters.
Edit this file to change mode, camera settings, YOLO thresholds, etc.


```yaml
mode: flight        # testing | flight  ← CHANGE THIS BEFORE FLIGHT

unit:
  name: atlas       # eg
  plane_number: BD-126
  province: Alberta

```

```yaml


notifications:
  email_to: wildfire@robotics.utias.utoronto.ca
  gmail_user: wildfire@robotics.utias.utoronto.ca
  gmail_app_password: <app-password>
  msmtprc: /home/atlas/.msmtprc

storage:
  root: /mnt/storage
  saved_frames_dir: /mnt/storage/saved_frames

trigger:
  topic: /save_images_trigger
  timeout_sec: 20       # record for 20s after last fire detection
  cooldown_sec: 5       # wait 5s before accepting new trigger

yolo:
  model: src/tanker_vision/tanker_vision/fire_detection.pt
  fire_class: 2         # class index for fire in the model
  confidence: 0.3       # detection confidence threshold

analog_camera:
  device: /dev/video0
  standard: NTSC        # NTSC | PAL

ptp:
  interface: auto
```

### Mode: testing vs flight

| Behaviour                  | testing       | flight              |
|---------------------------|---------------|---------------------|
| Session folder created    | at startup    | at startup          |
| Recording node launched   | no            | yes                 |
| PTP/clock check           | warn only     | required to start   |
| Trigger cooldown          | no            | yes                 |

---

## Storage Layout

All data for a session lands in one folder:

```
/mnt/storage/
└── YYYY-MM-DD/
    └── session_HH-MM-SS/         ← created by status_node on startup
        ├── flight_log.txt         ← black box event log (status_node)
        ├── saved_frames/          ← PNG previews (camera_viewer_node)
        │   ├── lucid_<ts>.png
        │   └── analog_<ts>.png
        ├── fire_<ts>/             ← rosbag on fire detection (record_data_node)
        │   └── fire_<ts>.mcap
        └── ros_log/               ← full ROS log copied on shutdown (status_node)
```

Raw Lucid frames from `arena_camera_node` are written directly into the
session root:
```
session_HH-MM-SS/
└── frame_<id>_<ts>_5320x4600_16bpp.raw
```

GNSS data is written continuously to dated folders (managed by `gnss-record.service`):
```
/mnt/storage/
└── YYYY-MM-DD/
    └── gnss_<ts>.ubx              ← raw GNSS serial (30-min chunks)
```

### Checking what was saved

```bash
# List today's sessions
ls /mnt/storage/$(date +%Y-%m-%d)/

# See everything in the latest session
ls -la /mnt/storage/$(date +%Y-%m-%d)/session_*/

# Watch a session fill up in real time
watch -n 2 'ls -lh /mnt/storage/$(date +%Y-%m-%d)/session_*/'

# Read the live flight log
tail -f /mnt/storage/$(date +%Y-%m-%d)/session_*/flight_log.txt

# Check rosbag info
ros2 bag info /mnt/storage/$(date +%Y-%m-%d)/session_*/fire_*/
```

---

## Data Streams

| Stream              | Node / Script          | Format      | When             | Location                        |
|--------------------|------------------------|-------------|------------------|---------------------------------|
| Lucid raw frames   | arena_camera_node      | .raw binary | On fire trigger  | session_folder/                 |
| Lucid + analog PNG | camera_viewer_node     | .png        | On save button/s | session_folder/saved_frames/    |
| Rosbag             | record_data_node       | .mcap       | On fire trigger  | session_folder/fire_<ts>/       |
| GNSS raw           | gnss-record.service    | .ubx        | Continuous       | YYYY-MM-DD/gnss_<ts>.ubx        |
| IMU rosbag         | save_imu.sh            | .mcap       | Continuous       | session_folder/imu_<ts>/        |
| Flight log         | status_node            | .txt        | Continuous       | session_folder/flight_log.txt   |
| ROS log            | status_node (shutdown) | directory   | On shutdown      | session_folder/ros_log/         |

### Rosbag topics recorded on fire detection

| Topic             | Content                        | Always present? |
|------------------|--------------------------------|-----------------|
| /cam0/image_raw  | Downsampled Triton2 (BGR8)     | Yes             |
| /cam1/image_raw  | MaxVis analog (YUYV)           | If signal active|
| /im19/imu        | IMU data                       | If IMU connected|

---

## Nodes and Topics

### Published topics

| Topic                        | Type              | Publisher           | Notes                        |
|-----------------------------|-------------------|---------------------|------------------------------|
| /arena_camera_node/images   | sensor_msgs/Image | arena_camera_node   | Downsampled Triton2 stream   |
| /cam1/image_raw             | sensor_msgs/Image | analog_camera       | MaxVis analog stream         |
| /save_images_trigger        | std_msgs/Empty    | record_data_node    | Fires on each detected frame |
| /tankervision/session_path  | std_msgs/String   | status_node         | Latched — current session dir|
| /camera_heartbeat           | std_msgs/String   | arena_camera_node   | Heartbeat                    |
| /camera/lucid_diagnostics   | std_msgs/String   | arena_camera_node   | Exposure, gain, PTP status   |
| /record_data/status         | std_msgs/String   | record_data_node    | SCANNING / RECORDING / COOLDOWN |

### Subscribed topics (key)

| Node               | Subscribes to                 | Purpose                          |
|-------------------|-------------------------------|----------------------------------|
| record_data_node  | /cam0/image_raw               | YOLO inference on each frame     |
| record_data_node  | /tankervision/session_path    | Where to write rosbags           |
| arena_camera_node | /save_images_trigger          | Save full-res raw frame to disk  |
| arena_camera_node | /tankervision/session_path    | Where to write raw frames        |
| status_node       | /rosout                       | Capture node errors to flight log|
| status_node       | /save_images_trigger          | Log each trigger event           |

---

## Startup Sequence

On boot, systemd brings services up in this order:

```
udev rules loaded
  → mnt-storage.mount          (mergerfs pool: /mnt/hdd0 + /mnt/hdd1 + /mnt/hdd2)
  → ptp4l.service              (PTP grandmaster on camera ethernet interface)
  → gpsd-chrony.service        (GPS time sync)
  → gnss-record.service        (GNSS raw recording starts — waits for /dev/gnss_raw)
  → tankervision.service
      ExecStartPre: configure-dfg-camera.sh  (sets NTSC on /dev/video0)
      ExecStart: ros2 launch tanker_vision flight.launch.py
          → status_node        (creates session folder, prints banner, sends email)
          → arena_camera_node  (Lucid Triton2 — waits for PTP slave, then streams)
          → analog_camera      (v4l2 DFG2USB Pro on /dev/video0)
          → im19_mems_node     (IMU — waits for /dev/im19_mems)
          → record_data_node   (flight mode only — YOLO fire detection)
```

---

## On-Login Terminal Display

Add to `~/.bashrc` on each unit to show the flight log on every terminal login:

```bash
source ~/tankervision_flight/scripts/tankervision_login.sh
```

Or install the autostart desktop entry to open a monitoring terminal on login:
```bash
cp startup_scripts/tankervision-monitor.desktop ~/.config/autostart/
```

---

## Checking System Status

```bash
# Are all services running?
systemctl status tankervision.service ptp4l.service gnss-record.service --no-pager

# Are all ROS nodes running?
ros2 node list

# Is mergerfs mounted?
mountpoint /mnt/storage && df -h /mnt/storage

# Is the Lucid camera connected and streaming?
ros2 topic hz /arena_camera_node/images

# Is MaxVis analog signal present?
v4l2-ctl --device=/dev/video0 --get-input

# Is PTP working?
journalctl -u ptp4l.service -n 10 --no-pager

# Is chrony synced?
chronyc tracking

# Check the live flight log
tail -f $(ls -t /mnt/storage/*/*/flight_log.txt 2>/dev/null | head -1)

# Check email was sent
cat ~/.msmtp.log | tail -5
```

---

## Deploying Changes to a Unit

After making changes to the repo on your Mac:

```bash
# On the Mac — commit and push
git add -A && git commit -m "your message" && git push

# On each flight unit
cd ~/tankervision_flight
git pull
sudo ./update.sh
```

`update.sh` will:
1. Deploy configs (chrony, gpsd, ptp, udev)
2. Deploy systemd service files
3. Reload systemd and restart enabled services
4. Rebuild the ROS2 workspace
5. Print a summary of service and node status

---

## First-Time Setup on a New Unit

```bash
# 1. Clone the repo
git clone https://github.com/TRAILab/tankervision_flight.git
cd tankervision_flight

# 2. Run install (apt packages, systemd enables, nmcli network, udev rules)
sudo bash install.sh

# 3. Configure msmtp for email notifications
sudo bash startup_scripts/setup_msmtp.sh config/tankervision.yaml

# 4. Edit tankervision.yaml with unit-specific values
nano config/tankervision.yaml   # set name, plane_number, province, msmtprc path

# 5. Build ROS workspace
colcon build --symlink-install

# 6. Reboot and verify
sudo reboot
# After reboot:
systemctl status tankervision.service
ros2 node list
ls /mnt/storage/$(date +%Y-%m-%d)/
```

---

## Switching to Flight Mode

Before a real flight, edit `config/tankervision.yaml`:

```yaml
mode: flight   # ← change from testing
```

Then restart:
```bash
sudo systemctl restart tankervision.service
```

In flight mode:
- `record_data_node` launches and runs YOLO on every Lucid frame
- Fire detection publishes `/save_images_trigger` and starts a rosbag
- Rosbag records for `trigger.timeout_sec` seconds after last detection
- A `trigger.cooldown_sec` wait applies before the next recording can start

Switch back to `mode: testing` for bench work — no rosbag, no YOLO inference.

---

## Troubleshooting

### tankervision.service not starting
```bash
journalctl -u tankervision.service -n 50 --no-pager -l | grep -v im19
```

### No session folder created
```bash
# Is status_node running?
ros2 node list | grep status
# Is storage mounted?
mountpoint /mnt/storage
```

### No startup email received
```bash
cat ~/.msmtp.log
# Check spam folder — subject: "TankerVision Online — <unit> (<plane>)"
```

### No rosbag on fire detection
```bash
# Is record_data_node running? (flight mode only)
ros2 node list | grep record
# Is Lucid streaming?
ros2 topic hz /arena_camera_node/images
# Lower confidence temporarily to test
# config/tankervision.yaml: confidence: 0.01
```

### MaxVis showing as offline when camera is connected
```bash
v4l2-ctl --device=/dev/video0 --get-input
# Should show signal status — if "no signal" appears, no analog input
```

### PTP not locking
```bash
journalctl -u ptp4l.service --no-pager | tail -20
# Camera must be on the same ethernet segment as ptp4l interface
```
