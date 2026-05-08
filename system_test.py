#!/usr/bin/env python3
"""
tankervision_system_test.py
Tests: clock sync, internet/email, hardware trigger switch,
       recording node running, fire detection pipeline,
       correct session folder, recording start/stop.
Run with: python3 tankervision/system_test.py
"""
import subprocess
import time
import os
import sys

PASS = '\033[92m✓\033[0m'
FAIL = '\033[91m✗\033[0m'
SKIP = '\033[93m~\033[0m'

def check(label, ok, note=''):
    sym = PASS if ok else FAIL
    print(f'  {sym} {label}' + (f'  ({note})' if note else ''))
    return ok

def run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return -1, 'timeout'

print('\n=== TankerVision System Test ===\n')

# ── a. Clock sync ─────────────────────────────────────────────────────────────
print('[a] Clock sync')
rc, out = run(['chronyc', 'sources'])
ntp_ok = rc == 0 and any(c in out for c in ['^*', '^+'])
pps_ok = 'PPS' in out and rc == 0
check('NTP source selected', ntp_ok)
check('PPS source present in chrony', pps_ok,
      'expected False indoors — Ian tests outdoors tomorrow')
year_ok = int(time.strftime('%Y')) >= 2024
check('System clock is sane (year >= 2024)', year_ok)

# ── b. Internet / startup email ───────────────────────────────────────────────
print('\n[b] Internet connectivity')
import socket
try:
    socket.create_connection(('www.google.com', 80), timeout=2)
    inet_ok = True
except OSError:
    inet_ok = False
check('Internet reachable', inet_ok)
if inet_ok:
    rc, out = run(['journalctl', '-u', 'tankervision.service',
                   '--no-pager', '-n', '100'])
    email_ok = 'Email sent: TankerVision Online' in out
    check('Startup email sent', email_ok)
else:
    print(f'  {SKIP} Startup email (no internet)')

# ── c. Cellular off on takeoff ────────────────────────────────────────────────
print('\n[c] Cellular management')
print(f'  {SKIP} Cellular off on takeoff — stub only, velocity source not wired')

# ── d. Hardware trigger switch on PPS ─────────────────────────────────────────
print('\n[d] Hardware trigger')
rc, out = run(['ros2', 'param', 'get', '/arena_camera_node', 'hardware_trigger'],
              timeout=10)
hw_trigger = 'true' in out.lower() or 'True' in out
check('arena_camera_node running', rc == 0)
check('hardware_trigger param readable', rc == 0)
# Note: will be False until PPS acquired outdoors
check('hardware_trigger state logged',
      'hardware_trigger' in
      subprocess.run(['journalctl', '-u', 'tankervision.service',
                      '--no-pager', '-n', '200'],
                     capture_output=True, text=True).stdout)

# ── e. Recording node running ─────────────────────────────────────────────────
print('\n[e] Recording node')
rc, out = run(['ros2', 'node', 'list'], timeout=10)
rec_running = '/record_data_node' in out
check('record_data_node in node list', rec_running)
rc2, out2 = run(['ros2', 'topic', 'echo', '--once',
                 '--timeout', '3', '/record_data/status'], timeout=8)
check('/record_data/status publishing', rc2 == 0 or 'SCANNING' in out2 or 'RECORDING' in out2)

# ── f. YOLO / fire detection pipeline ────────────────────────────────────────
print('\n[f] Fire detection pipeline')
rc, out = run(['ros2', 'topic', 'hz', '--window', '5', '/cam0/image_raw'], timeout=12)
cam_flowing = rc == 0 and 'average rate' in out
check('/cam0/image_raw flowing', cam_flowing)
rc, out = run(['ros2', 'topic', 'info', '/save_images_trigger'], timeout=8)
check('/save_images_trigger topic exists', rc == 0)

# ── g/h. Recording starts to correct folder on fire trigger ──────────────────
print('\n[g/h] Fire trigger → recording → correct folder')
session_path = None
try:
    with open('/tmp/tankervision_session_path') as f:
        session_path = f.read().strip()
except FileNotFoundError:
    pass
check('Session path file exists (/tmp/tankervision_session_path)',
      session_path is not None, session_path or 'missing')
if session_path:
    check('Session folder exists on disk', os.path.isdir(session_path))
    check('saved_frames subfolder exists',
          os.path.isdir(os.path.join(session_path, 'saved_frames')))

# Manually fire a trigger and check for rosbag
print('  → Publishing fire trigger...')
run(['ros2', 'topic', 'pub', '--once',
     '/save_images_trigger', 'std_msgs/msg/Empty', '{}'], timeout=5)
time.sleep(3)
bag_started = False
if session_path:
    contents = os.listdir(session_path)
    bag_started = any('fire_' in c for c in contents)
check('Rosbag folder created after trigger', bag_started,
      str(contents) if session_path else 'no session path')

# ── i. Recording stops after timeout ─────────────────────────────────────────
print('\n[i] Recording stop (timeout)')
print('  → Waiting for recording timeout (checks every 2s for up to 30s)...')
stopped = False
for _ in range(15):
    time.sleep(2)
    rc, out = run(['ros2', 'topic', 'echo', '--once',
                   '--timeout', '2', '/record_data/status'], timeout=5)
    if 'SCANNING' in out:
        stopped = True
        break
check('Recording stopped and status returned to SCANNING', stopped)

# ── j/k/l. Landing detection, cellular on, landing email ─────────────────────
print('\n[j/k/l] Landing detection / cellular / landing email')
print(f'  {SKIP} Landing detection — velocity_callback not subscribed (no velocity source)')
print(f'  {SKIP} Cellular on landing — depends on landing detection')
print(f'  {SKIP} Landing email — depends on landing detection')

# ── Summary ───────────────────────────────────────────────────────────────────
print('\n=== Done ===')
print('Items marked ~ are known stubs blocked on:')
print('  • Velocity source for takeoff/landing detection (IM19 fused velocity or GPS speed)')
print('  • Cellular modem accessible (test outdoors with signal)')
print('  • Real PPS lock (Ian outdoor GPS test tomorrow)\n')