#!/usr/bin/env python3
"""
Test the landing email locally without running ROS.

Usage (from repo root):
    python3 test_scripts/test_landing_email.py

Reads credentials from config/tankervision.yaml.
Sends a realistic landing email to the configured recipient.
"""
import os
import sys
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'tanker_vision'))

from tanker_vision.send_email import send_email, EmailError  # noqa: E402

CFG_PATH      = os.path.join(REPO_ROOT, 'config', 'tankervision.yaml')
UNIT_CFG_HINT = os.path.join(REPO_ROOT, 'config')


def _load_cfg() -> dict:
    cfg = {}
    for path in [CFG_PATH]:
        with open(path) as f:
            cfg.update(yaml.safe_load(f) or {})
    # Try to load a unit config too (atlas/helios/argus) if present
    for name in ('atlas', 'helios', 'argus'):
        unit_path = os.path.join(UNIT_CFG_HINT, f'{name}.yaml')
        if os.path.exists(unit_path):
            with open(unit_path) as f:
                unit_cfg = yaml.safe_load(f) or {}
            cfg.update(unit_cfg)
            print(f'[test] Loaded unit config: {unit_path}')
            break
    return cfg


def main():
    cfg   = _load_cfg()
    notif = cfg.get('notifications', {})
    unit  = cfg.get('unit', {})
    mode  = cfg.get('mode', 'testing')

    name  = unit.get('name', 'unknown')
    plane = unit.get('plane_number', '?')

    creds = dict(
        username  = notif.get('gmail_user', ''),
        password  = notif.get('gmail_app_password', ''),
        sender    = notif.get('gmail_user', ''),
        recipient = notif.get('email_to', ''),
    )

    if not all(creds.values()):
        print('[test] ERROR: Missing email credentials in config/tankervision.yaml')
        sys.exit(1)

    # Fake session data — edit these to test different scenarios
    fake_session  = f'/mnt/storage/2025-07-15/session_14-32-01'
    fake_raw      = 47
    fake_triggers = 12
    fake_maxvis   = True  # change to False to test the "no signal" path

    maxvis_str = 'YES — signal detected this session' if fake_maxvis else 'NO — no signal detected'

    subject = f'TankerVision Landing Alert — {name} ({plane})'
    body = (
        f'The plane has landed.\n\n'
        f'=== SESSION SUMMARY ===\n'
        f'Session folder : {fake_session}\n'
        f'Raw images saved: {fake_raw}\n'
        f'Fire triggers   : {fake_triggers}\n'
        f'MaxVis active   : {maxvis_str}\n\n'
        f'=== NODE LOGS ===\n'
        f'[14:32:01] INFO status_node: StatusNode started | unit={name} plane={plane} mode={mode}\n'
        f'[14:32:03] INFO status_node: SESSION STARTED: {fake_session}\n'
        f'[14:32:05] INFO status_node: Internet: INTERNET_OK\n'
        f'[14:32:10] INFO status_node: MaxVis analog signal detected\n'
        f'[14:45:17] INFO status_node: Fire trigger #1: /save_images_trigger\n'
        f'[14:58:33] INFO status_node: Landing detected: 0.42 m/s\n\n'
        f'=== STORAGE USAGE ===\n'
        f'(test — no real disk mounted)\n'
    )

    print(f'[test] Sending to: {creds["recipient"]}')
    print(f'[test] Subject: {subject}')
    print(f'[test] From: {creds["sender"]}')

    try:
        send_email(subject, body, **creds)
        print('[test] Email sent successfully. Check your inbox.')
    except EmailError as e:
        print(f'[test] FAILED: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
