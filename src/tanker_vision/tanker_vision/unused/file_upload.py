import os
import time
import subprocess
import threading
import queue
import shutil

# Configuration
LOCAL_DIR = "/media/trail/Wildfire1/Downloads"  # Folder to monitor
REMOTE_USER = "rob501"
REMOTE_HOST = "trail.utias.utoronto.ca"  # Remote desktop IP or hostname
REMOTE_PATH = "trailab_datasets"
CHECK_INTERVAL = 5  # Time in seconds between folder checks
NUM_WORKERS = 4     # Number of upload threads
PASSWORD = "501TAsrule"

# Create a queue to manage files
file_queue = queue.Queue()

def upload_worker():
    """Worker thread function to process the file upload queue."""
    while True:
        file_path = file_queue.get()
        if file_path is None:
            # A 'None' is our signal to shut down this worker
            file_queue.task_done()
            break

        print(f"[Worker] Uploading: {file_path}")
        try:
            result = subprocess.run(
                ["/usr/bin/sshpass", "-p", PASSWORD, "scp", "-r",
                 file_path, f"{REMOTE_USER}@{REMOTE_HOST}:{REMOTE_PATH}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            if result.returncode == 0:
                print(f"[Worker] Upload successful! {file_path}")
                return
                # Remove local file if desired:
                # os.remove(file_path)
            else:
                print("[Worker] Upload failed with return code:", result.returncode)
                print("[Worker] Stdout:", result.stdout)
                print("[Worker] Stderr:", result.stderr)

        except Exception as e:
            print(f"[Worker] Exception during upload of {file_path}: {e}")

        # Mark the current file as processed (successful or not)
        file_queue.task_done()

def monitor_folder():
    """
    Continuously checks LOCAL_DIR for new folders or .zip files.
    - If it finds a folder or file not ending in `.zip`, it will zip it
      and remove the original folder.
    - Then it puts the .zip file into the queue for uploading.
    """
    # seen = set()
    items = os.listdir(LOCAL_DIR)

    for item in items:
        full_path = os.path.join(LOCAL_DIR, item)
        
        # Skip if we've handled this exact path before
        # if full_path in seen:
        #     continue

        # If it's already a .zip, just queue it
        if not full_path.endswith(".zip"):
            print(f"[Monitor] Found new zip: {full_path}")
            file_queue.put(full_path)
            # seen.add(full_path)
            upload_worker()
            print(f"[Monitor] Zipping {full_path}")
            # Make the zip archive (archive path will be `full_path + '.zip'`)
            # but if `full_path` is itself a folder, we name the zip accordingly:
            zip_base = full_path  # e.g., /path/to/folder
            shutil.make_archive(zip_base, 'zip', full_path)
            # Now remove the original folder or file
            if os.path.isdir(full_path):
                shutil.rmtree(full_path)
            else:
                os.remove(full_path)

            # zipped_path = full_path + ".zip"
            # print(f"[Monitor] Queueing zip for upload: {zipped_path}")
            # file_queue.put(zipped_path)
            # seen.add(zipped_path)
                    

        # If the folder is empty, you might decide to exit
        # if not os.listdir(LOCAL_DIR):
        #     print("[Monitor] No more items found in the folder. Exiting monitor.")
        #     break

        # Wait before checking again
        # time.sleep(CHECK_INTERVAL)

def start_upload_service():
    """Starts the upload service as background threads."""
    print("[Main] Starting file upload service...")

    # Start worker threads
    workers = []
    #for _ in range(NUM_WORKERS):
        #thread = threading.Thread(target=upload_worker, daemon=True)
        #thread.start()
        #workers.append(thread)
    monitor_folder()

    # Start monitoring in a separate daemon thread
    #monitor_thread = threading.Thread(target=monitor_folder, daemon=True)
    #monitor_thread.start()

