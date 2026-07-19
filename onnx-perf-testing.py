import cv2
import time
import psutil
from picamera2 import Picamera2
from ultralytics import YOLO

model = YOLO("yolov8n.onnx")  # swap in your quantized model path
CAT_CLASS = 15
DETECT_INTERVAL = 5  # seconds between inference runs
MEM_WARNING_PCT = 85  # warn if system memory usage crosses this

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 640), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(1)  # let auto-exposure/white-balance settle

last_detect_time = 0

print("Starting. Ctrl+C to stop.\n")

try:
    while True:
        frame = picam2.capture_array()  # RGB888 numpy array, no ret/bool check needed

        now = time.time()

        if now - last_detect_time >= DETECT_INTERVAL:
            last_detect_time = now

            t0 = time.time()
            results = model(frame, classes=[CAT_CLASS], verbose=False)
            infer_ms = (time.time() - t0) * 1000

            cat_found = len(results[0].boxes) > 0

            cpu_pct = psutil.cpu_percent(interval=None)
            mem = psutil.virtual_memory()
            mem_pct = mem.percent
            mem_used_mb = mem.used / (1024 ** 2)
            mem_avail_mb = mem.available / (1024 ** 2)

            proc = psutil.Process()
            proc_mem_mb = proc.memory_info().rss / (1024 ** 2)

            print(
                f"[{time.strftime('%H:%M:%S')}] "
                f"cat={cat_found} | infer={infer_ms:.1f}ms | "
                f"CPU={cpu_pct:.1f}% | sys_mem={mem_pct:.1f}% "
                f"({mem_used_mb:.0f}MB used / {mem_avail_mb:.0f}MB avail) | "
                f"proc_mem={proc_mem_mb:.0f}MB"
            )

            if mem_pct >= MEM_WARNING_PCT:
                print(f"  ⚠️  Memory usage high ({mem_pct:.1f}%) — watch for swapping/OOM.")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    picam2.stop()