import cv2
import numpy as np
import math
import sys

# Import for finding hardware devices on Windows
try:
    import wmi
except ImportError:
    print("Error: The 'wmi' library is required for this script on Windows.")
    print("Please run: pip install wmi")
    sys.exit(1)

# --- Configuration ---
# Unique part of your webcam's Device Instance Path:
# Full Path: USB\VID_0C45&PID_64AB&MI_00\6&2B9D0D7D&0&0000
# We can use a unique portion of this ID to find the device.
TARGET_SERIAL_ID = "2B9D0D7D&0&0000"

# --- Helper Functions ---

def find_camera_index_by_serial(target_id):
    """
    Uses WMI to find the OpenCV index of a camera by its unique serial ID.
    This method is adapted from your provided script.
    """
    print("Searching for connected cameras...")
    c = wmi.WMI()
    
    # Get a list of all USB camera devices found by WMI
    wmi_devices = [dev for dev in c.Win32_PnPEntity(PNPClass="Camera") if "USB" in dev.DeviceID]

    if not wmi_devices:
        print("No USB cameras found by WMI.")
        return -1

    # Find the specific device that contains our target serial ID
    for index, dev in enumerate(wmi_devices):
        if target_id.lower() in dev.DeviceID.lower():
            print(f"✓ Found target camera '{dev.Name}' at assumed OpenCV index: {index}")
            # The key assumption: WMI's device order matches OpenCV's.
            return index
            
    print(f"❌ Error: Could not find a camera with serial ID containing '{target_id}'")
    return -1

def get_pointer_angle_and_line(frame):
    """Angle detection with 0-360° range, 0° pointing up, clockwise direction."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply stronger Gaussian blur to reduce noise and specs
    blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)
    
    # Adaptive thresholding with optimized parameters to reduce noise
    adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 15, 4)
    
    # Apply circular region of interest mask
    h, w = adaptive_thresh.shape
    center = (w//2, h//2)
    radius = min(w, h) // 4  # Adjust radius as needed
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    
    # Apply mask to threshold image
    thresh = cv2.bitwise_and(adaptive_thresh, mask)
    
    # Apply morphological operations to remove specs and clean up noise
    kernel_small = np.ones((2, 2), np.uint8)  # Smaller kernel for fine noise
    kernel_large = np.ones((3, 3), np.uint8)  # Larger kernel for general cleanup
    
    # Remove small noise specs
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel_small, iterations=2)
    # Fill small gaps
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_large, iterations=1)
    
    # Remove connected components smaller than threshold to eliminate tiny specs
    def remove_small_components(img, min_size=15):
        """Remove small connected components (specs) from binary image"""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
        # Create output image
        cleaned = np.zeros_like(img)
        # Keep components larger than min_size (skip background label 0)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_size:
                cleaned[labels == i] = 255
        return cleaned
    
    # Apply component filtering to remove tiny specs
    thresh = remove_small_components(thresh, min_size=15)

    # Use more restrictive parameters for line detection to avoid noise
    lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, threshold=25, minLineLength=30, maxLineGap=3)
    
    if lines is not None:
        # Filter lines by length and proximity to center
        valid_lines = []
        center_x, center_y = center
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line length
            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Calculate minimum distance from line to center
            # Distance from point to line formula
            A = y2 - y1
            B = x1 - x2
            C = x2 * y1 - x1 * y2
            dist_to_center = abs(A * center_x + B * center_y + C) / math.sqrt(A*A + B*B)
            
            # Only keep lines that are long enough and pass near the center
            if length > 25 and dist_to_center < 20:  # Adjust these thresholds as needed
                valid_lines.append((line, length))
        
        if valid_lines:
            # Get the longest valid line
            longest_line, _ = max(valid_lines, key=lambda x: x[1])
            x1, y1, x2, y2 = longest_line[0]
            
            # Calculate distances from both endpoints to center
            center_x, center_y = center
            dist1 = math.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
            dist2 = math.sqrt((x2 - center_x)**2 + (y2 - center_y)**2)
            
            # Use the point closest to center as start point
            if dist1 <= dist2:
                start_x, start_y = x1, y1
                end_x, end_y = x2, y2
            else:
                start_x, start_y = x2, y2
                end_x, end_y = x1, y1
            
            # Calculate vector from start to end
            dx = end_x - start_x
            dy = end_y - start_y
            
            # Calculate angle: atan2 gives -π to π, we want 0-360° with 0° pointing up
            angle_rad = math.atan2(dx, -dy)  # Note: swapped dx, dy and negated dy for "up" = 0°
            
            # Convert to 0-360° range
            angle_deg = math.degrees(angle_rad)
            if angle_deg < 0:
                angle_deg += 360
                
            return angle_deg, (start_x, start_y, end_x, end_y), thresh, center
    
    return None, None, thresh, (w//2, h//2)

# --- Main Application ---

# 1. Find the correct camera index using its hardware ID
cam_index = find_camera_index_by_serial(TARGET_SERIAL_ID)
if cam_index == -1:
    print("Could not initialize the target sensor. Exiting.")
    sys.exit(1)

# 2. Initialize the camera using the reliably found index
cap = cv2.VideoCapture(cam_index)
if not cap.isOpened():
    raise IOError(f"Cannot open webcam at found index {cam_index}")

print("\n--- Controls ---")
print("Press '0' to quit.")
print("----------------\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    key = cv2.waitKey(1) & 0xFF

    # Get pointer angle and line coordinates
    angle, line_coords, thresh, center = get_pointer_angle_and_line(frame)
    
    # Draw red dot at center of raw image
    cv2.circle(frame, center, 5, (0, 0, 255), -1)  # Red dot at center
    
    # Create a copy of the threshold image for overlay
    thresh_display = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    
    # Overlay detected line on the binary image
    if line_coords is not None:
        start_x, start_y, end_x, end_y = line_coords
        cv2.line(thresh_display, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)  # Green line on binary image
        
        # Draw a small blue circle at the start point (closest to center)
        cv2.circle(thresh_display, (start_x, start_y), 3, (255, 0, 0), -1)  # Blue dot at start
        
        # Overlay angle text on raw image only
        cv2.putText(frame, f"Angle: {angle:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "No pointer detected", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if key == ord('0'):
        break
        
    # Show both windows - raw image and binary image with line overlay
    cv2.imshow('Live Feed', frame)
    cv2.imshow('Preprocessed Image', thresh_display)

# Cleanup
cap.release()
cv2.destroyAllWindows()