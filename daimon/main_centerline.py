import time
import cv2
import numpy as np
from dmrobotics import Sensor, put_arrows_on_image

def compute_centerline_angle(centerline_points):
    """
    Compute the angle offset of the centerline from vertical (0 degrees baseline).
    
    Args:
        centerline_points: Array of points along the centerline
        
    Returns:
        angle_offset: Absolute angle offset from vertical in degrees, or None if no centerline
    """
    if centerline_points is None or len(centerline_points) < 2:
        return None
    
    # Use first and last points to determine the overall line direction
    start_point = centerline_points[0]
    end_point = centerline_points[-1]
    
    # Calculate direction vector
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    
    # Calculate angle from vertical (negative y-axis is up in image coordinates)
    # atan2 gives angle from positive x-axis, we want angle from negative y-axis (vertical up)
    angle_from_horizontal = np.arctan2(dy, dx)  # Angle from positive x-axis
    angle_from_vertical = angle_from_horizontal - np.pi/2  # Convert to angle from vertical
    
    # Convert to degrees
    angle_degrees = np.degrees(angle_from_vertical)
    
    # Normalize to [-90, 90] range and take absolute value
    # This ensures both CW and CCW deviations are positive
    if angle_degrees > 90:
        angle_degrees = 180 - angle_degrees
    elif angle_degrees < -90:
        angle_degrees = -180 - angle_degrees
    
    # Return absolute value for positive offset regardless of direction
    return abs(angle_degrees)

def detect_centerline(depth_data, contact_threshold=0.15):
    """
    Detect centerline from depth data by finding contours above threshold and fitting a line.
    
    Args:
        depth_data: The raw depth data array
        contact_threshold: Intensity threshold for contact detection (default: 0.15)
        
    Returns:
        centerline_points: Array of points along the centerline, or None if no centerline found
        largest_contour: The largest contour found, or None
    """
    # Create binary mask for pixels above threshold
    contact_mask = (depth_data > contact_threshold).astype(np.uint8) * 255
    
    # Find contours
    contours, _ = cv2.findContours(contact_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return None, None
    
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Check if contour is large enough to be meaningful
    if cv2.contourArea(largest_contour) < 10:  # Minimum area threshold
        return None, None
    
    # Fit a line to the contour points
    # Reshape contour points for cv2.fitLine
    contour_points = largest_contour.reshape(-1, 2)
    
    if len(contour_points) < 5:  # Need minimum points for line fitting
        return None, None
    
    # Fit line using least squares
    [vx, vy, x0, y0] = cv2.fitLine(contour_points, cv2.DIST_L2, 0, 0.01, 0.01)
    
    # Generate centerline points along the fitted line
    # Get bounding box of the contour to determine line extent
    x_min, y_min, w, h = cv2.boundingRect(largest_contour)
    x_max = x_min + w
    y_max = y_min + h
    
    # Calculate line parameters
    if abs(vx) > 1e-6:  # Avoid division by zero
        # Calculate t values for line boundaries
        t1 = (x_min - x0) / vx
        t2 = (x_max - x0) / vx
        t_min = min(t1, t2)
        t_max = max(t1, t2)
        
        # Generate points along the line
        t_values = np.linspace(t_min, t_max, 50)
        centerline_points = np.column_stack([
            x0 + t_values * vx,
            y0 + t_values * vy
        ])
    else:
        # Vertical line case
        centerline_points = np.column_stack([
            np.full(50, x0),
            np.linspace(y_min, y_max, 50)
        ])
    
    # Filter points to stay within image bounds
    h_img, w_img = depth_data.shape
    valid_mask = (
        (centerline_points[:, 0] >= 0) & 
        (centerline_points[:, 0] < w_img) &
        (centerline_points[:, 1] >= 0) & 
        (centerline_points[:, 1] < h_img)
    )
    centerline_points = centerline_points[valid_mask]
    
    if len(centerline_points) < 2:
        return None, None
    
    return centerline_points.astype(int), largest_contour

def add_angle_offset_overlay(depth_img, angle_offset):
    """
    Add angle offset value overlay to the depth image.
    
    Args:
        depth_img: The colorized depth image (BGR format)
        angle_offset: The angle offset from vertical in degrees, or None if no centerline
        
    Returns:
        The depth image with angle offset value overlaid
    """
    # Create a copy of the depth image to avoid modifying the original
    depth_img_with_overlay = depth_img.copy()
    
    # Prepare the text to display
    if angle_offset is not None:
        # Round to nearest 10 degrees for better readability - remove for exact angle
        rounded_angle = round(angle_offset / 10) * 10
        angle_text = f"Angle Offset: {rounded_angle:.0f} degrees" 
    else:
        angle_text = "Angle Offset: N/A"
    
    # Set text properties
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    color = (255, 255, 255)  # White color in BGR
    thickness = 1
    
    # Get text size to position it properly
    (text_width, text_height), baseline = cv2.getTextSize(angle_text, font, font_scale, thickness)
    
    # Position the text in the top-left corner with some padding
    x = 10
    y = text_height + 15
    
    # Add a semi-transparent background rectangle for better readability
    overlay = depth_img_with_overlay.copy()
    cv2.rectangle(overlay, (x-5, y-text_height-5), (x+text_width+5, y+baseline+5), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, depth_img_with_overlay, 0.4, 0, depth_img_with_overlay)
    
    # Add the text
    cv2.putText(depth_img_with_overlay, angle_text, (x, y), font, font_scale, color, thickness)
    
    return depth_img_with_overlay

def add_centerline_overlay(depth_img, centerline_points, largest_contour=None):
    """
    Add centerline and contour overlay to the depth image.
    
    Args:
        depth_img: The depth image (BGR format)
        centerline_points: Array of points along the centerline
        largest_contour: The contour from which centerline was derived
        
    Returns:
        The depth image with centerline overlaid
    """
    depth_img_with_centerline = depth_img.copy()
    
    if centerline_points is not None and len(centerline_points) > 1:
        # Draw the centerline
        for i in range(len(centerline_points) - 1):
            pt1 = tuple(centerline_points[i])
            pt2 = tuple(centerline_points[i + 1])
            cv2.line(depth_img_with_centerline, pt1, pt2, (0, 255, 0), 3)  # Green line
        
        # Optionally draw the contour outline
        if largest_contour is not None:
            cv2.drawContours(depth_img_with_centerline, [largest_contour], -1, (0, 255, 255), 2)  # Yellow contour
    
    return depth_img_with_centerline

if __name__ == "__main__":
    dev_serial_id = 0 
    sensor = Sensor(dev_serial_id)  # serial ID
    frame_num = 0.0
    start_time = time.time()
    black_img = np.zeros_like(sensor.getRawImage()) 
    black_img = np.stack([black_img]*3, axis=-1) 


    while True:
        img = sensor.getRawImage()
        frame_num += 1.0
        deformation = sensor.getDeformation2D()
        shear = sensor.getShear()

        depth = sensor.getDepth() # output the deformed depth
        depth_img = cv2.applyColorMap((depth*0.25* 255.0).astype('uint8'), cv2.COLORMAP_HOT)
        
        # Detect centerline from depth data
        centerline_points, largest_contour = detect_centerline(depth, contact_threshold=0.15)
        
        # Compute angle offset from vertical
        angle_offset = compute_centerline_angle(centerline_points)
        
        # Add centerline overlay if detected
        if centerline_points is not None:
            depth_img = add_centerline_overlay(depth_img, centerline_points, largest_contour)
        
        # Add angle offset overlay to the depth image
        depth_img_with_overlay = add_angle_offset_overlay(depth_img, angle_offset)

        cv2.imshow('depth', depth_img_with_overlay)
        cv2.imshow('deformation', put_arrows_on_image(black_img, deformation*20))
        cv2.imshow('shear', put_arrows_on_image(black_img, shear*20))


        k = cv2.waitKey(3)
        if k & 0xFF == ord('q'):
            break
        elif k & 0xFF == ord('r'):
            sensor.reset()
            print("Sensor reset")
        if time.time() - start_time > 1.0:
            fps = frame_num / (time.time() - start_time)
            print("Output FPS is: {:.2f}".format(fps),
                  end='\r')
            frame_num = 0.0
            start_time = time.time()

    sensor.disconnect()
    cv2.destroyAllWindows()
