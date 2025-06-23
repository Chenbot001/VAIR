import time
import cv2
import numpy as np
from dmrobotics import Sensor, put_arrows_on_image

if __name__ == "__main__":
    dev_serial_id = 0 
    sensor = Sensor(dev_serial_id)  # serial ID
    frame_num = 0.0
    start_time = time.time()
    black_img = np.zeros_like(sensor.getRawImage()) 
    black_img = np.stack([black_img]*3, axis=-1) 

    # Initialize baseline values for max intensity and pixel sum
    baseline_depth = sensor.getDepth()
    baseline_max_intensity = np.max(baseline_depth)
    baseline_pixel_sum = np.sum(baseline_depth)

    while True:
        img = sensor.getRawImage()
        frame_num += 1.0
        deformation = sensor.getDeformation2D()
        shear = sensor.getShear()

        depth = sensor.getDepth() # output the deformed depth
        depth_img = cv2.applyColorMap((depth*0.25* 255.0).astype('uint8'), cv2.COLORMAP_HOT)

        # Adjust depth values by subtracting baseline
        adjusted_depth = depth - baseline_depth

        # Find the pixel with the highest intensity
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(adjusted_depth)
        # Calculate the sum of pixel values in the adjusted depth image
        pixel_sum = np.sum(adjusted_depth)
        print(f"Max Intensity: {max_val:.2f} | Pixel Sum: {pixel_sum:.2f}     ", end='\r')

        cv2.imshow('depth', depth_img)
        # cv2.imshow('img', img)
        # cv2.imshow('deformation', put_arrows_on_image(black_img, deformation*20))
        # cv2.imshow('shear', put_arrows_on_image(black_img, shear*20))


        k = cv2.waitKey(3)
        if k & 0xFF == ord('q'):
            break
        elif k & 0xFF == ord('r'):
            sensor.reset()
            baseline_depth = sensor.getDepth()
            baseline_max_intensity = np.max(baseline_depth)
            baseline_pixel_sum = np.sum(baseline_depth)
            print("Sensor reset and baseline updated")

        if time.time() - start_time > 1.0:
            fps = frame_num / (time.time() - start_time)
            print("Output FPS is: {:.2f}".format(fps),
                  end='\r')
            frame_num = 0.0
            start_time = time.time()

    sensor.disconnect()
    cv2.destroyAllWindows()
