import cv2
import numpy as np

# Initialize the camera (0 is usually the default camera module/USB cam)
cap = cv2.VideoCapture(0)

# Lower the resolution to improve performance on the Raspberry Pi
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print("Starting camera... Press 'q' to quit.")

while True:
    # 1. Read the current frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # 2. Convert the frame from BGR (standard) to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 3. Define the HSV range for RED
    # Red is tricky because it wraps around the hue spectrum (0-10 and 160-180)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # 4. Create masks to isolate the red color, then combine them
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 5. Clean up the mask (remove small noise/false positives)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 6. Find contours (outlines of the detected red blobs)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the largest contour by area (assuming the largest red object is our ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the minimum enclosing circle and its center point (X, Y)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        # Calculate moments to find the exact center of the contour
        M = cv2.moments(largest_contour)
        
        # Only proceed if the radius is large enough (filters out tiny red specks)
        if radius > 10 and M["m00"] > 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            
            # Draw a green circle around the ball and a blue dot at the center
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # The students will use center_x to steer left/right, and radius to know when to grab!

    # 7. Display the result (Useful for testing with a monitor attached)
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Red Mask", mask)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up and release the camera
cap.release()
cv2.destroyAllWindows()