# import cv2

# video_path = 'data_color_z.avi'
# video_capture = cv2.VideoCapture(video_path)

# frame_counter = 0

# while video_capture.isOpened():
#     ret, frame = video_capture.read()
#     if not ret:
#         break

#     if frame_counter % 10 == 0:  # Change the interval as per your requirements
#         image_path = f'output/frame_{frame_counter}.jpg'
#         cv2.imwrite(image_path, frame)
#         print(frame_counter)

#     frame_counter += 1

# video_capture.release()




import cv2
import numpy as np

# Read the image
image = cv2.imread('output/frame_500.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the grayscale image to separate the black object from the background
_, threshold = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)

# Find contours in the thresholded image
contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Iterate over the contours and find the bounding box for the black object
# for contour in contours:
#     x, y, w, h = cv2.boundingRect(contour)
#     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

for contour in contours:
    # Calculate the contour area
    area = cv2.contourArea(contour)
    
    # Specify the minimum area threshold
    min_area_threshold = 300
    max_area_threshold = 50000
    
    # Check if the contour area is larger than the threshold
    if area > min_area_threshold and area < max_area_threshold:
        # Draw the bounding box
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
# Display the image with the bounding box
cv2.imshow('Bounding Box', image)
cv2.imshow('threshold', threshold)
cv2.waitKey(0)
cv2.destroyAllWindows()
