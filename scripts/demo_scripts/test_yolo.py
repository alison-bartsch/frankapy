import cv2
import torch
from pathlib import Path
from ultralytics import YOLO

model = YOLO('last.pt')  # load a custom model
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model.to(device)
video_path = 'data_color.avi'
cap = cv2.VideoCapture(video_path)
fps = int(cap.get(cv2.CAP_PROP_FPS))

output_path = 'output_video.avi'
output_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'XVID'), fps, output_size)

# Process each frame of the video
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame from BGR to RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Perform object detection on the frame
    results = model(frame)
    boxes = results[0].boxes
    if (len(results) > 1):
        print("more than 1")
    for i in range(boxes.shape[0]):
        box = boxes[i].xyxy.tolist()[0]
        # print(box)
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Write the frame to the output video
    out.write(frame)

    # Display the frame (optional)
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and writer objects
cap.release()
out.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
