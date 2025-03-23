from ultralytics import YOLO
import cv2
import numpy as np
from typing import Tuple, List

class YOLODetector:
    def __init__(self, model_path: str, conf_threshold: float = 0.5):
        """
        Initialize YOLOv8 detector
        Args:
            model_path: Path to the YOLOv8 model file (.pt)
            conf_threshold: Confidence threshold for detections
        """
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

    def detect_image(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        Detect objects in an image
        Args:
            image: Input image in BGR format (OpenCV)
        Returns:
            Tuple containing detection results and annotated image
        """
        # Run inference
        results = self.model(image)[0]
        
        # Process results
        detections = []
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, class_id = result
            
            if conf >= self.conf_threshold:
                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'confidence': float(conf),
                    'class_id': int(class_id),
                    'class_name': results.names[int(class_id)]
                })

        # Draw detections
        annotated_image = image.copy()
        for det in detections:
            bbox = det['bbox']
            label = f"{det['class_name']} {det['confidence']:.2f}"
            
            # Draw rectangle
            cv2.rectangle(annotated_image, 
                        (bbox[0], bbox[1]), 
                        (bbox[2], bbox[3]), 
                        (0, 255, 0), 2)
            
            # Draw label
            cv2.putText(annotated_image, label, 
                       (bbox[0], bbox[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (0, 255, 0), 2)

        return detections, annotated_image

def main():
    # Initialize detector
    detector = YOLODetector(
        model_path='yolov8n.pt',  # Use YOLOv8 nano model
        conf_threshold=0.5
    )
    
    # Open video capture (0 for webcam)
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Run detection
        detections, annotated_frame = detector.detect_image(frame)
        
        # Print detections
        for det in detections:
            print(f"Detected {det['class_name']} with confidence {det['confidence']:.2f}")
        
        # Show results
        cv2.imshow('YOLOv8 Detection', annotated_frame)
        
        # Break loop on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()