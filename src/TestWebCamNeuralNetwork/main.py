import cv2

def detect_cat(image):
    # Manually provide the correct path to the Haar cascade XML file
    cat_cascade = cv2.CascadeClassifier('haarcascade_frontalcatface.xml')
    
    # Convert the image to grayscale for better detection performance
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect cat faces in the grayscale image
    cats = cat_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))
    
    # Iterate through all detected cat faces
    for (x, y, w, h) in cats:
        # Draw a rectangle around each detected cat face
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    return image  # Return the processed image

# Open webcam
reader = cv2.VideoCapture(0)

while True:
    ok, frame = reader.read()
    if not ok:
        break
    
    # Process the frame to detect cats
    processed_frame = detect_cat(frame)
    
    # Display the processed frame
    cv2.imshow('Cat Detection', processed_frame)
    
    # Exit loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
reader.release()
cv2.destroyAllWindows()

