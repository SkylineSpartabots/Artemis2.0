import cv2
from flask import Flask, Response, request

app = Flask(__name__)

def webcam_stream():
    cap = cv2.VideoCapture(0) #sets the camera (default, 0)

    while True: #take images from camera while true
        ret, frame = cap.read()
        if not ret:
            break #if camera not working break
        _, img_encoded = cv2.imencode('.jpg', frame) # encodes as .jpg
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(img_encoded) + b'\r\n') #return the images

    cap.release()

# Camera Stream Output
@app.route('/')
def index():
    return Response(webcam_stream(), mimetype='multipart/x-mixed-replace; boundary=frame') #defining response, webcam_stream is the function that actually contains the stream.

if __name__ == '__main__':
    app.run('0.0.0.0',debug=True, port="6969") #set the url to the device ip so outside devices can connect, sets debug to on. sets port to 6969