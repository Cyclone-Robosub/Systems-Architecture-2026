from flask import Flask, Response
import cv2

app = Flask(__name__)
cap = cv2.VideoCapture(0)  
def gen():
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        ok, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')

@app.route('/video')
def video():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '<img src="/video" />'

if __name__ == '__main__':

    app.run(host="127.0.0.1", port=5000, threaded=True)