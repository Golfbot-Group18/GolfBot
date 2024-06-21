import requests
import cv2
import numpy as np

url = "http://IP:PORT/video_feed" #Change to appropriate feed



def fetch_frames():
    response = requests.get(url, stream=True)
    if response.status_code == 200:
        bytes_data = bytes()
        for chunk in response.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')
            if a != -1 and b != -1:
                frame = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                yield cv2.imdecode(np.fromstring(frame, np.uint8), cv2.IMREAD_UNCHANGED)


for frame in fetch_frames():
    cv2.imshow('Live Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()