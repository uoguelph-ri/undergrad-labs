#!/bin/python2

import requests
import cv2

def detect_toy(image):
    detector_url = 'http://10.16.48.143:8877/detector/detect'
    image_encoded = cv2.imencode(".jpg", image)[1]
    files = {'image': ('image.jpg', image_encoded.tostring(), 'image/jpeg', {'Expires': '0'})}
    response = requests.post(detector_url, files=files)
    result = response.json()
    categories = result['coco']['categories']
    
    if len(result['coco']['annotations']) > 0:
        annotation = result['coco']['annotations'][0]
        return {
            'animal': (next((item for item in categories if item["id"] == annotation.get('category_id')), None).get('name')),
            'center': tuple(annotation.get('center')),
            'angle': annotation.get('angle'),
            'confidence': annotation.get('confidence'),
            'min_rectangle': [tuple(x) for x in annotation.get('min_rectangle')],
            'bounding_box': annotation.get('bbox')
        }
    else:
        return None
