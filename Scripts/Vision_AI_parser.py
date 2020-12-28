#!/usr/bin/env python3
# license removed for brevity
from google.cloud import vision
import io
import os
import re
import json
import glob
from decimal import Decimal

class Vision_AI:

    @staticmethod
    def Make_request(path):
        # Imports the Google Cloud client library


        # Instantiates a client
        client = vision.ImageAnnotatorClient()

        # The name of the image file to annotate
        file_name = os.path.abspath(path)

        # Loads the image into memory
        with io.open(file_name, 'rb') as image_file:
            content = image_file.read()

        image = vision.Image(content=content)

        # Performs label detection on the image file
        response = client.label_detection(image=image)
        labels = response.label_annotations

        return labels


folder = "/home/pablo/catkin_ws/images/"
files_path = os.path.join(folder, '*.jpg')
files = sorted( glob.iglob(files_path), key=os.path.getctime, reverse=True) 
this_file = files[0] 

cloud = Vision_AI()
result = cloud.Make_request(this_file)

with open('/home/pablo/catkin_ws/src/topomapping/tmp/Vision_AI_output.txt', 'w') as outfile:
    outfile.write(str(result))


 