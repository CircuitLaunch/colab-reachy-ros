# Adapted from
# https://www.pyimagesearch.com/2020/05/04/covid-19-face-mask-detector-with-opencv-keras-tensorflow-and-deep-learning/

# Copyright (c) 2020 PyImageSearch.com
#
# SIMPLE VERSION
# Feel free to use this code for your own projects, whether they are
# purely educational, for fun, or for profit. THE EXCEPTION BEING if
# you are developing a course, book, or other educational product.
# Under *NO CIRCUMSTANCE* may you use this code for your own paid
# educational or self-promotional ventures without written consent
# from Adrian Rosebrock and PyImageSearch.com.
#
# LONGER, FORMAL VERSION
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# Notwithstanding the foregoing, you may not use, copy, modify, merge,
# publish, distribute, sublicense, create a derivative work, and/or
# sell copies of the Software in any work that is designed, intended,
# or marketed for pedagogical or instructional purposes related to
# programming, coding, application development, or information
# technology. Permission for such use, copying, modification, and
# merger, publication, distribution, sub-licensing, creation of
# derivative works, or sale is expressly withheld.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
import numpy as np
import cv2
from pathlib import Path


class MaskDetector:
    def __init__(self, face_model_dir: Path, mask_model: Path):
        prototxt_path = face_model_dir / "deploy.prototxt"
        weights_path = face_model_dir / "res10_300x300_ssd_iter_140000.caffemodel"
        self._face_net = cv2.dnn.readNet(str(prototxt_path.resolve()), str(weights_path.resolve()))

        self._mask_net = load_model(str(mask_model.resolve()))

    def detect_and_predict_mask(self, frame: np.ndarray, face_confidence: float = 0.5):
        """
        frame: cv2 image frame
        faceNet: face detector model
        maskNet: mask detector model
        face_confidence: If a face detection confidence is below this probability, the potential face will be ignored

        Returns a list of pairs of form (probability of mask, probability of no mask)
        """

        print(f"Frame dimensions: {frame.shape}")

        # grab the dimensions of the frame and then construct a blob
        # from it
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0))
        # pass the blob through the network and obtain the face detections
        self._face_net.setInput(blob)
        detections = self._face_net.forward()
        # initialize our list of faces, their corresponding locations,
        # and the list of predictions from our face mask network
        faces = []
        locs = []
        preds = []

        # loop over the detections
        for i in range(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the detection
            confidence = detections[0, 0, i, 2]
            # filter out weak detections by ensuring the confidence is
            # greater than the minimum confidence
            if confidence > face_confidence:
                # compute the (x, y)-coordinates of the bounding box for
                # the object
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # ensure the bounding boxes fall within the dimensions of
                # the frame
                (startX, startY) = (max(0, startX), max(0, startY))
                (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

                print(f"Face bounding box: startx: {startX}, starty: {startY}, endx: {endX}, endy: {endY}")

                # extract the face ROI, convert it from BGR to RGB channel
                # ordering, resize it to 224x224, and preprocess it
                face = frame[startY:endY, startX:endX]
                face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
                face = cv2.resize(face, (224, 224))
                print(f"Face type after resizing: {type(face)}")
                face = img_to_array(face)
                face = preprocess_input(face)
                # add the face and bounding boxes to their respective
                # lists
                print(f"Face dimensions: {face.shape}")
                faces.append(face)
                locs.append((startX, startY, endX, endY))

        # only make a predictions if at least one face was detected
        print(f"Faces: {faces}")
        if len(faces) > 0:
            preds = [self._mask_net.predict([face])[0] for face in faces]
        # return a 2-tuple of the face locations and their corresponding
        # locations
        return (locs, preds)


def generate_debug_frame(frame, locs, preds):
    # loop over the detected face locations and their corresponding
    # locations
    for (box, pred) in zip(locs, preds):
        # unpack the bounding box and predictions
        (startX, startY, endX, endY) = box
        (mask, withoutMask) = pred
        # determine the class label and color we'll use to draw
        # the bounding box and text
        label = "Mask" if mask > withoutMask else "No Mask"
        color = (0, 255, 0) if label == "Mask" else (0, 0, 255)
        # include the probability in the label
        label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)
        # display the label and bounding box rectangle on the output
        # frame
        cv2.putText(frame, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)

    return frame
