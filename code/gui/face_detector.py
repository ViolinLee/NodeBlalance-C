import cv2
from mtcnn import MTCNN


class CascadeDetector:
    def __init__(self, xml_model_path):
        self._model = cv2.CascadeClassifier(xml_model_path)

    def predict_cv2img(self, cv2img):
        # Convert into grayscale
        gray = cv2.cvtColor(cv2img, cv2.COLOR_BGR2GRAY)
        # Detect faces
        faces_bbox = self._model.detectMultiScale(gray, 1.1, 4)
        # Draw rectangle around the faces
        for (x, y, w, h) in faces_bbox:
            cv2.rectangle(cv2img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        return cv2img, faces_bbox


class MTCNNDetector:
    """https://github.com/ipazc/mtcnn"""

    def __init__(self):
        self.detector = MTCNN()

    def predict_cv2img(self, cv2img):
        img = cv2.cvtColor(cv2img, cv2.COLOR_BGR2RGB)
        results = self.detector.detect_faces(img)

        faces_bbox = []
        for result in results:
            # Result is an array with all the bounding boxes detected. We know that for 'ivan.jpg' there is only one.
            bounding_box = result['box']
            keypoints = result['keypoints']
            faces_bbox.append(bounding_box)

            cv2.rectangle(img,
                          (bounding_box[0], bounding_box[1]),
                          (bounding_box[0] + bounding_box[2], bounding_box[1] + bounding_box[3]),
                          (0, 155, 255), 2)
        return img, faces_bbox


if __name__ == '__main__':
    xml_cascade = 'haarcascade_frontalface_default.xml'
    cascade_detector = CascadeDetector(xml_cascade)
    mtcnn_detector = MTCNNDetector()

    image = cv2.imread("ivan.jpg")  # cv2 BGR image
    res_img = mtcnn_detector.predict_cv2img(image)
    cv2.imshow('face detection', res_img)
    cv2.waitKey(0)
