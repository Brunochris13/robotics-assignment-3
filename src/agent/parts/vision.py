
import os
import cv2
import numpy as np
import random
import re
import rospy
from numpy.core.fromnumeric import shape

WEIGHTS_DIR = "resources/weights/"
IMAGE_DIR = "resources/img/"

class Vision():
    def __init__(self, basename=""):
        self.first_order = True
        self.name = basename + "[VISION] "
        
        # print(os.path.isdir(WEIGHTS_DIR))
        # print(os.getcwd())
        # https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt
        self.FACE_PROTO = WEIGHTS_DIR + "deploy.prototxt.txt"
        # https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20180205_fp16/res10_300x300_ssd_iter_140000_fp16.caffemodel
        self.FACE_MODEL = WEIGHTS_DIR + "res10_300x300_ssd_iter_140000_fp16.caffemodel"
        # The gender model architecture
        # https://drive.google.com/open?id=1W_moLzMlGiELyPxWiYQJ9KFaXroQ_NFQ
        self.GENDER_MODEL = WEIGHTS_DIR + 'deploy_gender.prototxt'
        # The gender model pre-trained weights
        # https://drive.google.com/open?id=1AW3WduLk1haTVAxHOkVS_BEzel1WXQHP
        self.GENDER_PROTO = WEIGHTS_DIR + 'gender_net.caffemodel'
        # Each Caffe Model impose the shape of the input image also image preprocessing is required like mean
        # substraction to eliminate the effect of illunination changes
        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        # Represent the gender classes
        self.GENDER_LIST = ['Male', 'Female']
        # The model architecture
        # download from: https://drive.google.com/open?id=1kiusFljZc9QfcIYdU2s7xrtWHTraHwmW
        self.AGE_MODEL = WEIGHTS_DIR + 'deploy_age.prototxt'
        # The model pre-trained weights
        # download from: https://drive.google.com/open?id=1kWv0AjxGSN0g31OeJa02eBGM0R_jcjIl
        self.AGE_PROTO = WEIGHTS_DIR + 'age_net.caffemodel'
        # Represent the 8 age classes of this CNN probability layer
        self.AGE_INTERVALS = ['(0, 2)', '(4, 6)', '(8, 12)', '(13, 19)',
                         '(25, 33)', '(38, 44)', '(48, 54)', '(60, 100)']

        # Initialize frame size
        self.frame_width = 1280
        self.frame_height = 720
        # load face Caffe model
        self.face_net = cv2.dnn.readNetFromCaffe(self.FACE_PROTO, self.FACE_MODEL)
        # Load age prediction model
        self.age_net = cv2.dnn.readNetFromCaffe(self.AGE_MODEL, self.AGE_PROTO)
        # Load gender prediction model
        self.gender_net = cv2.dnn.readNetFromCaffe(self.GENDER_MODEL, self.GENDER_PROTO)


    def get_faces(self, frame, confidence_threshold=0.5):
        """Get faces coordinates"""
        # convert the frame into a blob to be ready for NN input
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104, 177.0, 123.0))
        # set the image as input to the NN
        self.face_net.setInput(blob)
        # perform inference and get predictions
        output = np.squeeze(self.face_net.forward())
        # initialize the result list
        faces = []
        # Loop over the faces detected
        for i in range(output.shape[0]):
            confidence = output[i, 2]
            if confidence > confidence_threshold:
                box = output[i, 3:7] * \
                    np.array([frame.shape[1], frame.shape[0],
                             frame.shape[1], frame.shape[0]])
                # convert to integers
                start_x, start_y, end_x, end_y = box.astype(np.int)
                # widen the box a little
                start_x, start_y, end_x, end_y = start_x - \
                    10, start_y - 10, end_x + 10, end_y + 10
                start_x = 0 if start_x < 0 else start_x
                start_y = 0 if start_y < 0 else start_y
                end_x = 0 if end_x < 0 else end_x
                end_y = 0 if end_y < 0 else end_y
                # append to our list
                faces.append((start_x, start_y, end_x, end_y))
        return faces


    def display_img(self, title, img):
        """Displays an image on screen and maintains the output until the user presses a key"""
        # Display Image on screen
        cv2.imshow(title, img)
        # Mantain output until user presses a key
        cv2.waitKey(3000)
        # Destroy windows when user presses a key
        cv2.destroyAllWindows()


    def image_resize(self, image, width = None, height = None, inter = cv2.INTER_AREA):
        "Resize the image"
        # initialize the dimensions of the image to be resized and
        # grab the image size
        dim = None
        (h, w) = image.shape[:2]
        # if both the width and height are None, then return the
        # original image
        if width is None and height is None:
            return image
        # check to see if the width is None
        if width is None:
            # calculate the ratio of the height and construct the
            # dimensions
            r = height / float(h)
            dim = (int(w * r), height)
        # otherwise, the height is None
        else:
            # calculate the ratio of the width and construct the
            # dimensions
            r = width / float(w)
            dim = (width, int(h * r))
        # resize the image
        return cv2.resize(image, dim, interpolation = inter)


    def get_gender_predictions(self, face_img):
        """Get a gender prediction"""
        blob = cv2.dnn.blobFromImage(
            image=face_img, scalefactor=1.0, size=(227, 227),
            mean=self.MODEL_MEAN_VALUES, swapRB=False, crop=False
        )
        self.gender_net.setInput(blob)
        return self.gender_net.forward()


    def get_age_predictions(self, face_img):
        """Get a age prediction"""
        blob = cv2.dnn.blobFromImage(
            image=face_img, scalefactor=1.0, size=(227, 227),
            mean=self.MODEL_MEAN_VALUES, swapRB=False
        )
        self.age_net.setInput(blob)
        return self.age_net.forward()


    def predict_age_and_gender(self, image):
        """Predict the gender of the faces showing in the image"""
        # Initialize frame size
        # frame_width = 1280
        # frame_height = 720
        # Read Input Image
        img = image
        # resize the image, uncomment if you want to resize the image
        # img = cv2.resize(img, (frame_width, frame_height))
        # Take a copy of the initial image and resize it
        frame = img.copy()
        if frame.shape[1] > self.frame_width:
            frame = self.image_resize(frame, width=self.frame_width)
        # predict the faces
        faces = self.get_faces(frame)
        # Loop over the faces detected
        # for idx, face in enumerate(faces):
        ppl=[]
        for i, (start_x, start_y, end_x, end_y) in enumerate(faces):
            face_img = frame[start_y: end_y, start_x: end_x]
            age_preds = self.get_age_predictions(face_img)
            gender_preds = self.get_gender_predictions(face_img)
            i = gender_preds[0].argmax()
            gender = self.GENDER_LIST[i]
            gender_confidence_score = gender_preds[0][i]
            i = age_preds[0].argmax()
            age = self.AGE_INTERVALS[i]
            age_confidence_score = age_preds[0][i]
            label = f"{gender}-{gender_confidence_score*100:.1f}%, {age}-{age_confidence_score*100:.1f}%"
            ppl.append((gender,age))
            rospy.loginfo(self.name + label)

        return ppl


    def formater(self, ppl):
        """Change the format of data"""
        people = []
        for person in ppl:
            gender, age = person
            gender_bool = gender == "Female"
            age = re.findall(r'\d+', age)
            age = (int(age[0]) + int(age[1])) / 2
            people.append((gender_bool, age))

        return people


    def see(self):
        """Pick a random image get the list of genders and ages representing each person.

        Returns:
            (list(tuple(bool, int))): 
        """
        
        file_list = os.listdir(IMAGE_DIR)
        # img_name = random.choice(file_list)
        if self.first_order:
            img_name = "male-family-members-the-male-members-of-the-family-representing-the-three-generations-stock-image_csp19032269.webp"
            self.first_order = False
        else:
            img_name = "man2.jpg"
        img = cv2.imread(IMAGE_DIR + img_name)
        self.display_img("Customer",img)
        ppl = self.predict_age_and_gender(img)
        ppl = self.formater(ppl)
        ages = [person[1] for person in ppl]
        rospy.loginfo(self.name + f"{ages}")

        return ppl

    def see_demo(self):
        file_list = os.listdir(IMAGE_DIR)
        img_name = "3ppl_with_3generation.webp"
        img = cv2.imread(IMAGE_DIR + img_name)
        self.display_img("Customer",img)
        ppl = self.predict_age_and_gender(img)
        ppl = self.formater(ppl)
        ages = [person[1] for person in ppl]
        rospy.loginfo(self.name + f"{ages}")

        return ppl

if __name__ == '__main__':
    
    v = Vision()
    v.see_demo()