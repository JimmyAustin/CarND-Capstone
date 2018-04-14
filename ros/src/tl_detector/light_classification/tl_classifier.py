import os
print('Hiding cuda devices') 
#os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
#os.environ["CUDA_VISIBLE_DEVICES"] = ""

from styx_msgs.msg import TrafficLight
import scipy
from .models import get_detector_model, get_classification_model
from collections import Counter
import numpy as np
from scipy.ndimage.measurements import label
import tensorflow as tf
from toolbox.robotics.blackbox import Blackbox
from toolbox.robotics import image_bounding_box_manager
from toolbox.profiler import time_function, Timer
from datetime import datetime
import rospy
import os

labels = {
    TrafficLight.GREEN: 0,
    TrafficLight.YELLOW: 1,
    TrafficLight.RED: 2,
    TrafficLight.UNKNOWN: 3
}

printable_label = {
    TrafficLight.GREEN: "GREEN",
    TrafficLight.YELLOW: "YELLOW",
    TrafficLight.RED: "RED",
    TrafficLight.UNKNOWN: "UNKNOWN"    
}

reverse_labels = {y: x for x,y in labels.items()}

detector_image_size = (640, 480)


class TLClassifier(object):
    def __init__(self):
        directory = os.path.dirname(__file__) + "/models"
        detector_model_location = directory + "/fast_traffic_detection_model_v2.hdf5"
        classification_model_location = directory + "/traffic_light_classification_v1.hdf5"
        self.blackbox = Blackbox(feeds=['raw', 'traffic_light_detection', 'traffic_light_classification'])

        
        self.g1 = tf.Graph()
        self.g2 = tf.Graph()
        session_config = tf.ConfigProto(intra_op_parallelism_threads=2,
                                        inter_op_parallelism_threads=2)
        with self.g1.as_default():
            self.session1 = tf.Session(config=session_config)
            with self.session1.as_default():
                self.detector_model = get_detector_model(detector_image_size[0], detector_image_size[1], 
                                                         weights_file=detector_model_location)
        with self.g2.as_default():
            self.session2 = tf.Session(config=session_config)
            with self.session2.as_default():
                self.classification_model = get_classification_model(weights_file=classification_model_location)
        print('loaded models')
        print("traffic")

        #TODO load classifier
        pass

    def get_classification(self, image):
        #return TrafficLight.UNKNOWN
        starttime = datetime.now()
        self.blackbox.add_image('raw', image)
        traffic_lights = self.find_traffic_lights(image)

        result_detection = {}
        overlayed_image = resize_for_detection_model(image)
        print("traffic")
        rospy.logerr('Traffic light count')
        rospy.logerr(len(traffic_lights))

        #traffic_lights = sorted(traffic_lights, key=lambda x: 0-x[0].area())[0:3]

        predictions = self.predict_traffic_lights([img for _, img in traffic_lights])

        for traffic_light, prediction in zip(traffic_lights, predictions):
            bbox, _ = traffic_light
            bbox.draw_on_image(overlayed_image, title=str(printable_label[prediction][0]))
            result_detection[prediction] = result_detection.get(prediction, 0) + 1
        self.blackbox.add_image('traffic_light_classification', overlayed_image)

        printable_results = {printable_label[k]: v for k, v in result_detection.items()}
        for k, v in printable_results.items():
            print(k + ' - ' + str(v))

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        self.blackbox.debug_information['classifier_results'] = printable_results
        self.blackbox.save_image()
        endtime = datetime.now()
        print("Total Time Taken: {0}".format((endtime-starttime).total_seconds()))

        if printable_results.get('RED', 0) > 1:
            print("returned red")
            return TrafficLight.RED
        if printable_results.get('YELLOW', 0) > 1:
            print("returned yellow")
            return TrafficLight.YELLOW
        if printable_results.get('GREEN', 0) > 1:
            print("returned green")
            return TrafficLight.GREEN
        return TrafficLight.UNKNOWN
    
    def find_traffic_lights(self, image):

        image = resize_for_detection_model(image)
        with self.g1.as_default():
            with self.session1.as_default():
                prediction = self.detector_model.predict(np.array([image]))[0]

        prediction = image_bounding_box_manager.apply_threshold(prediction, 0.05)


        sections = image_bounding_box_manager.find_contiguous_sections(prediction)

        found_traffic_lights = []
        
        prediction = prediction * 255

        for bounding_box in sections:
            sub_image = image_bounding_box_manager.get_image_in_bounding_box(image, bounding_box)
            found_traffic_lights.append((bounding_box, sub_image))
            bounding_box.draw_on_image(prediction, title='')

        self.blackbox.add_image('traffic_light_detection', prediction*255)

        return found_traffic_lights

    def predict_traffic_lights(self, images):
        if len(images) == 0:
            return []
        images = [scipy.misc.imresize(image, (64,32)) for image in images]
        with self.g2.as_default():
            with self.session2.as_default():
                rospy.logerr('Image shapes')
                rospy.logerr([image.shape for image in images])
                predictions = self.classification_model.predict(np.array(images))
                #prediction = self.classification_model.predict(np.array([image]))[0]    
        correct_labels = [reverse_labels[np.argmax(prediction)] for prediction in predictions]
        return correct_labels

def resize_for_detection_model(image):
    return scipy.misc.imresize(image, detector_image_size)
