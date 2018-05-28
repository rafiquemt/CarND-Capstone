from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    
    min_score = 0.2
    map_index_TrafficLight = {
        1 : TrafficLight.GREEN,
        2 : TrafficLight.RED,
        3 : TrafficLight.YELLOW,
        4 : TrafficLight.UNKNOWN
    }

    def __init__(self, model_graph):
        #Initialize tensorflow to use checked in model graph
        rospy.logwarn("Loading Model: %s", model_graph)
        graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_graph, 'rb') as graph_file:
                od_graph_def.ParseFromString(graph_file.read())
                tf.import_graph_def(od_graph_def, name='')
        self.image_tensor = graph.get_tensor_by_name('image_tensor:0')
        self.other_tensors = [ 
            graph.get_tensor_by_name('detection_scores:0'),
            graph.get_tensor_by_name('detection_classes:0'),
            graph.get_tensor_by_name('num_detections:0')
        ]
        self.session = tf.Session(graph = graph, config = config)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        (scores, classes, detections) = self.session.run(self.other_tensors, feed_dict={ self.image_tensor: np.expand_dims(image, axis=0)})
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        num_detections = detections[0]
        
        if num_detections > 0:
            # Grab the first light
            score = scores[0]
            light_state = self.map_index_TrafficLight[classes[0]]
            if score > self.min_score:
                return light_state
        
        return TrafficLight.UNKNOWN
