from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self, model_graph):
        #Initialize tensorflow to use checked in model graph
        graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_graph, 'rb') as graph_file:
                od_graph_def.ParseFromString(graph_file.read)
                tf.import_graph_def(od_graph_def, name='')
        self.image_tensor = graph.get_tensor_by_name('image_tensor:0')
        self.other_tensors = [ 
            graph.get_tensor_by_name('detection_boxes:0'), 
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
        (boxes, scores, classes, detections) = self.session.run(self.other_tensors, feed_dict={ self.image_tensor: np.expand_dims(image, axis=0)})
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        num = num[0]
        
        if num > 0:
            # Grab the first light
            
        else:
            return TrafficLight.UNKNOWN
