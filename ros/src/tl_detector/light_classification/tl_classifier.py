from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    
    min_score = 0.15

    map_index_TrafficLight = {
        1 : TrafficLight.GREEN,
        2 : TrafficLight.RED,
        3 : TrafficLight.YELLOW,
        4 : TrafficLight.UNKNOWN
    }
    map_TrafficLight_Color = {
        TrafficLight.RED: "Red",
        TrafficLight.GREEN: "Green",
        TrafficLight.YELLOW: "Yellow",
        TrafficLight.UNKNOWN: "Unknown"
    }

    def __init__(self, model_graph):
        # --- The classifier is built using instructions and data from
        # https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI/blob/master/README.md
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
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        num_detections = int(detections[0])
        
        max_score_found = 0.0

        light_state = TrafficLight.UNKNOWN
        # rospy.logwarn("Found %s matches", num_detections)

        class_count = {
            TrafficLight.RED: 0,
            TrafficLight.YELLOW: 0,
            TrafficLight.GREEN: 0,
            TrafficLight.UNKNOWN: 0
        }

        for i in range(num_detections):
            # Find the highest confidence match
            score = scores[i]
            if (score > self.min_score):
                current_light_state = self.map_index_TrafficLight[classes[i]]
                class_count[current_light_state] = class_count[current_light_state] + 1
                if (score > max_score_found):
                    light_state = current_light_state
                    max_score_found = score
        
        # Highest score light found:
        rospy.logwarn("Classifying - max-score :%s, state:%s", max_score_found, self.map_TrafficLight_Color[light_state])
        
        # class_with_max_count = max(class_count, key=class_count.get)
        # count_of_class_with_max = class_count[class_with_max_count]

        # if count_of_class_with_max == 0:
        #     return TrafficLight.UNKNOWN
        # else:
        #     rospy.logwarn("Found max class %s at %s", self.map_TrafficLight_Color[class_with_max_count], count_of_class_with_max)
        #     return class_with_max_count
        
        return light_state
