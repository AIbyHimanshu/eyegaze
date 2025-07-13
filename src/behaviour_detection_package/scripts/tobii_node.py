#!/usr/bin/env python3
import rospy
from tobii_research import find_all_eyetrackers, EyeTracker
from std_msgs.msg import Float32
import numpy as np

class TobiiTracker:
    def __init__(self):
        rospy.init_node('tobii_tracker')
        self.pupil_pub = rospy.Publisher('/pupil_diameter', Float32, queue_size=10)
        self.gaze_pub = rospy.Publisher('/gaze_position', Float32, queue_size=10)
        
        trackers = find_all_eyetrackers()
        if not trackers:
            rospy.logerr("No Tobii eye trackers found!")
            return
            
        self.tracker = trackers[0]
        rospy.loginfo(f"Connected to {self.tracker.model}")
        
        self.tracker.subscribe_to(
            tobii_research.EYE_TRACKER_GAZE_DATA, 
            self.gaze_data_callback, 
            as_dictionary=True
        )
        
        rospy.on_shutdown(self.shutdown)
        
    def gaze_data_callback(self, gaze_data):
        # Extract pupil diameter (average of both eyes)
        left_pupil = gaze_data['left_pupil_diameter']
        right_pupil = gaze_data['right_pupil_diameter']
        
        if left_pupil > 0 and right_pupil > 0:
            avg_diameter = (left_pupil + right_pupil) / 2.0
            self.pupil_pub.publish(Float32(avg_diameter))
        
        # Extract gaze position
        left_gaze = gaze_data['left_gaze_point_on_display_area']
        right_gaze = gaze_data['right_gaze_point_on_display_area']
        
        if all(0 <= coord <= 1 for coord in left_gaze) and all(0 <= coord <= 1 for coord in right_gaze):
            avg_x = (left_gaze[0] + right_gaze[0]) / 2
            avg_y = (left_gaze[1] + right_gaze[1]) / 2
            self.gaze_pub.publish(Float32(avg_x), Float32(avg_y))
            
    def shutdown(self):
        if hasattr(self, 'tracker'):
            self.tracker.unsubscribe_from(tobii_research.EYE_TRACKER_GAZE_DATA)
            
if __name__ == '__main__':
    node = TobiiTracker()
    rospy.spin()
