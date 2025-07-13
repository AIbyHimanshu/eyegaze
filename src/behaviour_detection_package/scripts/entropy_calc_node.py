#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import math
import numpy as np
import csv
import os
import antropy as ant

class entropy_calc_topic:

    def __init__(self):
        self.log_file_path = os.path.expanduser("~/entropy_log.csv")
        if not os.path.exists(self.log_file_path):
            with open(self.log_file_path, mode='w') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'entropy_ang', 'entropy_lin', 'total_entropy'])

        rospy.loginfo("INITIALISING NODE -> /entropy_node")

        # Initialize with empty lists
        self.error_list_ang = []
        self.error_list_lin = []
        
        # ADDED: Proper indentation and initialization
        self.max_buffer_size = 500  # Maximum buffer size
        
        self.a_ang = 0.14
        self.a_lin = 0.08

        self.bins_ang = [-5*self.a_ang, -2.5*self.a_ang, -self.a_ang, -0.5*self.a_ang,
                         0.5*self.a_ang, self.a_ang, 2.5*self.a_ang, 5*self.a_ang]
        self.bins_lin = [-5*self.a_lin, -2.5*self.a_lin, -self.a_lin, -0.5*self.a_lin,
                         0.5*self.a_lin, self.a_lin, 2.5*self.a_lin, 5*self.a_lin]

        self.entropy_started = False
        self.start_sub = rospy.Subscriber("/start_entropy", Float32, self.trigger_callback)

        self.pup_entropy = rospy.Publisher("entropy", Float32, queue_size=1)
        self.pup_entropy_ang = rospy.Publisher("entropy_ang", Float32, queue_size=1)
        self.pup_entropy_lin = rospy.Publisher("entropy_lin", Float32, queue_size=1)

        self.sub_ang_error = rospy.Subscriber("estimation_error_ang", Float32, self.estimation_error_ang_callback)
        self.sub_lin_error = rospy.Subscriber("estimation_error_lin", Float32, self.estimation_error_lin_callback)

        self.entropy = 0.0
        self.entropy_average = 0.0
        self.entropy_sum = 0.0
        self.entropy_average_counter = 0.0

    def trigger_callback(self, msg):
        self.entropy_started = (msg.data == 1.0)
        if self.entropy_started:
            rospy.loginfo("Entropy logging started.")
        else:
            rospy.loginfo("Entropy logging stopped.")

    def estimation_error_ang_callback(self, data):
        # Maintain buffer size
        if len(self.error_list_ang) >= self.max_buffer_size:
            self.error_list_ang.pop(0)
        self.error_list_ang.append(data.data)

    def estimation_error_lin_callback(self, data):
        # Maintain buffer size
        if len(self.error_list_lin) >= self.max_buffer_size:
            self.error_list_lin.pop(0)
        self.error_list_lin.append(data.data)

    def calculate_entropy(self, event=None):
        if not self.entropy_started:
            return

        # Use Sample Entropy if we have enough data points
        if len(self.error_list_ang) > 100:
            entropy_ang = ant.sample_entropy(np.array(self.error_list_ang), order=2)
        else:
            entropy_ang = 0.0
            
        if len(self.error_list_lin) > 100:
            entropy_lin = ant.sample_entropy(np.array(self.error_list_lin), order=2)
        else:
            entropy_lin = 0.0

        self.entropy = 0.7 * entropy_ang + 0.3 * entropy_lin

        self.pup_entropy.publish(self.entropy)
        self.pup_entropy_ang.publish(entropy_ang)
        self.pup_entropy_lin.publish(entropy_lin)

        self.entropy_sum += self.entropy
        self.entropy_average_counter += 1
        self.entropy_average = self.entropy_sum / self.entropy_average_counter

        rospy.loginfo(f"Entropy angular: {entropy_ang:.5f}, Entropy linear: {entropy_lin:.5f}, Total Entropy: {self.entropy:.5f}")

        # CSV Logging
        with open(self.log_file_path, mode='a') as f:
            writer = csv.writer(f)
            writer.writerow([
                rospy.get_time(),
                round(entropy_ang, 5),
                round(entropy_lin, 5),
                round(self.entropy, 5)
            ])

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = entropy_calc_topic()
        rospy.Timer(rospy.Duration(2.5), node.calculate_entropy)
        node.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        if node.entropy_average_counter > 0:
            rospy.loginfo(f"Total average entropy: {node.entropy_average:.5f}")
