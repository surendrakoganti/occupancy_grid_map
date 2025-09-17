#!/usr/bin/env python

# Sensor Data Fusion: Occupancy gridmap
# 
# Gridmap Node

import rclpy
from rclpy.node import Node
import math

import numpy as np

from nav_msgs.msg import OccupancyGrid


class Gridmap(Node):
       
    def __init__(self, size, resolution, topic_name, p_0, p_occ, p_free):
        super().__init__('my_g_map')
        print('Gridmap Node Initialized, It will publish an OccupancyGrid Message as topic:', topic_name)

        # Initialize grid map parameters
        self.size = size
        self.resolution = resolution

        # Calculate default log odds
        self.l_0 = self.prob2logOdds(p_0)
        self.l_occ = self.prob2logOdds(p_occ)
        self.l_free = self.prob2logOdds(p_free)
        self.log_odds = self.l_0 * np.ones((size, size))

        # Prepare Publisher
        self.topic_name = topic_name
        self.pub = self.create_publisher(OccupancyGrid, topic_name, 1)

        # Create timer for callbacking publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_gridmap)                     

    # Transformation from probability to logOdds 
    def prob2logOdds(self, prob):
        li_k = np.log(prob / (1-prob))
        return(li_k) 
    
    # Transformation from logOdds to probabilities
    def logOdds2prob(self, log_o):
        pi_k = 1 - (1 / (1 + np.exp(log_o)))
        return(pi_k)

    def update_logOdds(self, cell_x, cell_y, l_mi):
        self.log_odds[cell_x, cell_y] += l_mi - self.l_0

    def publish_gridmap(self):

        # Filling OccupancyGrid Message for the topic
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.size
        msg.info.height = self.size

        # Transform log_odds to probabilities and flatten array
        msg_data = self.logOdds2prob(self.log_odds).flatten('F')

        # Normalize and set to [0, 100]
        if (np.max(msg_data) != np.min(msg_data)):
            msg_data -= np.min(msg_data)
            msg_data *= 100. / np.max(msg_data)
        else:
            msg_data *= 100 

        # Assign -1 to nan-values
        msg_data[np.isnan(msg_data)] = -1

        # Cast to int8
        msg.data = msg_data.astype(np.int8).tolist()

        # Publishing grid message
        self.pub.publish(msg)


# Testing the module
def test_gridmap_node_checkered(args=None):   

    rclpy.init(args=args)

    # Generate Gridmap object
    g_map = Gridmap(20, 0.2, "hsc_map", 0.5, 0.8, 0.2)

    # Prepare Checkered Shape
    g_map.log_odds[1::2, :] = g_map.l_free
    g_map.log_odds[:, 1::2] = g_map.l_free
    g_map.initialized = True

    # Set ROS Node
    rclpy.spin(g_map)

    g_map.destroy_node()
    rclpy.shutdown()

def test_gridmap_node_figure(args=None): 
    rclpy.init(args=args)

    # Generate Gridmap object
    g_map = Gridmap(20, 0.2, "hsc_map", 0.5, 0.8, 0.2)

    # Draw smiley face
    # eyes
    g_map.log_odds[5, 6] = g_map.l_occ
    g_map.log_odds[5, 13] = g_map.l_occ

    # mouth
    mouth_coords = [
        (14, 6), (14, 7), (14, 8), (14, 9), (14, 10), (14, 11), (14, 12), (14, 13),
        (13, 5), (13, 14),
        (12, 4), (12, 15)
    ]
    
    for x, y in mouth_coords:
        g_map.log_odds[x, y] = g_map.l_occ

    # Set ROS Node
    rclpy.spin(g_map)

    g_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__test_gridmap_node_checkered__':
    test_gridmap_node_checkered()

if __name__ == '__test_gridmap_node_figure__':
    test_gridmap_node_figure()

