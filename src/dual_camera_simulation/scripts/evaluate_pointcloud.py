#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import statistics
import argparse
from colorama import Fore, Style, init

class PointCloudEvaluator:
    def __init__(self, topic):
        self.topic = topic
        self.point_count = 0
        self.density_values = []
        self.points_received = 0
        self.frames_received = 0
        self.start_time = None
        
        # Initialize colorama
        init()
        
        # Subscribe to point cloud topic
        rospy.Subscriber(topic, PointCloud2, self.cloud_callback)
        
    def cloud_callback(self, cloud_msg):
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            
        # Count points
        points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
        self.point_count = len(points)
        self.points_received += self.point_count
        self.frames_received += 1
        
        # Calculate point density (points per cubic meter)
        if self.point_count > 0:
            points_array = np.array(points)
            if len(points_array) > 0:
                x_min, y_min, z_min = np.min(points_array, axis=0)
                x_max, y_max, z_max = np.max(points_array, axis=0)
                
                volume = (x_max - x_min) * (y_max - y_min) * (z_max - z_min)
                if volume > 0:
                    density = self.point_count / volume
                    self.density_values.append(density)
        
        self.print_statistics()
        
    def print_statistics(self):
        # Calculate elapsed time
        if self.start_time is not None:
            elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        else:
            elapsed_time = 0
            
        # Clear terminal
        print("\033c", end="")
        
        # Print header
        print(f"{Fore.CYAN}========== Point Cloud Evaluation for {self.topic} =========={Style.RESET_ALL}")
        print(f"Elapsed time: {elapsed_time:.2f} seconds")
        print(f"Frames received: {self.frames_received}")
        
        # Print current frame stats
        print(f"\n{Fore.GREEN}Current Frame:{Style.RESET_ALL}")
        print(f"  Number of points: {self.point_count}")
        
        # Print overall stats
        print(f"\n{Fore.GREEN}Overall Statistics:{Style.RESET_ALL}")
        print(f"  Total points received: {self.points_received}")
        print(f"  Average points per frame: {self.points_received / max(1, self.frames_received):.2f}")
        
        # Print density stats
        if len(self.density_values) > 0:
            avg_density = sum(self.density_values) / len(self.density_values)
            print(f"\n{Fore.GREEN}Point Cloud Density:{Style.RESET_ALL}")
            print(f"  Current density: {self.density_values[-1]:.2f} points/m³")
            print(f"  Average density: {avg_density:.2f} points/m³")
            
            if len(self.density_values) > 1:
                stdev = statistics.stdev(self.density_values)
                print(f"  Density standard deviation: {stdev:.2f}")
                print(f"  Density stability: {100 * (1 - min(1, stdev/max(1, avg_density))):.2f}%")
        
        # Print fusion quality indicator (for merged point cloud)
        if "merged" in self.topic:
            if len(self.density_values) > 5:
                stability = 100 * (1 - min(1, statistics.stdev(self.density_values[-5:]) / 
                                        max(1, sum(self.density_values[-5:]) / 5)))
                
                quality_score = min(100, stability * self.point_count / 10000)
                
                print(f"\n{Fore.YELLOW}Fusion Quality Assessment:{Style.RESET_ALL}")
                print(f"  Recent stability: {stability:.2f}%")
                print(f"  Fusion quality score: {quality_score:.2f}/100")
                
                if quality_score > 80:
                    print(f"  {Fore.GREEN}Status: Excellent fusion{Style.RESET_ALL}")
                elif quality_score > 60:
                    print(f"  {Fore.CYAN}Status: Good fusion{Style.RESET_ALL}")
                elif quality_score > 40:
                    print(f"  {Fore.YELLOW}Status: Fair fusion{Style.RESET_ALL}")
                elif quality_score > 20:
                    print(f"  {Fore.RED}Status: Poor fusion{Style.RESET_ALL}")
                else:
                    print(f"  {Fore.RED}Status: Very poor fusion{Style.RESET_ALL}")

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Evaluate point cloud quality')
    parser.add_argument('--topic', type=str, default='/merged_point_cloud', 
                      help='Point cloud topic to evaluate')
    args = parser.parse_args()
    
    # Initialize ROS node
    rospy.init_node('point_cloud_evaluator')
    
    # Create evaluator
    evaluator = PointCloudEvaluator(args.topic)
    
    rospy.loginfo(f"Evaluating point cloud topic: {args.topic}")
    rospy.loginfo("Press Ctrl+C to stop")
    
    # Spin until shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass