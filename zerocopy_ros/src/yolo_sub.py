#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.event_handler import SubscriptionEventCallbacks
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
import threading
import queue
import time
import os
import csv

class YoloApp(Node):
    def __init__(self):
        super().__init__('yolo_app')
        
        if torch.cuda.is_available():
            self.device = 'cuda:0'
        else:
            self.device = 'cpu'
        
        self.model = YOLO("yolov8n.pt")
        self.br = CvBridge()
        
        self.raw_queue = queue.Queue(maxsize=1)
        self.gui_queue = queue.Queue(maxsize=1)
        self.is_connected = False 
        
        # ---  CSV SETUP ---
        home_dir = os.environ.get('HOME', '/home/gokul')
        csv_path = os.path.join(home_dir, 'drone_project', 'network_metrics.csv')
        self.csv_file = open(csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['System_Time', 'Publisher_Frame_ID', 'Transport_Latency_ms', 'Network_Drops'])
        self.last_frame_id = -1

        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=1)
        )

        event_callbacks = SubscriptionEventCallbacks(
            liveliness=self.on_liveliness_changed
        )

        self.subscription = self.create_subscription(
            Image, 
            'camera/image_raw', 
            self.listener_callback, 
            qos_policy,
            event_callbacks=event_callbacks
        )

        self.ai_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.ai_thread.start()

        self.get_logger().info("Node Ready. Waiting for Publisher...")

    def on_liveliness_changed(self, event):
        if event.alive_count > 0:
            self.get_logger().info("QoS Event: Publisher FOUND!")
            self.is_connected = True
        else:
            self.get_logger().warn("QoS Event: Publisher LOST!")
            self.is_connected = False


    def listener_callback(self, data):
        arrival_time = self.get_clock().now()
        msg_time = Time.from_msg(data.header.stamp)
        transport_latency = (arrival_time - msg_time).nanoseconds / 1e6
        
        # ---  FRAME DROP MATH ---
        try:
            current_frame_id = int(data.header.frame_id)
            drops = 0
            if self.last_frame_id != -1:
                drops = current_frame_id - self.last_frame_id - 1
                if drops < 0: drops = 0 
            
            self.last_frame_id = current_frame_id
            self.csv_writer.writerow([time.time(), current_frame_id, round(transport_latency, 2), drops])
            self.csv_file.flush()
        except ValueError:
            pass
        
        if self.raw_queue.full():
            try: self.raw_queue.get_nowait()
            except: pass
        self.raw_queue.put((data, transport_latency))

    def inference_worker(self):
        while True:
            try:
                data, transport_lat = self.raw_queue.get(timeout=0.1)
                
                try:
                    cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
                except:
                    continue

                t0 = time.time()
                results = self.model(cv_image, device=self.device, verbose=False)
                annotated_frame = results[0].plot() 
                t1 = time.time()
                infer_lat = (t1 - t0) * 1000.0

                pkg = {
                    "image": annotated_frame,
                    "t_lat": transport_lat,
                    "i_lat": infer_lat
                }
                
                if self.gui_queue.full():
                    try: self.gui_queue.get_nowait()
                    except: pass
                self.gui_queue.put(pkg)

            except queue.Empty:
                continue
            
    # --- CLEANUP ---
    def cleanup(self):
        self.csv_file.close()

def get_color(latency):
    if latency < 15.0:
        return (0, 255, 0)
    elif latency < 25.0:
        return (0, 255, 255)
    else:
        return (0, 0, 255)

def main(args=None):
    rclpy.init(args=args)
    node = YoloApp()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    last_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    try:
        while rclpy.ok():
            if not node.is_connected:
                display_img = last_frame.copy()
                display_img = cv2.addWeighted(display_img, 0.3, np.zeros(display_img.shape, display_img.dtype), 0, 0)
                
                text = "PUBLISHER DISCONNECTED"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)[0]
                text_x = (display_img.shape[1] - text_size[0]) // 2
                text_y = (display_img.shape[0] + text_size[1]) // 2
                
                cv2.putText(display_img, text, (text_x, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                
                cv2.imshow("Advanced Benchmark", display_img)
                cv2.waitKey(10)
                continue

            try:
                data = node.gui_queue.get(timeout=0.050)
                img = data["image"]
                t_lat = data["t_lat"]
                i_lat = data["i_lat"]
                
                last_frame = img.copy()

                cv2.rectangle(img, (10, 10), (320, 80), (0,0,0), -1)
                label_color = (255, 0, 0)
                
                cv2.putText(img, "Transport:", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_color, 2)
                cv2.putText(img, f"{t_lat:.2f} ms", (130, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, get_color(t_lat), 2)

                cv2.putText(img, "Inference:", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_color, 2)
                cv2.putText(img, f"{i_lat:.1f} ms", (130, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, get_color(i_lat), 2)

                cv2.imshow("Advanced Benchmark", img)
            
            except queue.Empty:
                pass
                
            key = cv2.waitKey(10)
            if key == 27: break
                
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
