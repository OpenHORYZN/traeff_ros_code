import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class MapGenerator(Node):

    def __init__(self):
        super().__init__('map_generator')
        self.box_map = OccupancyGrid()
        self.box_map.info.resolution = 0.1 # map resolution [m/cell]
        self.box_map.info.width = 500 # map width [m]
        self.box_map.info.height  = 500 # map height [m]
        self.box_map.header.frame_id = 'map'
        self.box_map.data = [0]*500*500 # default unoccupied
        # origin initialized to 0,0,0 with identity quaternion

        self.box_arr = PoseArray()
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'box_map', map_qos)
        self.box_sub = self.create_subscription(
            PoseArray,
            'boxes',
            self.box_callback,
            10)
        self.timer = self.create_timer(0.1, self.publish_once)

    def publish_once(self):
        self.map_pub.publish(self.box_map)
        self.timer.cancel() # cancel timer so it doesn't run again
        
    def box_callback(self, msg):
        new_boxes = []
        for p in msg.poses:
            if p not in self.box_arr.poses:
                self.box_arr.poses.append(p)
                new_boxes.append(p)
        if len(new_boxes):
            self.update_map(new_boxes)
            self.map_pub.publish(self.box_map)

    def update_map(self, new_boxes):
        for b in new_boxes:
            indices = self.calculate_indices(b.position.x, b.position.y)
            for i in indices:
                self.box_map.data[i] = 100 # occupied with probability 1
    
    def calculate_indices(self, bx, by):
        # We are not detecting boxes per size, assume large ones to get enough clearance
        # larger box size is 0.6m x 0.4m (divide by resolution to get size in cells).

        # Since we don't know the box orientation, just remove a square with side
        # length equal to twice the max distance to the center of the box
        # (square covering max circle) 2*sqrt(0,3*0.3+0.2*0.2) = 0.721
        indices = []
        res = self.box_map.info.resolution
        w = self.box_map.info.width
        h = self.box_map.info.height
        half_side = math.ceil(0.3605/res) # side length of obstacle square
        x_map, y_map = bx//res, by//res
        # make sure indices do not exceed grid bounds
        for i in range(-half_side, half_side):
            for j in range(-half_side, half_side):
                x = x_map + i
                y = y_map + j
                if x < w and y < h and x >= 0 and y >= 0:
                    indices.append(int(y*w + x)) # see OccupancyGrid documentation
        return indices

def main(args=None):
    rclpy.init(args=args)
    map_generator = MapGenerator()
    rclpy.spin(map_generator)
    map_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()