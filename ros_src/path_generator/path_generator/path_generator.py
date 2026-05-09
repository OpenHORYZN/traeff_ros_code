import math
import rclpy
import cv2
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose

from ompl import base as ob
from ompl import geometric as og

class MapValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, cv_map):
        ob.StateValidityChecker.__init__(self, si)
        self.cv_map = cv_map
        self.free_threshold = 200 

    def isValid(self, state):
        try:
            x = int(state[0])
            y = int(state[1])
            if x < 0 or y < 0 or x >= self.cv_map.shape[1] or y >= self.cv_map.shape[0]:
                return False
            return bool(self.cv_map[y, x] > 200)
        except Exception as e:
            print(f"isValid exception: {type(e).__name__}: {e}")
            return False

class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')
        self.trigger_sub = self.create_subscription(Empty, 'generate_map', self.trigger_cb, 10)
        self.occ_sub = self.create_subscription(OccupancyGrid, 'box_map', self.occ_cb, 10)
        self.probe_sub = self.create_subscription(PoseArray, 'probes', self.probe_cb, 10)
        self.latest_occupancy = OccupancyGrid()
        self.latest_probes = PoseArray()
        self.done = False
        self.probe_detection_threshold = 0.2 # m

    def occ_cb(self, msg):
        if not self.done:
            self.latest_occupancy = msg

    def probe_cb(self, msg):
        if not self.done:
            self.latest_probes = msg

    @staticmethod
    def pose_distance(a, b):
        pa = a.position
        pb = b.position
        return math.sqrt((pa.x-pb.x)**2 + (pa.y-pb.y)**2 + (pa.z-pb.z)**2)
    
    def plan_rrt_star(self, cv_map, start_pt, goal_pt):
        # 1. Setup State Space (2D Plane)
        space = ob.RealVectorStateSpace(2)
        
        # Set bounds based on image dimensions
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, 0)
        bounds.setLow(1, 0)
        bounds.setHigh(0, cv_map.shape[1]) # Width
        bounds.setHigh(1, cv_map.shape[0]) # Height
        space.setBounds(bounds)

        # 2. Setup Space Information and Validity Checker
        si = ob.SpaceInformation(space)
        checker = MapValidityChecker(si, cv_map)
        si.setStateValidityChecker(checker)
        si.setup()

        # prevent OMPL variables from getting garbage collected
        self._ompl_space = space
        self._ompl_si = si
        self._ompl_checker = checker

        # 3. Define Start and Goal states
        start = si.getStateSpace().allocState()
        si.getStateSpace().copyFromReals(start, [float(start_pt[0]), float(start_pt[1])])
        goal = si.getStateSpace().allocState()
        si.getStateSpace().copyFromReals(goal, [float(goal_pt[0]), float(goal_pt[1])])

        self._ompl_start = start
        self._ompl_goal = goal      

        # 4. Setup Problem Definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
        
        # Optimization objective: Path Length (Standard for RRT*)
        obj = ob.PathLengthOptimizationObjective(si)
        pdef.setOptimizationObjective(obj)

        self._ompl_pdef = pdef

        # 5. Setup Planner (RRT*)
        planner = og.RRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        self._ompl_planner = planner

        # 6. Solve
        solved = planner.solve(1.0) # Solve for 1 second

        if solved:
            path = pdef.getSolutionPath()
            # Interpolate for a smooth list of points
            path.interpolate(100)
            
            # Convert path to a NumPy array of coordinates
            points = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                points.append([state[0], state[1]])
            
            return np.array(points)
        else:
            return None

    def trigger_cb(self, msg):
        self.get_logger().info('Trigger signal received, generating map')
        self.done = True

        # 1. convert OccupancyMap to cv::Mat

        # 0  (Free)     -> 255 (White)
        # 100 (Occupied) -> 0   (Black)

        # a. Access the data buffer and cast to a NumPy array (1.x API)
        # OccupancyGrid data is int8 (-1 to 100)
        data = np.array(self.latest_occupancy.data, dtype=np.int8)
        
        # b. Reshape into (height, width)
        # ROS 2 maps are row-major
        grid_map = data.reshape((self.latest_occupancy.info.height, self.latest_occupancy.info.width))
        
        # c. Create an empty OpenCV-compatible array (uint8)
        # We use 8-bit to display it as a standard grayscale image
        img = np.zeros(grid_map.shape, dtype=np.uint8)
        
        # d. Map the values
        # These masks handle the specific ROS occupancy conventions
        img[grid_map == -1] = 127  # Unknown space
        img[grid_map == 0] = 255   # Free space
        img[grid_map == 100] = 0   # Occupied space
        
        # e. Flip the image
        # ROS map origin is usually bottom-left, OpenCV is top-left
        img = cv2.flip(img, 0)    

        # 2. pre-process data with openCV
        # (inflate obstacles by 50cm with erosion to ensure 1m rover clearance)
        # inflate 50cm @ 10cm per cell -> inflate 5 cells (7x7 to erode 6 is closest odd kernel)
        kernel = np.ones((7,7),np.uint8)
        img = cv2.erode(img, kernel, iterations = 1)
        cv2.imwrite('eroded_map.png', img)
        self.get_logger().info(f"Raw probes: {[p.position for p in self.latest_probes.poses]}")

        # 3. filter probe positions, remove
        filtered_probes = []
        skip = False
        for i in range(len(self.latest_probes.poses)):
            for j in range(i+1, len(self.latest_probes.poses)):
                dist = self.pose_distance(self.latest_probes.poses[i], self.latest_probes.poses[j])
                self.get_logger().info(f"dist({self.latest_probes.poses[i].position}, {self.latest_probes.poses[j].position}) = {dist}")
                if dist < self.probe_detection_threshold:
                    skip = True
                    break
            if not skip:
                filtered_probes.append(self.latest_probes.poses[i])
            skip = False
        self.get_logger().info(f"Filtered probes: {filtered_probes}")

        # 4. greedy probe ordering (closest first, overall path may be suboptimal)
        ordered_probes = [[0.0, 0.0]]
        num_probes = len(filtered_probes)
        while len(ordered_probes) < num_probes + 1:
            cur = Pose()
            cur.position.x = ordered_probes[-1][0]
            cur.position.y = ordered_probes[-1][1]
            min_dist = 100
            closest = None
            for p in filtered_probes:
                d = self.pose_distance(cur, p) 
                if d < min_dist:
                    closest = p
                    min_dist = d
            filtered_probes.remove(closest)
            ordered_probes.append([closest.position.x, closest.position.y])
                    
        ordered_probes.append([0.0, 0.0])
        self.get_logger().info(f"Ordered probes: {ordered_probes}")

        res = self.latest_occupancy.info.resolution
        origin_x = self.latest_occupancy.info.origin.position.x
        origin_y = self.latest_occupancy.info.origin.position.y
        probes_pixel = []
        for p in ordered_probes:
            px = int((p[0] - origin_x) / res)
            py = int((p[1] - origin_y) / res)
            # We did (cv2.flip(img, 0)) before so we need to also 
            # flip the Y coordinate for the planner
            py = self.latest_occupancy.info.height - py
            # Clamp to valid pixel range
            px = max(0, min(px, self.latest_occupancy.info.width - 1))
            py = max(0, min(py, self.latest_occupancy.info.height - 1))
            probes_pixel.append([px, py])
        
        # 5. plan path with RRT*
        subpaths = []
        for i in range(len(probes_pixel) - 1):
            s, g = probes_pixel[i], probes_pixel[i+1]
            self.get_logger().info(f"Planning segment {i}: {s} -> {g}")
            self.get_logger().info(
                f"  Start pixel value: {img[s[1], s[0]] if 0<=s[1]<img.shape[0] and 0<=s[0]<img.shape[1] else 'OOB'}"
            )
            self.get_logger().info(
                f"  Goal  pixel value: {img[g[1], g[0]] if 0<=g[1]<img.shape[0] and 0<=g[0]<img.shape[1] else 'OOB'}"
            )
            subpaths.append(self.plan_rrt_star(img, s, g))

        # 6. draw route on obstacle map
        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for sp in subpaths:
            for i in range(len(sp) - 1):
                pt1 = tuple(sp[i].astype(int))
                pt2 = tuple(sp[i+1].astype(int))
                cv2.line(color_img, pt1, pt2, (23, 128, 30), 2)

        # 7. convert route with drawing (cv mat) to png
        if cv2.imwrite('calculated_path.png', color_img):
            print(f"Successfully saved path as image")
        else:
            print("Failed to save image.")


def main(args=None):
    rclpy.init(args=args)

    path_generator = PathGenerator()

    rclpy.spin(path_generator)

    path_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()