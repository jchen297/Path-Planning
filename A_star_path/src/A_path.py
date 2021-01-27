import json
import numpy as np
import rospkg
import math
import rospy

################################# A* Path Finding
def eu_dis(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


class node:
    def __init__(self, xy, g_h=[0, 0], previous_xy=None):
        reform = lambda xy: [abs(xy[1] * 2 - 18) if xy[1] >= 0 else 18 + 2 * abs(xy[1]), xy[0] * 2 + 18]
        self.location = xy
        self.map_location = reform(xy)
        self.g_cost = g_h[0]
        self.h_cost = g_h[1]
        self.f_cost = self.g_cost + self.h_cost
        self.came_from = previous_xy

    def update(self):
        self.f_cost = self.g_cost + self.h_cost


def compute_costs(p, came_from=node([-8, -2])):
    global p2
    g_cost = eu_dis(p, came_from.location) + came_from.g_cost
    h_cost = eu_dis(p, p2)
    return [g_cost, h_cost]


def check_location_limit(x, y):
    if x >= -9 and x <= 8.5 and y >= -10.5 and y <= 9:
        return True
    else:
        return False


def get_neigbers(node):
    global map2D

    neigbers = []
    x, y = node.location
    reform = lambda xy: [int(abs(xy[1] * 2 - 18)) if xy[1] >= 0 else int(18 + 2 * abs(xy[1])), int(xy[0] * 2 + 18)]

    for xi in range(-1, 2, 1):
        xi = xi * 0.5
        for yi in range(-1, 2, 1):
            yi = yi * 0.5
            # ignore current node
            if xi == 0 and yi == 0:
                continue
            neigber_x, neigber_y = x + xi, y + yi
            if check_location_limit(neigber_x, neigber_y):
                # check the location on the map, if the path is not blocked
                map_x, map_y = reform([neigber_x, neigber_y])

                if map2D[map_x, map_y] == 0:
                    neigbers.append([neigber_x, neigber_y])

    return neigbers


# Core A* Path Finding Function
def A_Star_PathFinding(p1,p2):
    print("----- Finding the Path --------")
    # create start and goal node
    start = node(p1,compute_costs(p1))
    goal = node(p2,compute_costs(p2))

    # lists to store nodes
    open_nodes = [start]
    close_nodes = []

    open_temp = []
    temp = 0
    # while open_nodes is not empty
    while len(open_nodes) > 0 :
        # get the lowest f cost node from open_nodes
        current = min(open_nodes,key = lambda point: point.f_cost)

        # if the current node is our goal, then retrun path
        if current.location == goal.location:
            print('Found Path !!!!!!!!!!')
            path = []
            while current.came_from:
                path.append(current.location)
                current = current.came_from
            return path

        # remove current node from open nodes list and add it to close nodes list
        open_nodes.remove(current)
        close_nodes.append(current)

        # get and form neigber nodes to node class
        mapping_points = lambda p: node(xy = p, g_h = compute_costs(p,current),previous_xy = current)
        neigbers = list(map(mapping_points,get_neigbers(current)))

        # get open/close nodes' locations
        open_nodes_loaction = list(map(lambda x:x.location,open_nodes))
        close_nodes_loaction = list(map(lambda x:x.location,close_nodes))
        for neigber in neigbers:
            # if neigber is in close nodes list skip
            if neigber.location in close_nodes_loaction:
                continue

            # if neigber is in open nodes list, compare their g cost
            if neigber.location in open_nodes_loaction:
                ind = open_nodes_loaction.index(neigber.location)
                # if neigber has better g cost, then replace the node in open nodes list, and assign previous node
                if neigber.g_cost < open_nodes[ind].g_cost:
                    open_nodes[ind].g_cost = neigber.g_cost
                    open_nodes[ind].came_from = current
                    # recalculate f score
                    open_nodes[ind].update()
            else:
                open_nodes.append(neigber)
#                 print(f"current node{current.location}**",end="\r", flush=True)
    #             print(f"**{open_nodes_loaction}**")
    print('Fail to find the path')


######### Load Map in 20x18 and transform it to 40x36
map_data = ''

# get the file path for lab4
rospack = rospkg.RosPack()
file_path = rospack.get_path('lab4')
with open(file_path + '/src/map.txt', 'r') as f:
    for i in f:
        map_data += i.strip()
map_data = np.array(json.loads(map_data[5:])).reshape(20, 18)
map_data_temp = map_data.copy()
for x in range(0, 18):
    for y in range(0, 15):
        lis = map_data_temp[x:x + 2, y:y + 2]
        if lis[0][1] == 0 and lis[0][0] == lis[1][1] == 1:
            map_data_temp[x, y + 1] = 1

reform_map = lambda x: np.tile(x.repeat(2), (2, 1))
map2D = np.array(list(map(reform_map, map_data_temp))).reshape(40, 36)


# Print Path
def print_path(path,map_data):
    orignal_map = list(map(lambda x:list(x),map_data))

    reform_path = lambda xy: [9-int(xy[1]),int(xy[0])+9]
    path_points = list(map(reform_path,path))

    for path_point in path_points:
        x,y = path_point
        if orignal_map[x][y] != 1:
            orignal_map[x][y]= 4
    for i in orignal_map:
        print(i)

############################################ move turtle
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import sys

def callback(data):
    global pos, orientation
    pos = data.pose.pose.position
    orientation = data.pose.pose.orientation


# variables
pos, orientation = 0, 0
Status = 'Waiting Path'

# ROS SET-UP
rospy.init_node('robot_broadcaster', anonymous=True)
sub_truth = rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

# get location for start point and goal
# p1 = [-8,-2]
# p2 = [4.5,9]
p1 = rospy.get_param("start_postion")
p2 = rospy.get_param("goal")

# Find Path
path = A_Star_PathFinding(p1, p2)

# Print Path on the map
print_path(path, map_data)

print('============ Start =============')
path_status = len(path)-1
current_path_point = path[path_status]
path_len = 0
theta = 999
while not rospy.is_shutdown():

    if orientation != 0:
        twist = Twist()
        # get robot current angle
        roll, pitch, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # compute angle to the current goal
        path_angle = round(math.atan2( current_path_point[1]-pos.y, current_path_point[0]-pos.x),2)

        # compute distance between current position and current goal
        dist_to_path = round(math.sqrt((current_path_point[0] - pos.x) ** 2 + (current_path_point[1] - pos.y) ** 2),2)

        # initial twist
        angle = 0
        vel = 0

        # if arrive current goal
        if dist_to_path < 0.1:
            # stop and update path_status
            path_status -= 1
            vel = 0
            # if points in path then change to next path point
            if path_status >= 0:
                current_path_point = path[path_status]
            else:
                # the robot has arrived at goal
                print('--------- Arrive!!!!-----------')
                print_path(path, map_data)
                break
        # if not arrive current goal
        else:
            # the angle of the robot is not set to the angle that points to the current goal
            if abs(path_angle - theta) > 0.1:
                Status = 'Turning to the next path point'
                # stop
                vel = 0
                # check current robot angle and make turn
                if theta <= 0:
                    threthhold = 3.14 + theta
                    if threthhold > path_angle > theta:
                        angle = 0.5
                    else:
                        angle = -0.5
                else:
                    threthhold = -3.14 + theta
                    if theta > path_angle > threthhold:
                        angle = -0.5
                    else:
                        angle = 0.5
            # the robot angle is set to the current goal, just go straight
            else:
                Status = 'Follow the path'
                angle = 0
                vel = 1
        # publish twist
        twist.angular.z = angle
        twist.linear.x = vel
        pub.publish(twist)
        print(f'** Current Status:{Status}\n Current location: {round(pos.x,1),round(pos.y,1)}\n Heading to: {current_path_point}\n')
        rate.sleep()


