from geometry_msgs.msg import PoseArray, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

def toQuaternion(yaw):
    pitch = 0.0
    roll = 0.0
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def getCurrentmsg(map):
    pa = PoseArray()
    pa.header.frame_id = "map"
    for i in range(0,map.width,10):
        for j in range(0,map.height,10):
            x = i*map.res
            y = j*map.res
            p = Pose()
            p.position.x = float(x) + map.pos[0]
            p.position.y = float(y) + map.pos[1]
            p.position.z = 0.0

            cx, cy = map.current.current_x[j,i],map.current.current_y[j,i]
            yaw = np.arctan2(cy,cx)
            p.orientation = toQuaternion(yaw)
            pa.poses.append(p)
    return pa
def getGoalmsg(goal):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.seq = 0
    marker.id = 0
    marker.type = 2 #sphere
    marker.pose.orientation = toQuaternion(0.0)

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.g = 255.0
    marker.color.a = 1

    marker.pose.position.x = goal[0]
    marker.pose.position.y = goal[1]
    marker.pose.position.z = 0.0
    return marker

def getPathmsg(path):
    patharray = MarkerArray()

    for i in range(len(path)-1):
        currentx = path[i][0]
        currenty = path[i][1]

        nextx = path[i+1][0]
        nexty = path[i+1][1]
        speed = path[i+1][2]

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.seq = i
        marker.id = i
        marker.type = 2 #sphere
        dx = nextx - currentx
        dy = nexty - currenty
        yaw = np.arctan2(dy,dx)
        marker.pose.orientation = toQuaternion(yaw)

        # marker.scale.y = 0.05*speed
        # marker.scale.x = 0.05*speed
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.g = 255.0
        marker.color.a = 1

        marker.pose.position.x = currentx
        marker.pose.position.y = currenty
        marker.pose.position.z = 0.0



        patharray.markers.append(marker)

    return patharray
