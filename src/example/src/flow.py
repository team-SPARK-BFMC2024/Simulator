import networkx as nx
import math
import rospy  # ROS Python API
from geometry_msgs.msg import Twist  # Message type for controlling the robot

def load_graph(file_path):
    """
    Load the graph from a .graphml file.
    """
    try:
        graph = nx.read_graphml(file_path)
        return graph
    except Exception as e:
        rospy.logerr(f"Failed to load graph: {e}")
        return None

def calculate_path(graph, start_node, end_node):
    """
    Calculate the shortest path using Dijkstra's algorithm.
    """
    try:
        path = nx.shortest_path(graph, source=start_node, target=end_node, weight='weight')
        rospy.loginfo(f"Path calculated: {path}")
        return path
    except Exception as e:
        rospy.logerr(f"Failed to calculate path: {e}")
        return []

def get_node_coordinates(graph, node):
    """
    Get the coordinates of a node in the graph.
    """
    try:
        x = float(graph.nodes[node].get('x', 0))
        y = float(graph.nodes[node].get('y', 0))
        return x, y
    except Exception as e:
        rospy.logerr(f"Failed to get coordinates for node {node}: {e}")
        return None, None

def calculate_angle_and_distance(current_position, target_position):
    """
    Calculate the angle and distance to the target position.
    """
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    distance = math.sqrt(dx**2 + dy**2)
    angle = math.atan2(dy, dx)
    return angle, distance

def control_robot(pub, angle, speed):
    """
    Send control commands to the robot.
    """
    cmd = Twist()
    cmd.linear.x = speed  # Set forward speed
    cmd.angular.z = angle  # Set angular speed
    pub.publish(cmd)

def follow_path(graph, path, pub):
    """
    Follow the path from start to end node.
    """
    current_position = (0, 0)  # Replace with actual initial position
    for next_node in path:
        target_position = get_node_coordinates(graph, next_node)
        if target_position is None:
            rospy.logerr(f"Skipping node {next_node} due to missing coordinates.")
            continue

        rospy.loginfo(f"Moving to node {next_node}: {target_position}")
        while True:
            angle, distance = calculate_angle_and_distance(current_position, target_position)
            rospy.loginfo(f"Angle: {math.degrees(angle)}, Distance: {distance}")

            if distance < 0.1:  # Threshold to consider the node reached
                rospy.loginfo(f"Reached node {next_node}.")
                current_position = target_position
                break

            control_robot(pub, angle, speed=0.5)  # Adjust speed as needed
            rospy.sleep(0.1)  # Adjust control rate

def main():
    rospy.init_node('path_follower', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Adjust topic as needed

    graph_file = "src/example/src/Competition_track_graph.graphml"
    start_node = "107"  # Replace with the actual start node ID
    end_node = "83"   # Replace with the actual end node ID

    graph = load_graph(graph_file)
    if graph is None:
        return

    path = calculate_path(graph, start_node, end_node)
    if not path:
        rospy.logerr("No valid path found.")
        return

    follow_path(graph, path, pub)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
