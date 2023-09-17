# Import the necessary packages
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D
from message_filters import ApproximateTimeSynchronizer, Subscriber

from track_fuser import TrackFuser

# Define a node class that subscribes to two detection3d array topics and fuses them


class Detection3DFusionNode(Node):

    def __init__(self):
        # Initialize the node with a name
        super().__init__('detection3d_fusion_node')

        # Create a track fuser object with a cross-covariance fusion algorithm
        self.track_fuser = TrackFuser('Cross')

        # Create subscribers for the two detection3d array topics
        self.detection3d_sub1 = Subscriber(
            self, Detection3DArray, 'detection3d_array1')
        self.detection3d_sub2 = Subscriber(
            self, Detection3DArray, 'detection3d_array2')

        # Create an approximate time synchronizer with a queue size of 10 and a slop of 0.1 seconds
        self.ats = ApproximateTimeSynchronizer(
            [self.detection3d_sub1, self.detection3d_sub2], 10, 0.1)

        # Register the callback function for the synchronizer
        self.ats.registerCallback(self.fusion_callback)

        # Create a publisher for the fused detection3d array topic
        self.fused_detection3d_pub = self.create_publisher(
            Detection3DArray, 'fused_detection3d_array', 10)

    def fusion_callback(self, detection3d_array1, detection3d_array2):
        # This function is called when two detection3d arrays are approximately synchronized

        # Convert the detection3d arrays to tracks (a list of state vectors and covariance matrices)
        track1 = self.detection3d_array_to_track(detection3d_array1)
        track2 = self.detection3d_array_to_track(detection3d_array2)

        # Fuse the tracks using the track fuser object
        fused_track = self.track_fuser.fuse(track1, track2)

        # Convert the fused track to a detection3d array
        fused_detection3d_array = self.track_to_detection3d_array(fused_track)

        # Publish the fused detection3d array to a topic
        self.get_logger().info('Publishing fused detection3d array')
        self.fused_detection3d_pub.publish(fused_detection3d_array)

    def detection3d_array_to_track(self, detection3d_array):
        # This function converts a detection3d array message to a track (a list of state vectors and covariance matrices)
        # For simplicity, we assume that each detection in the array corresponds to one object and has a unique ID
        # We also assume that the state vector consists of [x, y, z, vx, vy, vz] and the covariance matrix is 6x6

        track = []
        for detection in detection3d_array.detections:
            # Get the ID of the detection
            id = detection.header.frame_id

            # Get the position and velocity of the detection from the bounding box pose and twist
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            z = detection.bbox.center.position.z
            vx = detection.bbox.center.velocity.linear.x
            vy = detection.bbox.center.velocity.linear.y
            vz = detection.bbox.center.velocity.linear.z

            # Get the covariance matrix of the detection from the bounding box pose and twist covariance
            cov = detection.bbox.covariance

            # Construct the state vector and covariance matrix for the track
            state = [x, y, z, vx, vy, vz]
            cov_matrix = [[cov[0], cov[1], cov[2], cov[6], cov[7], cov[8]],
                          [cov[12], cov[13], cov[14], cov[18], cov[19], cov[20]],
                          [cov[24], cov[25], cov[26], cov[30], cov[31], cov[32]],
                          [cov[36], cov[37], cov[38], cov[42], cov[43], cov[44]],
                          [cov[48], cov[49], cov[50], cov[54], cov[55], cov[56]],
                          [cov[60], cov[61], cov[62], cov[66], cov[67], cov[68]]]

            # Append the state vector and covariance matrix to the track list with the ID as a key
            track.append((id, state, cov_matrix))

        return track

    def track_to_detection3d_array(self, track):
        # This function converts a track (a list of state vectors and covariance matrices) to a detection3d array message
        # For simplicity, we assume that each state vector and covariance matrix in the track corresponds to one detection and has a unique ID
        # We also assume that the state vector consists of [x, y, z, vx, vy, vz] and the covariance matrix is 6x6

        detection3d_array = Detection3DArray()
        for id, state, cov_matrix in track:
            # Create a detection3d message
            detection = Detection3D()

            # Set the ID of the detection as the frame_id
            detection.header.frame_id = id

            # Set the position and velocity of the detection from the state vector
            detection.bbox.center.position.x = state[0]
            detection.bbox.center.position.y = state[1]
            detection.bbox.center.position.z = state[2]
            detection.bbox.center.velocity.linear.x = state[3]
            detection.bbox.center.velocity.linear.y = state[4]
            detection.bbox.center.velocity.linear.z = state[5]

            # Set the covariance matrix of the detection from the covariance matrix
            cov = [0] * 72
            cov[0] = cov_matrix[0][0]
            cov[1] = cov_matrix[0][1]
            cov[2] = cov_matrix[0][2]
            cov[6] = cov_matrix[0][3]
            cov[7] = cov_matrix[0][4]
            cov[8] = cov_matrix[0][5]
            cov[12] = cov_matrix[1][0]
            cov[13] = cov_matrix[1][1]
            cov[14] = cov_matrix[1][2]
            cov[18] = cov_matrix[1][3]
            cov[19] = cov_matrix[1][4]
            cov[20] = cov_matrix[1][5]
            cov[24] = cov_matrix[2][0]
            cov[25] = cov_matrix[2][1]
            cov[26] = cov_matrix[2][2]
            cov[30] = cov_matrix[2][3]
            cov[31] = cov_matrix[2][4]
            cov[32] = cov_matrix[2][5]
            cov[36] = cov_matrix[3][0]
            cov[37] = cov_matrix[3][1]
            cov[38] = cov_matrix[3][2]
            cov[42] = cov_matrix[3][3]
            cov[43] = cov_matrix[3][4]
            cov[44] = cov_matrix[3][5]
            cov[48] = cov_matrix[4][0]
            cov[49] = cov_matrix[4][1]
            cov[50] = cov_matrix[4][2]
            cov[54] = cov_matrix[4][3]
            cov[55] = cov_matrix[4][4]
            cov[56] = cov_matrix[4][5]
            cov[60] = cov_matrix[5][0]
            cov[61] = cov_matrix[5][1]
            cov[62] = cov_matrix[5][2]
            cov[66] = cov_matrix[5][3]
            cov[67] = cov_matrix[5][4]
            cov[68] = cov_matrix[5][5]

        # Set the covariance of the bounding box
        detection.bbox.covariance = cov

        # Append the detection to the detection3d array message
        detection3d_array.detections.append(detection)

        return detection3d_array


def main(args=None):
    rclpy.init(args=args)
    node = Detection3DFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
