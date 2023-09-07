from pipeline_interfaces.srv import Classifier

import rclpy
from rclpy.node import Node

labels = ['BICYCLE', 'BUS', 'CAR', 'EMERGENCY_VEHICLE', 'MOTORCYCLE', 'PEDESTRIAN', 'TRAILER', 'TRUCK', 'VAN']

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Classifier, 'svm_inference')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Classifier.Request()

    def send_request(self):
        self.req.request = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    f'Result: {labels[response.result]}({response.result})')
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
