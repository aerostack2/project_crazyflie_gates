import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.01, self.publish_tf)

    def publish_tf(self):
        tf_msgs = []

        # TF for /wind_turbine/nacelle
        wind_turbine_tf = TransformStamped()
        wind_turbine_tf.header.stamp = self.get_clock().now().to_msg()
        wind_turbine_tf.header.frame_id = 'earth'
        wind_turbine_tf.child_frame_id = 'wind_turbine/nacelle'
        wind_turbine_tf.transform.translation.x = 1.0  # Adjust the position as needed
        wind_turbine_tf.transform.translation.y = 0.0
        wind_turbine_tf.transform.translation.z = 0.0
        wind_turbine_tf.transform.rotation.x = 0.0
        wind_turbine_tf.transform.rotation.y = 0.0
        wind_turbine_tf.transform.rotation.z = 0.0
        wind_turbine_tf.transform.rotation.w = 1.0
        tf_msgs.append(wind_turbine_tf)

        # TF for /drone0/base_link
        wind_turbine_tf = TransformStamped()
        wind_turbine_tf.header.stamp = self.get_clock().now().to_msg()
        wind_turbine_tf.header.frame_id = 'earth'
        wind_turbine_tf.child_frame_id = 'drone0/base_link'
        wind_turbine_tf.transform.translation.x = -2.0  # Adjust the position as needed
        wind_turbine_tf.transform.translation.y = 0.0
        wind_turbine_tf.transform.translation.z = 0.0
        wind_turbine_tf.transform.rotation.x = 0.0
        wind_turbine_tf.transform.rotation.y = 0.0
        wind_turbine_tf.transform.rotation.z = 0.0
        wind_turbine_tf.transform.rotation.w = 1.0
        tf_msgs.append(wind_turbine_tf)

        # TF for /drone0/base_link/camera
        drone_camera_tf = TransformStamped()
        drone_camera_tf.header.stamp = self.get_clock().now().to_msg()
        drone_camera_tf.header.frame_id = 'drone0/base_link'
        drone_camera_tf.child_frame_id = 'drone0/base_link/camera'
        drone_camera_tf.transform.translation.x = 0.0
        drone_camera_tf.transform.translation.y = 0.0
        drone_camera_tf.transform.translation.z = 0.1  # Adjust the position as needed
        drone_camera_tf.transform.rotation.x = 0.0
        drone_camera_tf.transform.rotation.y = 0.0
        drone_camera_tf.transform.rotation.z = 0.0
        drone_camera_tf.transform.rotation.w = 1.0
        tf_msgs.append(drone_camera_tf)

        for tf_msg in tf_msgs:
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher_node = TFPublisherNode()
    rclpy.spin(tf_publisher_node)
    tf_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
