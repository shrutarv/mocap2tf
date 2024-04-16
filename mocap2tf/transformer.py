import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mocap4r2_msgs.msg import RigidBodies


class MinimalSubscriber(Node):
    """Test"""

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            RigidBodies,
            'rigid_bodies',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        """Callback"""
        # self.get_logger().info('I heard: "%s"' % msg)

        rigidbodies = msg.rigidbodies

        for rigidbody in rigidbodies:
            # geometry_msgs/Pose
            pose = rigidbody.pose

            # geometry_msgs/Point
            position = pose.position
            # geometry_msgs/Quaternion
            orientation = pose.orientation



            # std_msgs/msg/Header header
            # string child_frame_id
            # geometry_msgs/msg/Transform transform
            #   geometry_msgs/msg/Vector3 translation
            #   geometry_msgs/msg/Quaternion rotation
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = str(rigidbody.rigid_body_name).split('.')[0]

            t.transform.translation.x = position.x
            t.transform.translation.y = position.y
            t.transform.translation.z = position.z

            t.transform.rotation = orientation

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Test"""
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
