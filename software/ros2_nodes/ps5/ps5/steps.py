import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class StepOffset(Node):
    def __init__(self):
        super().__init__('step_offset')

        self.offset_base = None
        self.offset_applied = False

        self.sub_steps = self.create_subscription(
            Int32MultiArray,
            '/spm/steps',        # lo que publica madgwick_inverse_kinematics
            self.steps_callback,
            10
        )

        self.pub_steps_rel = self.create_publisher(
            Int32MultiArray,
            '/relative_steps',   # lo que mandamos a arduino
            10
        )

    def steps_callback(self, msg):
        steps = msg.data

        # Primera lectura: guardamos como referencia
        if self.offset_base is None:
            self.offset_base = steps.copy()
            self.get_logger().info(f"OFFSET BASE = {self.offset_base}")
            return

        # calculamos steps relativos
        steps_rel = [s - b for s, b in zip(steps, self.offset_base)]

        out = Int32MultiArray()
        out.data = steps_rel

        self.pub_steps_rel.publish(out)

        self.get_logger().info(f"Relativos: {steps_rel}")


def main(args=None):
    rclpy.init(args=args)
    node = StepOffset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
