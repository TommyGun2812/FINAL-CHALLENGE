# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult as SPR

# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declare a parameter to choose the signal type
        self.declare_parameter('Val', 'nulle')
        self.declare_parameter('Amp', 15.0)
        self.declare_parameter('Ome', 1.0)

        # Get initial value of 'Val' and ensure it's valid
        self.Val       = self.get_parameter('Val').value
        self.amplitude = self.get_parameter('Amp').value
        self.omega     = self.get_parameter('Ome').value

        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'ref', 10)
        timer_period          = 0.1  # seconds
        self.timer            = self.create_timer(timer_period, self.timer_cb)

        # Create a message and track start time
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_cb)

        self.get_logger().info("SetPoint Node Started 🚀")

    # Timer Callback: Generate and Publish Signal
    def timer_cb(self):
        # Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Generate the appropriate signal
        if self.Val == 'sen':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.Val == 'cua':
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        elif self.Val == 'nulle':
            self.signal_msg.data = 0.0
        else:
            self.signal_msg.data = self.amplitude

        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

    # Parameter Callback
    def parameter_cb(self, params):
        for param in params:
            if param.name == "Val":
                if param.value in {"sen", "cua","esc", "nulle"}:
                    self.Val = param.value
                    self.get_logger().info(f"Signal type updated to {self.Val}")
                else:
                    self.get_logger().warn(f"Invalid parameter '{param.value}'! Valid options: 'sen','cua','esc','nulle'")
                    return SPR(successful=False, reason="Invalid signal type")
            if param.name == "Amp":
                if param.value <= 15.0 and param.value > 0.0:
                    self.amplitude = param.value
                    self.get_logger().info(f"Signal type updated to {self.Val}")
                else:
                    self.get_logger().warn(f"Invalid parameter '{param.value}'! Amp must be greater than 0 and less than 15")
                    return SPR(successful=False, reason="Invalid signal type")
            if param.name == "Ome":
                if param.value <= 2.0 and param.value > 0.0:
                    self.omega = param.value
                    self.get_logger().info(f"Signal type updated to {self.Val}")
                else:
                    self.get_logger().warn(f"Invalid parameter '{param.value}'! Ome must be greater than 0 and less than 2")
                    return SPR(successful=False, reason="Invalid signal type")

        return SPR(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()