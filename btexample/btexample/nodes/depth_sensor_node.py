import rclpy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig

@define_bt_node(NodeConfig(
    options={
        'min_depth': float,
        'max_depth': float
    },
    inputs={},
    outputs={
        'current_depth': float,
        'status_message': str
    },
    max_children=0
))
class DepthSensorNode(Node):
    """Depth sensor node reacting to /depth_data topic."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.latest_depth = None

        # ✅ Move subscription to constructor to make it persistent across resets
        self.sub = self.ros_node.create_subscription(
            Float32,
            '/depth_data',
            self.depth_callback,
            10
        )
        self.outputs['status_message'] = "[DepthSensorNode] Subscribed to /depth_data"

    def depth_callback(self, msg):
        self.latest_depth = msg.data
        self.ros_node.get_logger().info(f"[DepthSensorNode] Received depth: {self.latest_depth:.2f}")

    def _do_setup(self):
        # ❌ No subscription logic here anymore
        return NodeState.IDLE

    def _do_tick(self):
        if self.latest_depth is None:
            self.outputs['status_message'] = "[DepthSensorNode] Waiting for depth data..."
            return NodeState.RUNNING

        self.outputs['current_depth'] = self.latest_depth
        min_d = self.options['min_depth'] if 'min_depth' in self.options else 2.0
        max_d = self.options['max_depth'] if 'max_depth' in self.options else 5.0

        if min_d <= self.latest_depth <= max_d:
            self.outputs['status_message'] = "[DepthSensorNode] Depth OK. Awaiting next..."
            return NodeState.RUNNING

        command = 'thruster_down' if self.latest_depth < min_d else 'thruster_up'
        self.outputs['status_message'] = f"[DepthSensorNode] Depth out of range → calling {command}"

        client = self.ros_node.create_client(Trigger, command)
        if not client.wait_for_service(timeout_sec=1.0):
            self.outputs['status_message'] = f"[DepthSensorNode] {command} service not available"
            return NodeState.FAILED

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.ros_node, future, timeout_sec=2.0)

        if future.result() is not None:
            self.outputs['status_message'] = f"[DepthSensorNode] Service {command} responded: {future.result().message}"
            self.ros_node.get_logger().info(f"[DEBUG] Tick complete, status: RUNNING, msg: {self.outputs['status_message']}")
        else:
            self.outputs['status_message'] = f"[DepthSensorNode] {command} service failed"
            return NodeState.FAILED

        return NodeState.RUNNING

    def _do_reset(self):
        self.latest_depth = None
        return NodeState.IDLE

    def _do_shutdown(self):
        if self.sub is not None:
            self.ros_node.destroy_subscription(self.sub)
            self.sub = None
            self.ros_node.get_logger().info("[DepthSensorNode] Unsubscribed from /depth_data")
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE
