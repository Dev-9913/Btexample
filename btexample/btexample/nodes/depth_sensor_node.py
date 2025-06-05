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
        self.pending_client = None
        self.pending_future = None
        self.pending_command = None

        # Persistent subscription
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
        return NodeState.IDLE

    def _do_tick(self):
        # 1. Handle pending service call from previous tick
        if self.pending_future is not None:
            if self.pending_future.done():
                result = self.pending_future.result()
                if result is not None:
                    self.outputs['status_message'] = (
                        f"[DepthSensorNode] Service {self.pending_command} responded: {result.message}"
                    )
                    self.ros_node.get_logger().info(
                        f"[DEBUG] Tick complete, status: RUNNING, msg: {self.outputs['status_message']}"
                    )
                else:
                    self.outputs['status_message'] = (
                        f"[DepthSensorNode] {self.pending_command} service failed"
                    )
                    return NodeState.FAILED

                # Reset future for next tick
                self.pending_client = None
                self.pending_future = None
                self.pending_command = None

            return NodeState.RUNNING  # Wait for future to resolve

        # 2. No future pending — check depth logic
        if self.latest_depth is None:
            self.outputs['status_message'] = "[DepthSensorNode] Waiting for depth data..."
            return NodeState.RUNNING

        self.outputs['current_depth'] = self.latest_depth
        min_d = self.options['min_depth'] if 'min_depth' in self.options else 2.0
        max_d = self.options['max_depth'] if 'max_depth' in self.options else 5.0

        if min_d <= self.latest_depth <= max_d:
            self.outputs['status_message'] = "[DepthSensorNode] Depth OK. Awaiting next..."
            return NodeState.RUNNING

        # 3. Depth out of range — initiate service call (non-blocking)
        command = 'thruster_down' if self.latest_depth < min_d else 'thruster_up'
        self.outputs['status_message'] = f"[DepthSensorNode] Depth out of range → calling {command}"
        self.pending_command = command

        self.pending_client = self.ros_node.create_client(Trigger, command)
        if not self.pending_client.wait_for_service(timeout_sec=0.5):
            self.outputs['status_message'] = f"[DepthSensorNode] {command} service not available"
            self.pending_client = None
            self.pending_command = None
            return NodeState.FAILED

        self.pending_future = self.pending_client.call_async(Trigger.Request())
        return NodeState.RUNNING

    def _do_reset(self):
        self.latest_depth = None
        self.pending_future = None
        self.pending_client = None
        self.pending_command = None
        return NodeState.IDLE

    def _do_shutdown(self):
        if self.sub is not None:
            self.ros_node.destroy_subscription(self.sub)
            self.sub = None
            self.ros_node.get_logger().info("[DepthSensorNode] Unsubscribed from /depth_data")
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE
