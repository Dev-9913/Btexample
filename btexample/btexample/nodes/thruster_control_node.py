import rclpy
from std_srvs.srv import Trigger

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig

@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=0
))
class ThrusterControlNode(Node):
    """Dummy thruster control node which provides two services."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service_up = None
        self.service_down = None

    def _do_setup(self):
        self.service_up = self.ros_node.create_service(Trigger, 'thruster_up', self.handle_thruster_up)
        self.service_down = self.ros_node.create_service(Trigger, 'thruster_down', self.handle_thruster_down)
        self.ros_node.get_logger().info("[ThrusterControlNode] Services 'thruster_up' and 'thruster_down' are ready")
        return NodeState.IDLE

    def handle_thruster_up(self, request, response):
        response.success = True
        response.message = "Activating upward thrusters"
        self.ros_node.get_logger().info("[ThrusterControlNode] Service call: UP → Activating upward thrusters")
        return response

    def handle_thruster_down(self, request, response):
        response.success = True
        response.message = "Activating downward thrusters"
        self.ros_node.get_logger().info("[ThrusterControlNode] Service call: DOWN → Activating downward thrusters")
        return response

    def _do_tick(self):
        # This node simply hosts services — no real tick logic.
        self.ros_node.get_logger().info("[ThrusterControlNode] Ticked (no-op — services wait for external call)")
        return NodeState.RUNNING  # stay alive so services stay up

    def _do_shutdown(self):
        if self.service_up is not None:
            self.ros_node.destroy_service(self.service_up)
            self.service_up = None
        if self.service_down is not None:
            self.ros_node.destroy_service(self.service_down)
            self.service_down = None
        self.ros_node.get_logger().info("[ThrusterControlNode] Services shut down")
        return NodeState.IDLE

    def _do_reset(self):
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE
