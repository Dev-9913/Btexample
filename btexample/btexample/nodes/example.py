import rclpy

# Import the ROS message class under a different name to avoid confusion.
from ros_bt_py_interfaces.msg import NodeState as NodeMsg
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

@define_bt_node(NodeConfig(
    options={'some_option': int},
    inputs={'run_child_index': int},
    outputs={},
    max_children=4
))
class MyAwesomeNode(Node):
    """Custom node for demonstration purposes."""

    def __init__(self, **kwargs):
        # Call the parent constructor (if needed, accepting any keyword arguments)
        super().__init__(**kwargs)
        # Initialize custom attributes
        self.children = []  # Ensure children list exists
        self.inputs = {}    # Ensure inputs dictionary exists

    def _do_setup(self):
        # Setup each child node, if any
        for child in self.children:
            child.setup()

    def _do_tick(self):
        # Tick the child specified by 'run_child_index' if available
        idx = self.inputs.get('run_child_index', 0)
        if 0 <= idx < len(self.children):
            return self.children[idx].tick()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Shutdown each child node
        for child in self.children:
            child.shutdown()

    def _do_reset(self):
        # Reset all child nodes and return idle state
        for child in self.children:
            child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        # Untick (deactivate) all child nodes
        for child in self.children:
            child.untick()
        return NodeMsg.IDLE


# Executable entry point for the node
def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MyAwesomeNode(passthrough_type=int)
    # In a real BT, you would integrate the node into the tree and spin accordingly.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
