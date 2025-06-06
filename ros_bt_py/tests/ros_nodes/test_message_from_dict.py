# Copyright 2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import pytest
from ros_bt_py.custom_types import RosTopicType
from ros_bt_py.ros_nodes.messages_from_dict import MessageFromDict
from std_msgs.msg import Int64, String, Bool
from ros_bt_py_interfaces.srv import ChangeTreeName
from ros_bt_py_interfaces.msg import NodeState


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (RosTopicType("std_msgs/msg/Int64"), {"data": 667}),
        (RosTopicType("std_msgs/msg/String"), {"data": "this is a test string"}),
        (RosTopicType("std_msgs/msg/Bool"), {"data": True}),
    ],
)
def test_node_success(message, message_dict):
    node = MessageFromDict(options={"message_type": message})
    node.setup()
    assert node.state == NodeState.IDLE
    node.inputs["dict"] = message_dict
    node.tick()
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeState.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (RosTopicType("std_msgs/msg/Int64"), {"data": 667}),
        (RosTopicType("std_msgs/msg/String"), {"data": "this is a test string"}),
        (RosTopicType("std_msgs/msg/Bool"), {"data": True}),
    ],
)
def test_node_untick(message, message_dict):
    node = MessageFromDict(options={"message_type": message})
    node.setup()
    assert node.state == NodeState.IDLE
    node.inputs["dict"] = message_dict

    node.tick()
    assert node.state == NodeState.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.untick()
    assert node.state == NodeState.IDLE

    node.tick()
    assert node.state == NodeState.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeState.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (RosTopicType("std_msgs/msg/Int64"), {"data": 667}),
        (RosTopicType("std_msgs/msg/String"), {"data": "this is a test string"}),
        (RosTopicType("std_msgs/msg/Bool"), {"data": True}),
    ],
)
def test_node_reset(message, message_dict):
    node = MessageFromDict(options={"message_type": message})
    node.setup()
    assert node.state == NodeState.IDLE
    node.inputs["dict"] = message_dict

    node.tick()
    assert node.state == NodeState.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.reset()
    assert node.state == NodeState.IDLE

    node.tick()
    assert node.state == NodeState.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeState.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict", [(RosTopicType("std_msgs/msg/Bool"), {"tequila": False})]
)
def test_node_failure(message, message_dict):
    node = MessageFromDict(options={"message_type": message})
    node.setup()
    assert node.state == NodeState.IDLE
    node.inputs["dict"] = message_dict
    node.tick()
    assert node.state == NodeState.FAILURE

    out_message = node.outputs["message"]
    assert out_message is None
