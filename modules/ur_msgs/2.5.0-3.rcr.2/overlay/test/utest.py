import pytest
from ur_msgs.msg import Analog, Digital, IOStates
from ur_msgs.srv import SetIO
from ur_msgs.action import ToolContact

def test_messages():
    msg = Analog()
    msg.pin = 1
    msg.domain = Analog.VOLTAGE
    msg.state = 3.14
    assert msg.pin == 1
    assert msg.domain == Analog.VOLTAGE
    assert abs(msg.state - 3.14) < 1e-5

def test_services():
    req = SetIO.Request()
    resp = SetIO.Response()
    req.fun = 1
    req.pin = 2
    req.state = 3.5
    resp.success = True
    assert req.fun == 1
    assert req.pin == 2
    assert req.state == 3.5
    assert resp.success

def test_actions():
    goal = ToolContact.Goal()
    assert goal is not None
