from action_msgs.msg import GoalStatus

from vehicle_mission.nav_result import handle_nav_result


def test_succeeded_status_returns_true():
    assert handle_nav_result(GoalStatus.STATUS_SUCCEEDED) is True


def test_aborted_status_returns_false():
    assert handle_nav_result(GoalStatus.STATUS_ABORTED) is False


def test_canceled_status_returns_false():
    assert handle_nav_result(GoalStatus.STATUS_CANCELED) is False
