from action_msgs.msg import GoalStatus


def handle_nav_result(status: int) -> bool:
    return status == GoalStatus.STATUS_SUCCEEDED
