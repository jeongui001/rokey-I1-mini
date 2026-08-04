from vehicle_mission.vehicle_mission_node import MissionState


def test_mission_states_include_undock_and_waypoint():
    assert MissionState.UNDOCKING.name == 'UNDOCKING'
    assert MissionState.NAVIGATING_TO_WAYPOINT.name == 'NAVIGATING_TO_WAYPOINT'
    assert MissionState.APPROACH_ENABLED.name == 'APPROACH_ENABLED'
