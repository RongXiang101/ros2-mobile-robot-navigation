"""Unit tests for frontier goal coordinates and action readiness."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData
import pytest

from frontier_explorer import FrontierExplorer


@pytest.mark.parametrize(
    ('resolution', 'origin_xy', 'yaw', 'cell', 'expected_xy'),
    [
        (1.0, (0.0, 0.0), 0.0, (0, 0), (0.5, 0.5)),
        (0.05, (-6.0, 2.0), 0.0, (3, 5), (-5.825, 2.275)),
        (0.2, (1.0, -2.0), math.pi / 2, (2, 1), (0.7, -1.5)),
        (0.4, (-1.0, 3.0), -math.pi / 4, (1, 0),
         (-1.0 + 0.4 * math.sqrt(2), 3.0 - 0.2 * math.sqrt(2))),
    ],
)
def test_frontier_uses_grid_cell_center(resolution, origin_xy, yaw, cell, expected_xy):
    info = MapMetaData(resolution=resolution)
    info.origin.position.x, info.origin.position.y = origin_xy
    info.origin.orientation.z = math.sin(yaw / 2.0)
    info.origin.orientation.w = math.cos(yaw / 2.0)

    goal = FrontierExplorer.frontier_to_goal(SimpleNamespace(map_info=info), cell)

    assert goal.header.frame_id == 'map'
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx(expected_xy)
    assert goal.pose.orientation.w == 1.0


def test_navigation_unavailable_releases_active_goal_without_waiting():
    goal = PoseStamped()
    client = Mock()
    client.server_is_ready.return_value = False
    explorer = SimpleNamespace(
        nav_client=client, exploring=True, active_goal=goal, get_logger=Mock(),
    )

    FrontierExplorer.send_goal(explorer, goal)

    client.server_is_ready.assert_called_once_with()
    client.wait_for_server.assert_not_called()
    client.send_goal_async.assert_not_called()
    assert not explorer.exploring
    assert explorer.active_goal is None
