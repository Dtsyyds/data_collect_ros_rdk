from types import SimpleNamespace

from inspection_rerun.tank_scene_adapter import ActualTrajectoryLogger

from test_tank_scene import message, write_scene


class FakeSession:
    def __init__(self):
        self.calls = []

    def log(self, path, *entities, **kwargs):
        self.calls.append((path, len(entities), kwargs))


def test_actual_trajectory_uses_linked_2d_and_3d_entities(tmp_path):
    scene = write_scene(tmp_path)
    logger = ActualTrajectoryLogger(scene, min_step_m=0.01, max_segments=3)
    session = FakeSession()
    assert logger.log(session, message(s_m=1.0))
    assert logger.log(session, message(s_m=1.1))
    assert [call[0] for call in session.calls] == [
        "world/inspection/actual_position",
        "world/inspection/actual_position",
        "world/inspection/actual_trajectory/segments/00000",
    ]
    assert session.calls[-1][1] == 2


def test_actual_trajectory_decimates_short_steps(tmp_path):
    scene = write_scene(tmp_path)
    logger = ActualTrajectoryLogger(scene, min_step_m=0.5, max_segments=3)
    session = FakeSession()
    logger.log(session, message(s_m=1.0))
    logger.log(session, message(s_m=1.1))
    assert not any("segments" in call[0] for call in session.calls)


def test_actual_trajectory_does_not_connect_reposition_jump(tmp_path):
    scene = write_scene(tmp_path)
    logger = ActualTrajectoryLogger(scene, min_step_m=0.01, max_segments=3,
                                    max_step_m=0.5)
    session = FakeSession()
    logger.log(session, message(s_m=1.0))
    logger.log(session, message(s_m=3.0))
    assert not any("segments" in call[0] for call in session.calls)
