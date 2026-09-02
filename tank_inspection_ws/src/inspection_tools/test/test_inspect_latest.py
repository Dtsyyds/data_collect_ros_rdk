import os

from inspection_tools.inspect_latest import latest_closed_bag
import pytest


def test_latest_closed_bag_ignores_unclosed_directory(tmp_path):
    older = tmp_path / "eddy_sync_loop_older"
    older.mkdir()
    (older / "metadata.yaml").write_text("closed: true\n")
    os.utime(older, (10, 10))

    newer = tmp_path / "eddy_sync_loop_newer"
    newer.mkdir()
    (newer / "metadata.yaml").write_text("closed: true\n")
    os.utime(newer, (20, 20))

    unclosed = tmp_path / "eddy_sync_loop_in_progress"
    unclosed.mkdir()
    os.utime(unclosed, (30, 30))

    assert latest_closed_bag(tmp_path) == newer


def test_latest_closed_bag_reports_missing_capture(tmp_path):
    with pytest.raises(ValueError, match="no closed"):
        latest_closed_bag(tmp_path)
