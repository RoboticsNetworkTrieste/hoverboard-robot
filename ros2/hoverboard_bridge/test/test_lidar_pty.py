import os
import select
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='pty is Linux only')


def test_bytes_flow_both_ways(tmp_path):
    from hoverboard_bridge.lidar_pty import LidarPty
    link = str(tmp_path / 'ttyLIDAR')
    lidar = LidarPty(link)
    try:
        driver = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            lidar.write(b'\x54\x2c')
            assert select.select([driver], [], [], 1.0)[0]
            assert os.read(driver, 2) == b'\x54\x2c'
            os.write(driver, b'\xa5\x60')
            data = b''
            deadline = time.monotonic() + 1.0
            while len(data) < 2 and time.monotonic() < deadline:
                data += lidar.read()
                time.sleep(0.005)
            assert data == b'\xa5\x60'
        finally:
            os.close(driver)
    finally:
        lidar.close()
    assert not os.path.lexists(link)


def test_existing_regular_file_is_not_replaced(tmp_path):
    from hoverboard_bridge.lidar_pty import LidarPty
    link = tmp_path / 'ttyLIDAR'
    link.write_text('keep')
    with pytest.raises(FileExistsError):
        LidarPty(str(link))
    assert link.read_text() == 'keep'


def test_close_is_idempotent(tmp_path):
    from hoverboard_bridge.lidar_pty import LidarPty
    link = str(tmp_path / 'ttyLIDAR')
    lidar = LidarPty(link)
    lidar.close()
    lidar.close()
    assert not os.path.lexists(link)
