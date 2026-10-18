"""Pseudo terminal that exposes the LIDAR byte stream to a serial LIDAR driver."""
import os
import pty
import tty


class LidarPty:
    """Pty with a fixed symlink; the slave fd stays open to avoid EIO on driver restart."""

    def __init__(self, link_path):
        self.link_path = link_path
        self._closed = False
        self._master, self._slave = pty.openpty()
        try:
            tty.setraw(self._slave)  # No echo or line editing; the driver may reconfigure it
            os.set_blocking(self._master, False)
            self.slave_name = os.ttyname(self._slave)
            if os.path.islink(link_path):
                os.remove(link_path)
            os.symlink(self.slave_name, link_path)
        except BaseException:
            os.close(self._master)
            os.close(self._slave)
            raise

    def write(self, data):
        """Bytes from the LIDAR; dropped if the driver is not reading."""
        try:
            os.write(self._master, data)
        except BlockingIOError:
            pass

    def read(self, max_bytes=512):
        """Bytes written by the driver, to be forwarded to the LIDAR."""
        try:
            return os.read(self._master, max_bytes)
        except BlockingIOError:
            return b''

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            # Remove the link only if it still points to this pty
            if os.path.islink(self.link_path) and os.readlink(self.link_path) == self.slave_name:
                os.remove(self.link_path)
        finally:
            try:
                os.close(self._master)
            finally:
                os.close(self._slave)
