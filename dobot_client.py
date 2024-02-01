import socket
import struct
import time


class DobotClient:
    """
    Implementation of the Dobot Magician client based on sockets.
    A custom protocol is used (see dobot_server.py for details)
    """

    def __init__(self):
        self._sock = socket.socket()

        while True:
            try:
                time.sleep(1)
                with open("/tmp/DOBOT_MAGICIAN_PORT", "r") as f:
                    port = int(f.read())
                self._sock.connect(("localhost", port))
                break
            except:
                pass

    def start_homing(self):
        self._sock.send(struct.pack("<1B4f", 0, 0.0, 0.0, 0.0, 0.0))

    def get_joints(self):
        self._sock.send(struct.pack("<1B4f", 1, 0.0, 0.0, 0.0, 0.0))
        data = self._sock.recv(struct.calcsize("<4f"))
        response = struct.unpack("<4f", data)
        return response

    def is_goal_valid(self, j1, j2, j3, j4):
        self._sock.send(struct.pack("<1B4f", 2, j1, j2, j3, j4))
        data = self._sock.recv(struct.calcsize("<1B"))
        (response,) = struct.unpack("<1B", data)
        return bool(response)

    def set_ptp(self, j1, j2, j3, j4):
        self._sock.send(struct.pack("<1B4f", 3, j1, j2, j3, j4))

    def set_suction_cup(self, enable):
        if enable:
            self._sock.send(struct.pack("<1B4f", 4, 0.0, 0.0, 0.0, 0.0))
        else:
            self._sock.send(struct.pack("<1B4f", 5, 0.0, 0.0, 0.0, 0.0))
