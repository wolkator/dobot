import serial
import struct
import subprocess
import sys

# Joint limits in degrees
# Note: joint 3 limits depend linearly on joint 2 state
# (J3MIN, J3MAX) = (J3MIN - J2, J3MAX - J2)
J1MIN = -90
J1MAX = 90
J2MIN = 0
J2MAX = 60
J3MIN = 5  # - J2
J3MAX = 70  # - J2
J4MIN = -150
J4MAX = 150


class DobotDriver:
    """
    Implementation of the Dobot Magician USB driver based on the communication
    protocol v.1.1.5. Four basic commands are implemented:
        Start homing procedure
        Get joints state
        Check if point-to-point goal for joints is valid
        Set point-to-point goal for joints
        Set suction cup on/off
    """

    def __init__(self):
        # Detect USB port connected to the Dobot and create serial connection (blocking)
        for i in range(8):
            try:
                output = subprocess.run(
                    f"udevadm info --name=/dev/ttyUSB{i} | grep Silicon_Labs_CP2102N",
                    shell=True,
                    capture_output=True,
                )
                if output.stdout:
                    print(f"Dobot Magician found at /dev/ttyUSB{i}")
                    self._usb = serial.Serial(f"/dev/ttyUSB{i}", 115200)
                    return None
            except:
                pass

        print("Dobot Magician was not found at /dev/ttyUSB1-8")
        sys.exit(1)

    def __del__(self):
        self._usb.close()

    def _send_command(
        self,
        command_id,
        ctrl_rw,
        ctrl_isqueued,
        params_struct,
        params_values,
        response_size,
    ):
        """
        Send command to Dobot with the fields:
            command_id : index of command
            ctrl_rw, ctrl_isqueued : control bits
            params_struct, params_values : C structure and values of parameters
            response_size : byte size of expected response
        """
        # Create control byte
        ctrl = ctrl_rw + (ctrl_isqueued << 1)

        # Convert command parameters to bytes and calculate the byte size
        params = struct.pack(params_struct, *params_values)
        params_size = struct.calcsize(params_struct)

        # Calculate checksum
        checksum = -(command_id + ctrl + sum(params)) % 256

        # Create packet with the fields:
        # Header (2B), len (1B), id (1B), ctrl (1B), params ({params_size}B), checksum (1B)
        packet = struct.pack(
            f"<{params_size + 6}B",
            0xAA,
            0xAA,
            params_size + 2,
            command_id,
            ctrl,
            *params,
            checksum,
        )

        # Send packet to Dobot
        self._usb.write(packet)

        # Receive and return the response
        response = self._usb.read(response_size + 6)
        assert len(response) == (
            response_size + 6
        ), f"Response byte size is {len(response)} but expected {response_size + 6}"
        return response

    # Parse response received after sending a command
    @staticmethod
    def _parse_response(params_struct, response):
        # Calculate checksum and byte size of response parameters
        checksum = -sum(response[3:-1]) % 256
        params_size = struct.calcsize(params_struct)

        # Verify checksum and byte size of response parameters
        assert (
            checksum == response[-1]
        ), f"Checksum error: calculated {checksum}, but expected {response[-1]}"
        assert params_size == (
            response[2] - 2
        ), f"Response parameters byte size is {response[2] - 2}, but expected {params_size}"

        # Unpack and return the response parameters
        params = struct.unpack(params_struct, response[5:-1])
        return params

    # Start homing procedure (ID: 31) and adjust joint values to within valid range (ID: 84)
    # Command params: 1 long (reserved)
    # Response params: (queue index)
    def start_homing(self):
        self._send_command(31, 1, 1, "<L", (0,), 8)
        # set home joint state within limits (check PTP command for details)
        self._send_command(84, 1, 1, "<B4f", (4, 0, 0, 5, 0), 8)

    # Get joints state (ID: 10)
    # Command params: none
    # Response: 8 floats (x, y, z, r, 4 joint angles)
    # Note: j3 - j2 below is because j3actual = j3dobot - j2 in the Dobot internal representation
    def get_joints(self):
        response = self._send_command(10, 0, 0, "", (), 32)
        _, _, _, _, j1, j2, j3, j4 = self._parse_response("<8f", response)
        return j1, j2, j3 - j2, j4

    # Check if point-to-point (PTP) goal for joints is valid
    def is_goal_valid(self, j1, j2, j3, j4):
        # PTP goal is invalid if target joint states are outside limits
        if (
            j1 < J1MIN
            or j1 > J1MAX
            or j2 < J2MIN
            or j2 > J2MAX
            or j3 < J3MIN - j2
            or j3 > J3MAX - j2
            or j4 < J4MIN
            or j4 > J4MAX
        ):
            return False
        else:
            return True

    # Set point-to-point (PTP) goal for joints (ID: 84)
    # Command params: 1 uint8 (4 for MOVJ) and 4 floats j1, j2, j3, j4
    # Response params: (queue index)
    # Note: j2 + j3 below is because j3dobot = j2 + j3actual in the Dobot internal representation
    def set_ptp(self, j1, j2, j3, j4):
        if self.is_goal_valid(j1, j2, j3, j4):
            self._send_command(84, 1, 1, "<B4f", (4, j1, j2, j2 + j3, j4), 8)

    # Set suction cup on/off
    # Command params: 1 uint8 for suction cup present and 1 uint8 for enable
    # (1 for on, 0 for off)
    def set_suction_cup(self, enable):
        self._send_command(62, 1, 0, "<2B", (1, 1 if enable else 0), 0)
