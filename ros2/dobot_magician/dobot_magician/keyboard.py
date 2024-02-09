import evdev
import subprocess
import sys

class Keyboard:
    def __init__(self, keys):
        for i in range(8):
            try:
                output = subprocess.run(
                    f"udevadm info --name=/dev/input/event{i} | grep KEYBOARD",
                    shell=True,
                    capture_output=True,
                )
                if output.stdout:
                    self._device = evdev.InputDevice(f"/dev/input/event{i}")
                    self._keys = keys
                    return None
            except:
                pass

        print("Keyboard not found at /dev/input/event[0-7]")
        sys.exit(1)

    def read_key(self):
        event = self._device.read_one()
        if event is not None and event.type == evdev.ecodes.EV_KEY:
            event = evdev.categorize(event)
            if event.keycode in self._keys and event.keystate in {0, 1}:
                return (event.keycode, event.keystate)
        else:
            return None
