from pathlib import Path
import sys
from unittest.mock import MagicMock, patch
from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from autonomy.drone_controller import DroneController

def test_missing_telemetry() -> None:
    drone = DroneController()

    fake_connection = MagicMock()
    fake_connection.recv_match.return_value = None

    drone.connection = fake_connection

    try:
        drone.wait_until_landed(timeout=0.1)
    except TimeoutError:
        print("PASS: Missing telemetry caused timeout.")
    else:
        raise AssertionError(
            "Missing telemetry was incorrectly accepted."
        )



def test_stale_landing_telemetry() -> None:

    """I wrote this test funcction to prevent old telemetry data from the drone from causing an incorrect landing confirmation. For example, if a message indicating that the drone is on the ground is received, and four seconds later another message arrives stating that the motors are off, the first message is no longer considered current. In this case, the software should not consider the landing successful. We are testing this using fake MAVLink messages and simulated time, without actually flying the drone.

    The importance of this is that in flight safety, data must not only be correct but also current."""
    drone = DroneController()
    fake_connection = MagicMock()

    fake_connection.target_system = 1
    fake_connection.target_component = 1
    drone.connection = fake_connection

    # Simulated clock starts at zero.
    clock = [0.0]

    ground_message = MagicMock()
    ground_message.get_type.return_value = "EXTENDED_SYS_STATE"
    ground_message.get_srcSystem.return_value = 1
    ground_message.get_srcComponent.return_value = 1
    ground_message.landed_state = (
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    )

    disarmed_message = MagicMock()
    disarmed_message.get_type.return_value = "HEARTBEAT"
    disarmed_message.get_srcSystem.return_value = 1
    disarmed_message.get_srcComponent.return_value = 1
    disarmed_message.base_mode = 0

    messages = [ground_message, disarmed_message]
    received_count = [0]

    def fake_recv_match(**kwargs):
        index = received_count[0]
        received_count[0] += 1

        if index == 0:
            clock[0] = 0.5
            return messages[0]

        if index == 1:
            clock[0] = 4.0
            return messages[1]

        clock[0] = 6.5
        return None

    fake_connection.recv_match.side_effect = fake_recv_match

    with patch(
        "autonomy.drone_controller.time.monotonic",
        side_effect=lambda: clock[0]
    ):
        try:
            drone.wait_until_landed(timeout=6)
        except TimeoutError:
            assert received_count[0] >= 3
            print("PASS: Stale landing telemetry rejected.")
        else:
            raise AssertionError(
                "Stale telemetry incorrectly confirmed landing."
            )


def test_armed_drone_on_ground() -> None:
    """
    Verify that landing cannot be confirmed while the drone is armed.

    Even if the drone reports ON_GROUND, landing verification must
    fail unless a DISARMED heartbeat is also received.

    """
    drone = DroneController()
    fake_connection = MagicMock()

    fake_connection.target_system = 1
    fake_connection.target_component = 1
    drone.connection = fake_connection

    ground_message = MagicMock()
    ground_message.get_type.return_value = "EXTENDED_SYS_STATE"
    ground_message.get_srcSystem.return_value = 1
    ground_message.get_srcComponent.return_value = 1
    ground_message.landed_state = (
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    )

    armed_message = MagicMock()
    armed_message.get_type.return_value = "HEARTBEAT"
    armed_message.get_srcSystem.return_value = 1
    armed_message.get_srcComponent.return_value = 1
    armed_message.base_mode = (
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )

    messages = [ground_message, armed_message]
    message_index = [0]

    def fake_recv_match(**kwargs):
        message = messages[message_index[0] % 2]
        message_index[0] += 1
        return message

    fake_connection.recv_match.side_effect = fake_recv_match

    try:
        drone.wait_until_landed(timeout=0.1)
    except TimeoutError:
        print("PASS: Armed drone was not considered safely landed.")
    else:
        raise AssertionError(
            "Landing was incorrectly confirmed while armed."
        )


def main() -> None:
    test_missing_telemetry()
    test_stale_landing_telemetry()
    test_armed_drone_on_ground()
    drone = DroneController()

    drone.connect()
    drone.verify_sitl()
    drone.check_preflight_state()

    try:
        drone.wait_until_landed(timeout=0)

    except ValueError:
        print("PASS: Invalid timeout rejected.")
    else:
        raise AssertionError("Invalid timeout was not rejected.")

    try:
        drone.wait_until_landed(timeout=-5)
    except ValueError:
        print("PASS: Negative timeout rejected.")
    else:
        raise AssertionError("Negative timeout was not rejected.")

    result = drone.wait_until_landed(timeout=15)

    if result is not True:
        raise AssertionError("Landing verification returned an unexpected result.")

    print("PASS: ON_GROUND and DISARMED confirmed.")


if __name__ == "__main__":
    main()