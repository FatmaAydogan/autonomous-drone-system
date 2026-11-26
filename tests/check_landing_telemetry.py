
import time
from pymavlink import mavutil


def main():
    print("Connecting to ArduPilot SITL...")

    connection = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14551"
    )

    if not isinstance(connection, mavutil.mavfile):
        raise TypeError("Expected a live MAVLink connection.")

    heartbeat = connection.wait_heartbeat(timeout=15)

    if heartbeat is None:
        raise TimeoutError("No heartbeat received from SITL.")

    connection.target_system = heartbeat.get_srcSystem()
    connection.target_component = heartbeat.get_srcComponent()

    print("Connected!")
    print("Requesting landing-state telemetry...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        500000,
        0, 0, 0, 0, 0
    )

    deadline = time.monotonic() + 10
    message_count = 0

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="EXTENDED_SYS_STATE",
            blocking=True,
            timeout=1
        )

        if message is None:
            continue

        if message.get_srcSystem() != connection.target_system:
            continue

        if message.get_srcComponent() != connection.target_component:
            continue

        state = message.landed_state

        if state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            state_name = "ON_GROUND"

        elif state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
            state_name = "IN_AIR"

        elif state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF:
            state_name = "TAKEOFF"

        elif state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
            state_name = "LANDING"

        else:
            state_name = "UNDEFINED"

        print(f"Landing state: {state_name}")

        message_count += 1

        if message_count >= 5:
            print("Landing telemetry test successful!")
            return

    raise TimeoutError(
        "EXTENDED_SYS_STATE telemetry was not received."
    )


if __name__ == "__main__":
    main()
