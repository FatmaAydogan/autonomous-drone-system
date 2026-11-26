
from pymavlink import mavutil


# ArduPilot SITL connection settings
CONNECTION_STRING = "udpin:127.0.0.1:14551"


def connect_to_drone():
    print("Connecting to ArduPilot SITL...")

    connection = mavutil.mavlink_connection(
        CONNECTION_STRING
    )

    # The factory can also return log readers; telemetry needs a live connection.
    if not isinstance(connection, mavutil.mavfile):
        raise TypeError("Expected a live MAVLink connection.")

    if connection.wait_heartbeat(timeout=15) is None:
        raise TimeoutError("No heartbeat received from ArduPilot SITL within 15 seconds.")

    print("Connection established!")
    print(f"System ID: {connection.target_system}")
    print(f"Component ID: {connection.target_component}")

    return connection


def read_telemetry(connection: mavutil.mavfile):
    print("Reading drone telemetry...")

    while True:
        message = connection.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=5
        )

        if message is None:
            print("Waiting for position data...")
            continue

        latitude = message.lat / 1e7
        longitude = message.lon / 1e7
        altitude = message.relative_alt / 1000.0

        print(
            f"Latitude: {latitude:.7f} | "
            f"Longitude: {longitude:.7f} | "
            f"Altitude: {altitude:.2f} m"
        )


def main():
    try:
        connection = connect_to_drone()
        read_telemetry(connection)

    except KeyboardInterrupt:
        print("\nTelemetry monitoring stopped.")

    except Exception as error:
        print(f"Error: {error}")


if __name__ == "__main__":
    main()
