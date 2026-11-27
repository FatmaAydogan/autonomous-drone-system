import time

from pymavlink import mavutil


#Converting numerical error codes from ArduPilot into human-readable text
def result_name(result):
   
    entry = mavutil.mavlink.enums["MAV_RESULT"].get(result)
    return entry.name if entry is not None else str(result)


class DroneController:

    def __init__(self, connection_string="udpin:127.0.0.1:14551"):
        self.connection_string = connection_string
        self.connection: mavutil.mavfile | None = None

    def connect(self):
        print("Connecting to ArduPilot...")

        connection = mavutil.mavlink_connection(
            self.connection_string
        )

        # The factory also supports log readers, which are not live connections.
        if not isinstance(connection, mavutil.mavfile):
            raise TypeError("Expected a live MAVLink connection.")

        heartbeat = connection.wait_heartbeat(timeout=15)

        if heartbeat is None:
            raise TimeoutError("No heartbeat received from drone.")

        connection.target_system = heartbeat.get_srcSystem()
        connection.target_component = heartbeat.get_srcComponent()
        self.connection = connection

        print("Drone connected successfully!")
        print(f"System ID: {connection.target_system}")
        print(f"Component ID: {connection.target_component}")



    def verify_sitl(self, timeout=10):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        print("Checking SITL environment...")

        if self.connection_string != "udpin:127.0.0.1:14551":
            raise RuntimeError(
                "Unexpected connection address for SITL test."
            )

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type="SIMSTATE",
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            print("SITL simulation telemetry confirmed.")
            return True

        raise RuntimeError(
            "SIMSTATE not received. SITL could not be verified."
        )



    def check_preflight_state(self, timeout=10):
        import time

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        print("Checking preflight state...")

        deadline = time.monotonic() + timeout

        ground_time = None
        disarmed_time = None

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["EXTENDED_SYS_STATE", "HEARTBEAT"],
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            if message.get_srcComponent() != self.connection.target_component:
                continue

            now = time.monotonic()

            if message.get_type() == "EXTENDED_SYS_STATE":
                if message.landed_state == (
                    mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                ):
                    ground_time = now
                else:
                    ground_time = None

            elif message.get_type() == "HEARTBEAT":
                armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if not armed:
                    disarmed_time = now
                else:
                    disarmed_time = None

            ground_recent = (
                ground_time is not None
                and now - ground_time <= 3
            )

            disarmed_recent = (
                disarmed_time is not None
                and now - disarmed_time <= 3
            )

            if ground_recent and disarmed_recent:
                print("Preflight state confirmed!")
                print("Drone is ON_GROUND and DISARMED.")
                return True

        raise TimeoutError(
            "Preflight state could not be confirmed."
        )




    def get_flight_mode(self, timeout=5):
        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            heartbeat = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if heartbeat is None:
                continue

            if heartbeat.get_srcSystem() != self.connection.target_system:
                continue

            if heartbeat.get_srcComponent() != self.connection.target_component:
                continue

            mode = mavutil.mode_string_v10(heartbeat)
            return mode

        raise TimeoutError("No heartbeat received.")



#read the arm status from the drone itself
    def is_armed(self, timeout=5):
        
        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            heartbeat = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if heartbeat is None:
                continue

            if heartbeat.get_srcSystem() != self.connection.target_system:
                continue

            if heartbeat.get_srcComponent() != self.connection.target_component:
                continue

            return bool(
                heartbeat.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

        raise TimeoutError("No heartbeat received.")
    
    
    def set_flight_mode(self, mode, timeout=10):
        #max 10 sec for mode change confirmation
        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        mode = mode.upper()
        #the ID of the relevant mode for MAVLink 
        mode_mapping = self.connection.mode_mapping()


        if mode_mapping is None:
            raise RuntimeError(
                "Flight modes are unavailable: vehicle type is unknown or unsupported."
            )

        if mode not in mode_mapping:
            raise ValueError(f"Unsupported flight mode: {mode}")

        mode_id = mode_mapping[mode]

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        print(f"Flight mode change requested: {mode}")

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            heartbeat = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if heartbeat is None:
                continue

            if heartbeat.get_srcSystem() != self.connection.target_system:
                continue

            if heartbeat.get_srcComponent() != self.connection.target_component:
                continue

            current_mode = mavutil.mode_string_v10(heartbeat)

            if current_mode == mode:
                print(f"Flight mode confirmed: {current_mode}")
                return True

        raise TimeoutError(
            f"Flight mode change to {mode} was not confirmed."
        )


#activates the engines
    def arm(self, timeout=15):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        print("Requesting drone arming...")

        self.connection.mav.command_long_send(

            self.connection.target_system,# ID of the Drone
            self.connection.target_component,# ID of the MAVLink 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,#ID of the arm/diasrm
            0,  #0 = disarm
            1,  # 1 = Arm
            0, 0, 0, 0, 0, 0
        )

        deadline = time.monotonic() + timeout

        #to find teh reason if it is rejected
        last_status_text = None 


        #control heartbeat messages
        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"],
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            message_type = message.get_type()

            if message_type == "STATUSTEXT":
                print(f"ArduPilot: {message.text}")
                last_status_text = message.text
                continue

            if message.get_srcComponent() != self.connection.target_component:
                continue

            if message_type == "COMMAND_ACK":

                if message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    continue

                if message.result not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS
                ):
                    raise RuntimeError(
                        f"Arming command rejected. Result: {result_name(message.result)}"
                        f" | Last ArduPilot message: {last_status_text}"
                    )


                print("Arming command acknowledged.")
                continue

            if message_type == "HEARTBEAT":

                armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if armed:
                    print("Drone arming confirmed!")
                    return True

        raise TimeoutError(
            "Drone arming was not confirmed within the timeout."
        )


#turn off the engines
    def disarm(self, timeout=15):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        print("Requesting drone disarming...")

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 = Disarm
            0, 0, 0, 0, 0, 0
        )

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["HEARTBEAT", "COMMAND_ACK"],
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            if message.get_srcComponent() != self.connection.target_component:
                continue


            if message.get_type() == "COMMAND_ACK":
                if message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    continue

                if message.result not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS
                ):
                    raise RuntimeError(
                        f"Disarming rejected: {message.result}"
                    )

            elif message.get_type() == "HEARTBEAT":
                armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if not armed:
                    print("Drone disarming confirmed!")
                    return True

        raise TimeoutError("Drone disarming was not confirmed.")


    
    def takeoff(self, altitude, timeout=10):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        if altitude <= 0:
            raise ValueError("Takeoff altitude must be positive.")

        print(f"Requesting takeoff to {altitude} meters...")

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["COMMAND_ACK", "STATUSTEXT"],
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            if message.get_type() == "STATUSTEXT":
                print(f"ArduPilot: {message.text}")
                continue

            if message.get_srcComponent() != self.connection.target_component:
                continue

            if message.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                continue

            if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Takeoff command accepted!")
                return True

            if message.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                print("Takeoff command in progress...")
                continue

            raise RuntimeError(
                f"Takeoff command rejected. Result: {message.result}"
            )

        raise TimeoutError(
            "Takeoff command acknowledgment not received."
        )




    def wait_until_altitude(self, target_altitude, timeout=30):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        if target_altitude <= 0:
            raise ValueError("Target altitude must be positive.")

        print(f"Waiting to reach {target_altitude} meters...")

        deadline = time.monotonic() + timeout

        #%95 percent success level
        required_altitude = target_altitude * 0.95

#print the altitude once per second
        last_print_time = 0.0

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                #Global position ınt -> gps ,ımu
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            relative_altitude = message.relative_alt / 1000.0

            print(f"Current altitude: {relative_altitude:.2f} m")

            if relative_altitude >= required_altitude:
                print("Target altitude reached!")
                return True

        raise TimeoutError(
            f"Drone did not reach {target_altitude} meters."
        )


    #controlled landing
    #if return true ıt means drone goes to land mode
    def land(self, timeout=15):

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        print("Requesting automatic landing...")

        self.set_flight_mode("LAND", timeout=timeout)

        print("LAND mode confirmed.")
        print("ArduPilot is now responsible for landing.")

        return True

    def wait_until_landed(self, timeout: float = 60.0) -> bool:

        if self.connection is None:
            raise RuntimeError("Drone is not connected.")

        if not 0 < timeout < float("inf"):
            raise ValueError("Timeout must be a finite positive number.")

        print("Waiting for drone to land...")

        # Request EXTENDED_SYS_STATE at 2 Hz.
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
            500000,
            0, 0, 0, 0, 0
        )

        #calculates the end of the waiting
        deadline = time.monotonic() + timeout
        freshness_limit = 3.0

        ground_time: float | None = None
        disarmed_time: float | None = None

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()

            message = self.connection.recv_match(

                #EXTENDED_SYS_STATE -> AIR,LAND,GRAOUND,
                #HEARBEAT ->ARM, DISARM
                type=["EXTENDED_SYS_STATE", "HEARTBEAT"],
                blocking=True,
                timeout=min(1.0, max(0.0, remaining))
            )

            if message is None:
                continue

            if message.get_srcSystem() != self.connection.target_system:
                continue

            if message.get_srcComponent() != self.connection.target_component:
                continue

            now = time.monotonic()

            if message.get_type() == "EXTENDED_SYS_STATE":
                if message.landed_state == (
                    mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                ):
                    ground_time = now
                else:
                    ground_time = None

            elif message.get_type() == "HEARTBEAT":
                armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if not armed:
                    disarmed_time = now
                else:
                    disarmed_time = None

            ground_recent = (
                ground_time is not None
                and now - ground_time <= freshness_limit
            )

            disarmed_recent = (
                disarmed_time is not None
                and now - disarmed_time <= freshness_limit
            )

            if ground_recent and disarmed_recent:
                print("Landing confirmed!")
                print("Drone is ON_GROUND and DISARMED.")
                return True

        raise TimeoutError(
            "Landing could not be confirmed with fresh telemetry."
        )


def main():
    drone = DroneController()
    armed = False
    landing_requested = False 

    try:
        #ArduPilot connects to SITL
        drone.connect()
        drone.verify_sitl()

#verify the drone is on the ground and disarmed
        drone.check_preflight_state()

        current_mode = drone.get_flight_mode()
        print(f"Current flight mode: {current_mode}")

       

        # Confirm this is a SITL-only test
        confirmation = input("SITL ONLY - Type TAKEOFF to continue: ")
        if confirmation != "TAKEOFF":
            
            print("Flight test cancelled.")
            return

        drone.set_flight_mode('GUIDED')

        # Arm the drone
        drone.arm()
        armed = True

        drone.takeoff(5)

        drone.wait_until_altitude(
            target_altitude=5,
            timeout=30
        )

        print("Takeoff altitude confirmed.")

        landing_requested = True
        drone.land()

        drone.wait_until_landed(timeout=60)
        armed = False

        print("Autonomous flight test completed!")

    except KeyboardInterrupt:
        print("Interrupted by user.")


    except Exception as error:
        print(f"Flight error: {error}")


    finally:
        if drone.connection is not None:
            try:
                if drone.is_armed():
                    armed = True
                    if not landing_requested:
                        print("Attempting controlled landing...")
                        landing_requested = True
                        drone.land()

                    drone.wait_until_landed(timeout=60)
                    armed = False
                else:
                    armed = False

            except Exception as landing_error:
                print(f"Landing could not be confirmed: {landing_error}")
                print("Manual intervention may be required.")

        if armed or landing_requested:
            print ("Flight sequence ended. Check SITL state.")
        else:
            print("Program finished.")


if __name__ == "__main__":
    main()



