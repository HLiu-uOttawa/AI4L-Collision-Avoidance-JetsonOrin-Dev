#!/usr/bin/env python3
"""
nrc_ardu_copter.py

A simple “wrapper” library for ArduCopter via pymavlink, all in one file,
extended with a socket server. When a JSON message {"collision": 1} arrives,
the aircraft will climb to 75 m AGL immediately, and from that point onward
all waypoints will be flown at 75 m instead of their original altitude.

Features:
  • arm_and_takeoff(target_alt_m)
  • land()
  • rtl()
  • goto_waypoint(lat, lon, alt_agl, speed_m_s, threshold_m=5.0)
      – climbs/descends to target altitude first, then cruises horizontally
      – yaw always points at the next waypoint (with a small deadband)
      – no need to reach exact lat/lon; uses a user-defined threshold
      – if a collision interrupt has occurred, all subsequent waypoints use
        75 m AGL instead of their original alt_agl
  • Listens on TCP port 65432 for JSON messages. If {"collision": 1} is received,
    the copter immediately climbs to 75 m AGL, sets an “override altitude” to 75 m,
    and continues the mission at that new altitude.

This version requests HIGH-RATE GLOBAL_POSITION_INT and ATTITUDE streams (10 Hz)
and uses non-blocking reads + a 0.1 s loop so that debug prints remain “live.”
"""

import socket
import json
import threading
import math
import time
from pymavlink import mavutil

HOST = 'localhost'
PORT = 65432
INTERRUPT_ALT = 75.0  # m AGL climb target when collision=1
REDUCED_SPEED = 1.0   # <-- NEW: Reduced horizontal speed (m/s) after collision:2

class NRCArduCopter:
    def __init__(self, connection_str="udp:127.0.0.1:14551"):
        # Connect & wait for heartbeat
        self.master = mavutil.mavlink_connection(connection_str)
        print("[NRC] waiting for heartbeat…")
        

        # add timestamp and all of print out to logging....

        self.master.wait_heartbeat()
        print(f"[NRC] connected (sysid={self.master.target_system})")

        # By default, hold heading on mission
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b"WP_YAW_BEHAVIOR", 0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        )
        time.sleep(0.5)  # allow time for param to update

        # Mask for LOCAL_NED that ignores position/accel but uses vx,vy,vz,yaw
        self.MASK_LOCAL_NED = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE    |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE    |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE    |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE   |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE   |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE   |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        # Placeholder for the most recent position/attitude
        self._last_global = None    # (lat, lon, rel_alt)
        self._last_attitude = None  # (roll, pitch, yaw_rad)

        # Collision-interrupt flags
        self._collision_flag = False
        self._reduced_speed_flag = False  # Initialize reduced speed flag

        self._override_alt = None   # once set to INTERRUPT_ALT, all future waypoints use this

        # Start incoming-message listener thread to keep self._last_global and _last_attitude fresh
        self._stop_reader = False
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # Start socket-server thread
        self._stop_socket = False
        self._socket_thread = threading.Thread(target=self._socket_loop, daemon=True)
        self._socket_thread.start()

        # Once initialized, request HIGH-RATE streams
        self._start_high_rate_streams()

    def _start_high_rate_streams(self):
        """Request GLOBAL_POSITION_INT and ATTITUDE at ~10 Hz."""
        # GLOBAL_POSITION_INT @ 10 Hz (interval=100 000 µs)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            100_000, 0, 0, 0, 0, 0
        )
        # ATTITUDE @ 10 Hz
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            100_000, 0, 0, 0, 0, 0
        )
        time.sleep(0.2)

    def _reader_loop(self):
        """Continuously pull GLOBAL_POSITION_INT and ATTITUDE (non-blocking)."""
        while not self._stop_reader:
            msg = self.master.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=False)
            if msg:
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    self._last_global = (
                        msg.lat / 1e7,
                        msg.lon / 1e7,
                        msg.relative_alt / 1000.0
                    )
                elif msg.get_type() == "ATTITUDE":
                    self._last_attitude = (msg.roll, msg.pitch, msg.yaw)
            else:
                # no new message right now; yield briefly
                time.sleep(0.005)

    def _socket_loop(self):
        """
        Accept multiple client connections.
        Each client is handled in a separate thread.
        """
        def handle_client(conn, addr):
            with conn:
                print(f"[SOCKET] Connected by {addr}")
                while not self._stop_socket:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            break
                        received = json.loads(data.decode())
                        print("[SOCKET] Received:", received)

                        if received.get("collision") == 1:
                            self._collision_flag = True
                            self._override_alt = INTERRUPT_ALT
                            resp = {"message": "collision (altitude override) received", "status": "success"}

                        elif received.get("collision") == 2:
                            self._reduced_speed_flag = True
                            resp = {"message": "collision (speed reduction) received", "status": "success"}
                            print("[SOCKET] Speed reduction triggered. Setting reduced speed flag.")

                        else:
                            resp = {"message": "unrecognized command", "status": "ok"}

                    except json.JSONDecodeError:
                        resp = {"message": "invalid json", "status": "error"}
                    except Exception as e:
                        print(f"[SOCKET] Error: {e}")
                        break

                    conn.sendall(json.dumps(resp).encode())
            print(f"[SOCKET] Client {addr} disconnected.")

        # Server socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            print(f"[SOCKET] Server listening on {HOST}:{PORT}")
            while not self._stop_socket:
                try:
                    s.settimeout(1.0)  # allow for graceful shutdown
                    conn, addr = s.accept()
                    threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
                except socket.timeout:
                    continue

    def stop(self):
        """Stop both reader and socket threads."""
        self._stop_reader = True
        self._reader_thread.join()
        self._stop_socket = True
        # Socket thread will exit once stopped; give it a moment
        self._socket_thread.join(timeout=0.1)

    # ──────────────────────────────────────────────────────────────────────────
    # BASIC HELPERS
    # ──────────────────────────────────────────────────────────────────────────
    def set_guided_mode(self):
        mode_id = self.master.mode_mapping()["GUIDED"]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        time.sleep(0.5)

    def arm_and_takeoff(self, target_alt_m):
        """Arms the motors and takes off to target_alt_m (AGL)."""
        # Record “home” from the very first GLOBAL_POSITION_INT
        while self._last_global is None:
            time.sleep(0.1)
        self._home_lat, self._home_lon, _ = self._last_global

        print("[ARM] arming motors…")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        self.master.motors_armed_wait()
        print("[ARM] motors armed")

        print(f"[TAKEOFF] climbing to {target_alt_m:.1f} m AGL")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0,
            self._home_lat, self._home_lon, target_alt_m,
        )
        # simple blocking: wait until altitude ≥ (target_alt_m - 1)
        while True:
            if self._last_global is None:
                time.sleep(0.1)
                continue
            _, _, alt_now = self._last_global
            if abs(alt_now - target_alt_m) < 1.0:
                print(f"[ALT] reached {alt_now:.1f} m / {target_alt_m:.1f} m")
                break
            print(f"[ALT] {alt_now:.1f} m / {target_alt_m:.1f} m")
            time.sleep(0.2)

    def land(self):
        """Sends a LAND command and waits until disarmed."""
        print("[LAND] initiating landing…")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        # wait until disarmed
        while True:
            if not self.master.motors_armed():
                print("[LAND] motors disarmed")
                break
            time.sleep(0.5)

    def rtl(self):
        """Sends an RTL (Return to Launch) command and waits until disarmed."""
        print("[RTL] returning to launch…")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        # wait until disarmed
        while True:
            if not self.master.motors_armed():
                print("[RTL] completed; motors disarmed")
                break
            time.sleep(0.5)

    def get_position(self):
        """Returns (lat, lon, alt_agl) or None if not yet available."""
        return self._last_global

    def get_yaw(self):
        """Returns current yaw in degrees (0..360), or None if not yet available."""
        if self._last_attitude is None:
            return None
        yaw_rad = self._last_attitude[2]
        return (math.degrees(yaw_rad) + 360.0) % 360.0

    # ──────────────────────────────────────────────────────────────────────────
    # GEODESY HELPERS
    # ──────────────────────────────────────────────────────────────────────────
    def _distance_m(self, lat1, lon1, lat2, lon2):
        """Small-angle approximation → horizontal distance in meters."""
        dn = (lat2 - lat1) * 111_139
        de = (lon2 - lon1) * 111_139 * math.cos(math.radians(lat1))
        return math.hypot(dn, de)

    def _bearing_to(self, lat1, lon1, lat2, lon2):
        """Bearing in degrees [0..360) from (lat1,lon1) toward (lat2,lon2)."""
        dn = (lat2 - lat1)
        de = (lon2 - lon1) * math.cos(math.radians(lat1))
        θ = math.degrees(math.atan2(de, dn)) % 360.0
        return θ

    # ──────────────────────────────────────────────────────────────────────────
    # INTERRUPT CLIMB
    # ──────────────────────────────────────────────────────────────────────────
    def _climb_to_interrupt_alt(self):
        """
        Climb vertically to INTERRUPT_ALT (m AGL) at ±1 m/s,
        keeping yaw at the current heading. Once reached, leave
        self._override_alt = INTERRUPT_ALT so subsequent waypoints
        use 75 m instead of their original altitude.
        """
        if self._last_global is None:
            return
        _, _, alt_now = self._last_global
        yaw_now = self.get_yaw() or 0.0
        yaw_cmd = math.radians(yaw_now)

        print(f"[INTERRUPT] climbing to {INTERRUPT_ALT:.1f} m AGL")
        while True:
            if self._last_global is None:
                time.sleep(0.1)
                continue
            _, _, alt_now = self._last_global
            dz = INTERRUPT_ALT - alt_now
            if abs(dz) <= 1.0:
                print(f"[INTERRUPT COMPLETE] {alt_now:.1f} m")
                break

            vz_cmd = -1.0 if dz > 0 else 1.0  # negative vz = climb
            # Send LOCAL_NED: vx=0, vy=0, vz=vz_cmd, yaw=yaw_cmd
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.MASK_LOCAL_NED,
                0, 0, 0,          # ignore pos
                0, 0, vz_cmd,     # vertical speed
                0, 0, 0,          # ignore accel
                yaw_cmd, 0        # yaw, yaw_rate=0
            )
            time.sleep(0.1)

        # Keep override altitude set (do not clear).  
        # The next goto_waypoint calls will use 75 m instead of the original tgt_alt.
        self._collision_flag = False
        self._reduced_speed_flag = False

    # ──────────────────────────────────────────────────────────────────────────
    # GOTO WAYPOINT WITH YAW-POINTING, DEADZONE & COLLISION-INTERRUPT
    # ──────────────────────────────────────────────────────────────────────────
    def goto_waypoint(self, tgt_lat, tgt_lon, tgt_alt, speed=3.0, threshold=5.0):
        """
        Moves drone to specified waypoint with altitude and speed handling:
        - Climb/descend vertically to target altitude (or INTERRUPT_ALT if collision:1 occurred)
        - Cruise horizontally toward waypoint at normal speed, reduce immediately if collision:2 occurs
        - If collision:1 occurred and override altitude is set, ignore reduced speed
        """
        YAW_DEADBAND = 3.0  # degrees tolerance before changing yaw

        final_target_alt = self._override_alt if self._override_alt else tgt_alt

        print(f"[GOTO] Going to lat={tgt_lat:.6f}, lon={tgt_lon:.6f}, alt={final_target_alt:.1f} m")

        while self._last_global is None or self._last_attitude is None:
            time.sleep(0.1)

        # Phase 1: Vertical alignment
        while True:
            if self._collision_flag:
                self._climb_to_interrupt_alt()
                final_target_alt = self._override_alt

            lat_now, lon_now, alt_now = self._last_global
            dz = final_target_alt - alt_now

            if abs(dz) <= 1.0:
                break

            vz_cmd = -1.0 if dz > 0 else 1.0

            desired_bearing = self._bearing_to(lat_now, lon_now, tgt_lat, tgt_lon)
            yaw_now = self.get_yaw()
            if yaw_now is None:
                yaw_cmd = 0.0
            elif abs((desired_bearing - yaw_now + 180) % 360 - 180) > YAW_DEADBAND:
                yaw_cmd = math.radians(desired_bearing)
            else:
                yaw_cmd = math.radians(yaw_now)

            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.MASK_LOCAL_NED,
                0, 0, 0,
                0, 0, vz_cmd,
                0, 0, 0,
                yaw_cmd, 0
            )

            time.sleep(0.1)

        # Phase 2: Horizontal cruise
        while True:
            if self._collision_flag:
                self._climb_to_interrupt_alt()
                final_target_alt = self._override_alt

            lat_now, lon_now, alt_now = self._last_global
            dist_h = self._distance_m(lat_now, lon_now, tgt_lat, tgt_lon)
            dz = final_target_alt - alt_now
            dist_3d = math.hypot(dist_h, dz)

            if dist_3d < threshold:
                print("[GOTO] Waypoint reached")
                break

            bearing_rad = math.atan2(
                (tgt_lon - lon_now) * math.cos(math.radians(lat_now)),
                tgt_lat - lat_now
            )

            desired_bearing = (math.degrees(bearing_rad) + 360) % 360
            yaw_now = self.get_yaw()
            if yaw_now is None:
                yaw_cmd = 0.0
            elif abs((desired_bearing - yaw_now + 180) % 360 - 180) > YAW_DEADBAND:
                yaw_cmd = math.radians(desired_bearing)
            else:
                yaw_cmd = math.radians(yaw_now)

            # Use reduced speed only if altitude override is not active
            if self._override_alt is None and self._reduced_speed_flag:
                current_speed = REDUCED_SPEED
            else:
                current_speed = speed

            vn = current_speed * math.cos(bearing_rad)
            ve = current_speed * math.sin(bearing_rad)

            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.MASK_LOCAL_NED,
                0, 0, 0,
                vn, ve, 0,
                0, 0, 0,
                yaw_cmd, 0
            )

            print(f"[GOTO] pos=({lat_now:.6f},{lon_now:.6f},{alt_now:.1f})m | vel=({vn:.1f},{ve:.1f})m/s | yaw={math.degrees(yaw_cmd):.1f}°")
            time.sleep(0.1)


    # ──────────────────────────────────────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────────
# Example usage (at bottom of file). Adjust lat/lon/alt as needed.
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    cop = NRCArduCopter("udp:127.0.0.1:14551")

    cop.set_guided_mode()
    cop.arm_and_takeoff(10.0)

    waypoints_forward = [
        (45.654170, -73.719708, 50.0, 3.0),
        (45.654782, -73.719959, 50.0, 3.0),
        (45.655166, -73.720094, 50.0, 3.0),
    ]

    # Fly forward sequence
    for lat, lon, alt_agl, normal_spd in waypoints_forward:
        alt_to_use = INTERRUPT_ALT if cop._override_alt else alt_agl
        cop.goto_waypoint(lat, lon, alt_to_use, speed=normal_spd, threshold=8.0)

    # Fly reverse sequence
    for lat, lon, alt_agl, normal_spd in reversed(waypoints_forward):
        alt_to_use = INTERRUPT_ALT if cop._override_alt else alt_agl
        cop.goto_waypoint(lat, lon, alt_to_use, speed=normal_spd, threshold=8.0)

    cop.rtl()
    cop.land()
    cop.stop()

    print("[NRC] mission complete.")
