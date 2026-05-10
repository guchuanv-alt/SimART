#!/usr/bin/env python3
import argparse
import json
import math
import os
import select
import sys
import time


def emit(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


class OrbitCameraState:
    def __init__(self):
        self.focus = [0.0, 0.0, -5.0]
        self.yaw_deg = 35.0
        self.pitch_deg = -20.0
        self.distance = 45.0
        self.follow_vehicle = True

    def clamp(self):
        self.pitch_deg = max(-89.0, min(89.0, self.pitch_deg))
        self.distance = max(3.0, min(500.0, self.distance))

    def apply_drag(self, dx, dy, buttons, shift=False):
        if buttons & 2 or shift:
            self.pan(dx, dy)
        else:
            self.yaw_deg += dx * 0.22
            self.pitch_deg += dy * 0.18
            self.clamp()

    def pan(self, dx, dy):
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        right = [math.sin(yaw - math.pi / 2.0), math.cos(yaw - math.pi / 2.0), 0.0]
        up = [-math.cos(yaw) * math.sin(pitch), math.sin(yaw) * math.sin(pitch), math.cos(pitch)]
        pan_scale = max(0.02 * self.distance, 0.05)
        for i in range(3):
            self.focus[i] -= right[i] * dx * pan_scale
            self.focus[i] += up[i] * dy * pan_scale

    def zoom(self, wheel_delta):
        self.distance *= pow(0.90, wheel_delta / 120.0)
        self.clamp()

    def reset(self):
        self.__init__()

    def update_focus_from_vehicle(self, client, vehicle_name):
        try:
            if vehicle_name:
                pose = client.simGetVehiclePose(vehicle_name=vehicle_name)
            else:
                pose = client.simGetVehiclePose()
            self.focus = [pose.position.x_val, pose.position.y_val, pose.position.z_val - 3.0]
        except Exception:
            pass

    def camera_pose(self, airsim):
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        dir_x = math.cos(pitch) * math.cos(yaw)
        dir_y = math.cos(pitch) * math.sin(yaw)
        dir_z = math.sin(pitch)
        cam = [
            self.focus[0] - dir_x * self.distance,
            self.focus[1] - dir_y * self.distance,
            self.focus[2] - dir_z * self.distance,
        ]
        look = [self.focus[0] - cam[0], self.focus[1] - cam[1], self.focus[2] - cam[2]]
        horiz = math.sqrt(look[0] * look[0] + look[1] * look[1])
        cam_pitch = math.atan2(look[2], max(1e-6, horiz))
        cam_yaw = math.atan2(look[1], look[0])
        quat = airsim.to_quaternion(cam_pitch, 0.0, cam_yaw)
        return airsim.Pose(airsim.Vector3r(cam[0], cam[1], cam[2]), quat)


class Bridge:
    def __init__(self, args):
        self.args = args
        self.output = args.output
        self.state = OrbitCameraState()
        self.state.follow_vehicle = bool(int(args.follow))
        self.connected = False
        self.last_error = ""
        self.client = None
        self.airsim = None
        self.last_frame_ts = 0.0
        self.frame_period = 1.0 / max(1.0, args.fps)
        self.connect()

    def connect(self):
        try:
            import airsim  # type: ignore
            self.airsim = airsim
            self.client = airsim.VehicleClient(ip=self.args.host, port=self.args.port)
            self.client.confirmConnection()
            self.connected = True
            emit({"type": "status", "message": f"Connected to AirSim at {self.args.host}:{self.args.port}"})
        except Exception as exc:
            self.connected = False
            self.last_error = str(exc)
            emit({"type": "error", "message": f"AirSim connection failed: {exc}"})

    def process_command(self, line):
        try:
            data = json.loads(line)
        except Exception:
            return True
        cmd = data.get("cmd", "")
        if cmd == "quit":
            return False
        if cmd == "drag":
            self.state.apply_drag(float(data.get("dx", 0.0)), float(data.get("dy", 0.0)), int(data.get("buttons", 0)), bool(data.get("shift", False)))
        elif cmd == "zoom":
            self.state.zoom(float(data.get("delta", 0.0)))
        elif cmd == "reset_camera":
            self.state.reset()
        elif cmd == "pan":
            self.state.pan(float(data.get("dx", 0.0)), float(data.get("dy", 0.0)))
        elif cmd == "set_config":
            self.args.host = data.get("host", self.args.host)
            self.args.port = int(data.get("port", self.args.port))
            self.args.camera = data.get("camera", self.args.camera)
            self.args.vehicle = data.get("vehicle", self.args.vehicle)
            self.args.follow = 1 if data.get("follow", self.args.follow) else 0
            self.state.follow_vehicle = bool(self.args.follow)
            self.args.fps = float(data.get("fps", self.args.fps))
            self.frame_period = 1.0 / max(1.0, self.args.fps)
            self.output = data.get("output", self.output)
            self.connect()
        return True

    def maybe_render(self):
        now = time.time()
        if now - self.last_frame_ts < self.frame_period:
            return
        self.last_frame_ts = now
        if not self.connected or self.client is None or self.airsim is None:
            emit({"type": "error", "message": self.last_error or "AirSim not connected"})
            return
        try:
            if self.state.follow_vehicle:
                self.state.update_focus_from_vehicle(self.client, self.args.vehicle)
            pose = self.state.camera_pose(self.airsim)
            self.client.simSetCameraPose(self.args.camera, pose, vehicle_name=self.args.vehicle or "")
            png = self.client.simGetImage(self.args.camera, self.airsim.ImageType.Scene, vehicle_name=self.args.vehicle or "")
            if png is None:
                emit({"type": "error", "message": "simGetImage returned no data"})
                return
            data = png if isinstance(png, (bytes, bytearray)) else png.encode("latin1")
            tmp = self.output + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, self.output)
            emit({
                "type": "frame",
                "path": self.output,
                "timestamp_ms": int(now * 1000),
                "connected": True,
                "yaw_deg": self.state.yaw_deg,
                "pitch_deg": self.state.pitch_deg,
                "distance": self.state.distance,
                "message": f"Live from {self.args.host}:{self.args.port} · drag to orbit"
            })
        except Exception as exc:
            self.connected = False
            self.last_error = str(exc)
            emit({"type": "error", "message": f"AirSim frame fetch failed: {exc}"})
            time.sleep(0.5)
            self.connect()

    def run(self):
        keep_running = True
        while keep_running:
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if ready:
                    line = sys.stdin.readline()
                    if line == "":
                        break
                    keep_running = self.process_command(line.strip())
            except Exception:
                pass
            self.maybe_render()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--camera", default="front_center")
    parser.add_argument("--vehicle", default="")
    parser.add_argument("--follow", default="1")
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    bridge = Bridge(args)
    bridge.run()


if __name__ == "__main__":
    main()
