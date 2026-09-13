"""Persistent Session API client for the mock-only YAM operator simulator."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from queue import Empty, Queue
import threading
import time
from typing import Any, Protocol

import websockets


class SimulatorBridge(Protocol):
    def api_connection_changed(self, connected: bool, error: str | None = None) -> None: ...
    def api_error_received(self, error: str) -> None: ...
    def api_prepare_session(self, payload: dict[str, Any]) -> dict[str, Any] | None: ...
    def api_handle_joint_command(self, payload: dict[str, Any]) -> list[dict[str, Any]]: ...
    def api_handle_joint_trajectory(self, payload: dict[str, Any]) -> list[dict[str, Any]]: ...
    def api_handle_stop(self, payload: dict[str, Any]) -> None: ...
    def api_heartbeat_payload(self) -> dict[str, Any] | None: ...
    def api_next_observation_payload(self) -> dict[str, Any] | None: ...
    def api_station_status_payload(self) -> dict[str, Any]: ...


class SessionApiSimClient:
    """Register the simulator as a Jetson and relay only validated messages."""

    def __init__(
        self,
        bridge: SimulatorBridge,
        websocket_url: str,
        jetson_id: str,
        token_file: Path,
    ) -> None:
        self.bridge = bridge
        self.websocket_url = websocket_url
        self.jetson_id = jetson_id
        self.token_file = token_file
        self.outbound: Queue[dict[str, Any]] = Queue()
        self.stop_event = threading.Event()
        self.reconnect_grace_s = 0.0
        self.disconnected_at = None
        self.pending_outbound = None
        self.thread = threading.Thread(target=self._thread_main, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=3.0)

    def request_ready(self) -> None:
        self.send({"schema_version": 1, "type": "ready"})

    def send(self, payload: dict[str, Any]) -> None:
        self.outbound.put(dict(payload))

    def _thread_main(self) -> None:
        asyncio.run(self._reconnect_loop())

    async def _reconnect_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                await self._connect_once()
            except Exception as exc:
                lease = await asyncio.to_thread(self.bridge.api_heartbeat_payload)
                if self.reconnect_grace_s and lease and hasattr(self.bridge, 'api_transport_paused'):
                    if self.disconnected_at is None:
                        self.disconnected_at = time.monotonic()
                    if time.monotonic() - self.disconnected_at < self.reconnect_grace_s:
                        await asyncio.to_thread(self.bridge.api_transport_paused, str(exc))
                    else:
                        await asyncio.to_thread(self.bridge.api_connection_changed, False, 'reconnect_grace_expired')
                        self.disconnected_at = None
                else:
                    await asyncio.to_thread(self.bridge.api_connection_changed, False, str(exc))
            if not self.stop_event.is_set():
                await asyncio.sleep(1.0)

    async def _connect_once(self) -> None:
        token = self.token_file.read_text(encoding="utf-8").strip()
        if not token:
            raise RuntimeError("Jetson Session API token file is empty")
        headers = {"Authorization": f"Bearer {token}"}
        async with websockets.connect(
            self.websocket_url,
            additional_headers=headers,
            open_timeout=10,
            ping_interval=2,
            ping_timeout=6,
        ) as socket:
            resume = await asyncio.to_thread(self.bridge.api_heartbeat_payload)
            await socket.send(json.dumps({
                "schema_version": 1,
                "type": "register",
                "jetson_id": self.jetson_id,
                "reconnect": hasattr(self.bridge, "api_transport_paused"),
                "resume": resume,
            }))
            registered = json.loads(await asyncio.wait_for(socket.recv(), timeout=5.0))
            if registered.get("type") != "registered" or registered.get("jetson_id") != self.jetson_id:
                raise RuntimeError(f"Session API registration failed: {registered}")
            self.reconnect_grace_s = min(30.0, float(registered.get('reconnect_grace_s', 0)))
            if resume and not registered.get('resumed'):
                await asyncio.to_thread(self.bridge.api_connection_changed, False, 'session_not_resumable')
                self.pending_outbound = None
                while not self.outbound.empty():
                    self.outbound.get_nowait()
            if resume and registered.get('resumed') and hasattr(self.bridge, 'api_transport_resume'):
                await asyncio.to_thread(self.bridge.api_transport_resume)
            self.disconnected_at = None
            await asyncio.to_thread(self.bridge.api_connection_changed, True)
            next_heartbeat = asyncio.get_running_loop().time() + 1.0
            next_observation = asyncio.get_running_loop().time() + 0.5
            next_station_status = asyncio.get_running_loop().time()
            while not self.stop_event.is_set():
                while True:
                    if self.pending_outbound is None:
                        try:
                            self.pending_outbound = self.outbound.get_nowait()
                        except Empty:
                            break
                    await socket.send(json.dumps(self.pending_outbound, separators=(",", ":")))
                    self.pending_outbound = None
                now = asyncio.get_running_loop().time()
                if now >= next_heartbeat:
                    heartbeat = await asyncio.to_thread(self.bridge.api_heartbeat_payload)
                    if heartbeat is not None:
                        await socket.send(json.dumps(heartbeat, separators=(",", ":")))
                    next_heartbeat = now + 1.0
                if now >= next_observation:
                    observation = await asyncio.to_thread(self.bridge.api_next_observation_payload)
                    if observation is not None:
                        await socket.send(json.dumps(observation, separators=(",", ":")))
                    next_observation = now + 0.5
                if now >= next_station_status:
                    await socket.send(json.dumps(
                        await asyncio.to_thread(self.bridge.api_station_status_payload), separators=(",", ":")
                    ))
                    next_station_status = now + 0.5
                try:
                    raw = await asyncio.wait_for(socket.recv(), timeout=0.05)
                except asyncio.TimeoutError:
                    continue
                await self._handle_message(json.loads(raw))

    async def _handle_message(self, payload: dict[str, Any]) -> None:
        message_type = payload.get("type")
        if message_type == "prepare_session":
            observation = await asyncio.to_thread(self.bridge.api_prepare_session, payload)
            if observation is not None:
                self.send(observation)
        elif message_type == "joint_command":
            for response in await asyncio.to_thread(self.bridge.api_handle_joint_command, payload):
                self.send(response)
        elif message_type == "joint_trajectory":
            for response in await asyncio.to_thread(self.bridge.api_handle_joint_trajectory, payload):
                self.send(response)
        elif message_type == "stop_session":
            await asyncio.to_thread(self.bridge.api_handle_stop, payload)
        elif "error" in payload:
            await asyncio.to_thread(self.bridge.api_error_received, json.dumps(payload["error"]))
            error = payload["error"]
            if (isinstance(error, dict) and error.get("code") == "not_found"
                    and error.get("message") == f"Jetson {self.jetson_id!r} is not connected"):
                # Close this stale transport and run the authenticated registration
                # handshake again. Session errors must not trigger this path.
                raise ConnectionError("server lost robot registration; reconnecting")
        else:
            raise RuntimeError(f"unsupported Session API message type: {message_type}")
