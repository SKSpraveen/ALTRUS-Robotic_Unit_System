from __future__ import annotations

import socket
from collections.abc import Iterable

from altrus_cli.ports.interfaces import PayloadReceiver


class UdpReceiver(PayloadReceiver):
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port

    def iter_messages(self) -> Iterable[bytes]:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((self._host, self._port))
            sock.settimeout(1.0)
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                yield data


class TcpReceiver(PayloadReceiver):
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port

    def iter_messages(self) -> Iterable[bytes]:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((self._host, self._port))
            sock.listen(1)
            sock.settimeout(1.0)
            while True:
                try:
                    conn, _ = sock.accept()
                except socket.timeout:
                    continue
                with conn:
                    conn.settimeout(1.0)
                    buffer = b""
                    while True:
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buffer += chunk
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            yield line
