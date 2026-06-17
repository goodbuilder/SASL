# networkclient.py
import socket
import select
import time
from typing import Optional


class NetworkClient:
    def __init__(self, host: str, port: str, debug: bool = False):
        self.host = host
        self.port = int(port)
        self.debug = debug
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self.connect()

    def __bool__(self):
        return self.connected

    # ---- Connection management ----
    def connect(self):
        if self.debug:
            print(f"{self.__class__.__name__}.connect: {self.host} - {self.port}")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.connect((self.host, self.port))
            s.setblocking(True)
            self.sock = s
            self.connected = True
        except OSError as e:
            self.connected = False
            if self.debug:
                print(f"{self.__class__.__name__}.connect: {self.host} - {self.port} - {e}")

    def disconnect(self):
        if self.debug:
            print(f"{self.__class__.__name__}.disconnect: {self.host} - {self.port}")
        if self.sock is not None:
            try:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self.sock.close()
            finally:
                self.sock = None
                self.connected = False

    # ---- I/O helpers ----
    def write(self, buffer):
        if not self.connected or self.sock is None:
            return -1
        try:
            if isinstance(buffer, str):
                # C++ version writes string plus the terminating null byte
                data = buffer.encode('utf-8') + b'\x00'
            else:
                # assume bytes-like
                data = bytes(buffer)
            self.sock.sendall(data)
            return len(data)
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.write: {e}")
            return -1

    def wait(self, timeout):
        """Wait until the socket is readable.
        Returns: 0 if ready, 1 if timed out, -1 on error.
        """
        if not self.connected or self.sock is None:
            return -1
        try:
            r, _, _ = select.select([self.sock], [], [], _to_secs(timeout))
            return 0 if r else 1
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.wait: {e}")
            return -1

    def available(self):
        """Approximate number of bytes immediately readable without blocking."""
        if not self.connected or self.sock is None:
            return -1
        try:
            # Use MSG_PEEK to check how many bytes are available up to a cap.
            self.sock.setblocking(False)
            try:
                data = self.sock.recv(65536, socket.MSG_PEEK)
            except BlockingIOError:
                data = b""
            finally:
                self.sock.setblocking(True)
            return len(data)
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.available: {e}")
            return -1

    def read_some(self, size):
        if not self.connected or self.sock is None:
            return -1
        try:
            n_avail = self.available()
            if n_avail <= 0:
                return 0 if n_avail == 0 else -1
            n = min(n_avail, size)
            data = self.sock.recv(n)
            return data  # return bytes
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.read_some: {e}")
            return -1

    def read_some_wait(self, size):
        if not self.connected or self.sock is None:
            return -1
        try:
            data = self.sock.recv(size)
            return data
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.read_some_wait: {e}")
            return -1

    def read_exactly(self, size, timeout):
        """Read up to 'size' bytes or until timeout. Returns bytes read (may be < size).
        Returns -1 on error.
        """
        if not self.connected or self.sock is None:
            return -1
        end_time = time.monotonic() + _to_secs(timeout)
        chunks = []
        total = 0
        try:
            while total < size:
                remaining = end_time - time.monotonic()
                if remaining <= 0:
                    break  # timeout like boost::asio run_for not finished
                r, _, _ = select.select([self.sock], [], [], remaining)
                if not r:
                    break
                chunk = self.sock.recv(size - total)
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
            return b"".join(chunks)
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.read_exactly: {e}")
            return -1

    def read_until(self, delim: str, timeout):
        """Read until delimiter byte appears or timeout.
        Returns the data including the delimiter, or -1 on error, or b'' on timeout/no data.
        """
        if not self.connected or self.sock is None:
            return -1
        if isinstance(delim, str):
            delim_b = delim.encode('utf-8')
        else:
            delim_b = bytes([delim])
        end_time = time.monotonic() + _to_secs(timeout)
        buf = bytearray()
        try:
            while True:
                remaining = end_time - time.monotonic()
                if remaining <= 0:
                    break
                r, _, _ = select.select([self.sock], [], [], remaining)
                if not r:
                    break
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                if delim_b in chunk or (buf and buf[-1:] == delim_b):
                    break
            return bytes(buf)
        except OSError as e:
            if self.debug:
                print(f"{self.__class__.__name__}.read_until: {e}")
            return -1

    # The original had run_for/run_until for the io_context; here we inline that behavior above.
    def run_for(self, timeout):
        # Kept for API parity; returns True if timed out, False otherwise.
        # We emulate by always returning True only if timeout elapses
        # The higher-level methods already implement timeouts.
        time.sleep(_to_secs(timeout))
        return True

    def run_until(self, timepoint):
        # Not used in main.cpp; included for parity.
        return self.run_for(max(0.0, timepoint - time.monotonic()))


def _to_secs(timeout):
    """Accepts timeout in seconds (float) or a (seconds) like chrono duration."""
    # In this port we expect timeout as seconds (float). If None, use 0.
    if timeout is None:
        return 0.0
    if isinstance(timeout, (int, float)):
        return float(timeout)
    # Fallback
    try:
        return float(timeout)
    except Exception:
        return 0.0