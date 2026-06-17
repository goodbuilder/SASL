
from __future__ import annotations

import re
from typing import Optional, Union

from .networkclient import NetworkClient

_DELIM = '\x00'  # hard-fixed delimiter (NUL) for read_until

def _parse_last_int(s: str) -> Optional[int]:
    m = re.findall(r'[-+]?\d+', s or '')
    if not m:
        return None
    try:
        return int(m[-1])
    except Exception:
        return None

def _parse_float_any(s: str) -> Optional[float]:
    # Accept 'OK 12.34', '12,34', '1.23E+01', '1234mm'
    t = (s or '').strip()
    if ',' in t and '.' not in t:
        t = t.replace(',', '.')
    t = re.sub(r'(?<=\d),(?=\d{3}\b)', '', t)  # thousand separators
    m = re.findall(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', t)
    if not m:
        return None
    try:
        return float(m[-1])
    except Exception:
        return None

class MiscFunction:
    def __init__(self, host: str, port: str):
        # Use NetworkClient exactly as provided: write() appends NUL, read_until(delim, timeout) blocks until delim.
        self.net = NetworkClient(host, port)

    def send_cmd(self, message: str, timeout_sec: float = 10.0, debug_print: bool = True) -> Optional[str]:
        """Send a camera command and read until NUL (\x00).
        Returns decoded text (NUL stripped) or None on error/timeout.
        """
        # write (NetworkClient.write will append NUL automatically, matching the C++ behavior)
        self.net.write(message)
        # read exactly until NUL delimiter
        data: Union[int, bytes, str] = self.net.read_until(_DELIM, timeout_sec)

        if debug_print:
            print(f"Command:{message}", flush=True)
            print("Answer:", end="", flush=True)  # newline handling below

        if isinstance(data, int) and data == -1:
            if debug_print:
                print()  # end line
            return None

        # normalize to str and strip NULs for readability
        if isinstance(data, bytes):
            text = data.decode('utf-8', errors='replace')
        elif isinstance(data, str):
            text = data
        else:
            text = str(data)
        text = text.replace('\x00', '')

        if debug_print:
            print(f"\n{text}", end="", flush=True)
            print("------------", flush=True)

        return text

    # Backward compatibility
    def send_raw(self, cmd: str, timeout_sec: float = 10.0) -> Optional[str]:
        return self.send_cmd(cmd, timeout_sec=timeout_sec, debug_print=True)

    # ---- Convenience wrappers ----
    def ping(self, cmd: Optional[str] = None, timeout_sec: float = 10.0) -> bool:
        try:
            if cmd:
                ans = self.send_cmd(cmd, timeout_sec=timeout_sec, debug_print=True)
                return bool(ans)
            # If no command is provided, treat a successful connection as alive
            return True if self.net else False
        except Exception:
            return False

    def set_datetime(self, set_cmd_prefix: str, when_str: str, timeout_sec: float = 10.0) -> bool:
        ans = self.send_cmd(f"{set_cmd_prefix} {when_str}", timeout_sec=timeout_sec, debug_print=True)
        return bool(ans)

    def reboot(self, reboot_cmd: str, timeout_sec: float = 10.0) -> bool:
        self.send_cmd(reboot_cmd, timeout_sec=timeout_sec, debug_print=True)
        return True

    # ---- Zoom ----
    def set_zoom_discrete(self, set_cmd: str, index: int, timeout_sec: float = 10.0) -> bool:
        ans = self.send_cmd(f"{set_cmd} {index}", timeout_sec=timeout_sec, debug_print=True)
        return bool(ans)

    def get_zoom_discrete(self, get_cmd: str, timeout_sec: float = 10.0) -> Optional[int]:
        ans = self.send_cmd(get_cmd, timeout_sec=timeout_sec, debug_print=True)
        if ans is None:
            return None
        return _parse_last_int(ans)

    # ---- Relative distance ----
    def get_relative_distance(self, get_cmd: str, timeout_sec: float = 10.0) -> Optional[float]:
        ans = self.send_cmd(get_cmd, timeout_sec=timeout_sec, debug_print=True)
        if ans is None:
            return None
        return _parse_float_any(ans)
