"""Thin local-process binding to the C++ manipulation runtime; no device SDK imports."""

from .client import DeviceClient, DeviceCommandError

__all__ = ["DeviceClient", "DeviceCommandError"]
