"""
fulldatetime_codec.py
Utility helpers for converting between lora_msgs/msg/FullDateTime and common formats.
"""

from __future__ import annotations
from datetime import datetime, timezone
from typing import Any, Dict, List, Union

try:
    # Preferred in ROS 2
    from rosidl_runtime_py.utilities import get_message  # type: ignore
except Exception:  # pragma: no cover
    # Fallback to user's import path
    from rclpy.type_support import get_message  # type: ignore


Jsonable = Union[None, bool, int, float, str, List["Jsonable"], Dict[str, "Jsonable"]]


def _get_nanos(obj: Any) -> int:
    """Return nanoseconds from an object or dict that may have nanosecond/nanosec field."""
    if hasattr(obj, "nanosecond"):
        return int(getattr(obj, "nanosecond"))
    if hasattr(obj, "nanosec"):
        return int(getattr(obj, "nanosec"))
    if isinstance(obj, dict):
        if "nanosecond" in obj:
            return int(obj["nanosecond"])
        if "nanosec" in obj:
            return int(obj["nanosec"])
    return 0


def _construct_fulldatetime(year: int, month: int, day: int,
                            hour: int, minute: int, second: int, nanos: int):
    """Create a lora_msgs/msg/FullDateTime instance, handling nanosecond vs nanosec field names."""
    FullDateTime = get_message('lora_msgs/msg/FullDateTime')
    msg = FullDateTime()
    # Set common fields (some interfaces may use different defaults for month/day; we assume given ints are valid)
    msg.year = int(year)
    msg.month = int(month)
    msg.day = int(day)
    msg.hour = int(hour)
    msg.minute = int(minute)
    msg.second = int(second)

    # Pick the right nano field
    if hasattr(msg, "nanosecond"):
        msg.nanosecond = int(nanos)
    elif hasattr(msg, "nanosec"):
        msg.nanosec = int(nanos)
    else:
        # If neither exists, ignore silently
        pass
    return msg


class FullDateTimeCodec:
    """Codec/serializer helpers for lora_msgs/msg/FullDateTime."""

    @staticmethod
    def to_datetime_utc(value: Any) -> datetime:
        """
        Convert lora_msgs/msg/FullDateTime or a dict with the same fields
        into a timezone-aware datetime in UTC.
        """
        if value is None:
            return datetime(1970, 1, 1, tzinfo=timezone.utc)

        if isinstance(value, dict):
            year   = int(value.get("year", 0))
            month  = int(value.get("month", 1))
            day    = int(value.get("day", 1))
            hour   = int(value.get("hour", 0))
            minute = int(value.get("minute", 0))
            second = int(value.get("second", 0))
            nanos  = _get_nanos(value)
        else:
            year   = int(getattr(value, "year", 0))
            month  = int(getattr(value, "month", 1))
            day    = int(getattr(value, "day", 1))
            hour   = int(getattr(value, "hour", 0))
            minute = int(getattr(value, "minute", 0))
            second = int(getattr(value, "second", 0))
            nanos  = _get_nanos(value)

        micro = nanos // 1000  # datetime supports only microseconds
        return datetime(year, month, day, hour, minute, second, microsecond=micro, tzinfo=timezone.utc)

    @staticmethod
    def to_iso8601(value: Any) -> str:
        """
        Return ISO-8601 string in UTC with suffix 'Z'. If we have nanoseconds,
        include 9 digits of fractional seconds by stitching the extra 3 digits.
        """
        dt = FullDateTimeCodec.to_datetime_utc(value)
        nanos = _get_nanos(value)

        micro = nanos // 1000
        extra_nanos = nanos % 1000

        # Base ISO with microseconds
        base = dt.isoformat(timespec="microseconds").replace("+00:00", "Z")

        if micro == 0 and extra_nanos == 0:
            return dt.replace(microsecond=0).isoformat(timespec="seconds").replace("+00:00", "Z")

        # Insert 3 extra nano digits to make 9
        date_part, frac_part = base.split("T")
        time_part = frac_part  # e.g., 12:34:56.123456Z
        if "." in time_part:
            hhmmss, fracZ = time_part.split(".", 1)           # "12:34:56", "123456Z"
            frac = fracZ[:-1]                                  # "123456"
            frac_9 = f"{frac}{extra_nanos:03d}"                # -> "123456789"
            return f"{date_part}T{hhmmss}.{frac_9}Z"
        else:
            return base

    @staticmethod
    def to_timestamp(value: Any) -> float:
        """Return seconds since Unix epoch (UTC)."""
        dt = FullDateTimeCodec.to_datetime_utc(value)
        return dt.timestamp()

    @staticmethod
    def to_jsonable(obj: Any) -> Jsonable:
        """
        Convert objects containing FullDateTime into JSON-friendly types.
        Dates are returned as ISO8601 with nanosecond precision.
        """
        # Heuristic: looks like FullDateTime
        if hasattr(obj, "year") and hasattr(obj, "month") and hasattr(obj, "day") and (
            hasattr(obj, "nanosecond") or hasattr(obj, "nanosec")
        ):
            return FullDateTimeCodec.to_iso8601(obj)

        if isinstance(obj, dict):
            return {k: FullDateTimeCodec.to_jsonable(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple, set)):
            return [FullDateTimeCodec.to_jsonable(v) for v in obj]
        if hasattr(obj, "__dict__"):
            return {k: FullDateTimeCodec.to_jsonable(v) for k, v in obj.__dict__.items()}
        return obj

    # Reverse direction -----------------------------------------------

    @staticmethod
    def parse_iso8601(value: Any):
        """
        Parse various inputs (ISO-8601 str, timestamp float/int, FullDateTime, dict)
        into lora_msgs/msg/FullDateTime. Empty/falsey values return zeroed message.
        """
        FullDateTime = get_message('lora_msgs/msg/FullDateTime')

        # Zeroed message on empty
        if not value:
            return _construct_fulldatetime(0, 0, 0, 0, 0, 0, 0)

        # Already correct type
        if isinstance(value, FullDateTime):
            return value

        # Dict with fields
        if isinstance(value, dict):
            year   = int(value.get("year", 0))
            month  = int(value.get("month", 0))
            day    = int(value.get("day", 0))
            hour   = int(value.get("hour", 0))
            minute = int(value.get("minute", 0))
            second = int(value.get("second", 0))
            nanos  = _get_nanos(value)
            return _construct_fulldatetime(year, month, day, hour, minute, second, nanos)

        # Numeric timestamp
        if isinstance(value, (int, float)):
            dt = datetime.fromtimestamp(value, tz=timezone.utc)

        # ISO string
        elif isinstance(value, str):
            s = value.strip()
            if s.endswith("Z"):
                s = s[:-1] + "+00:00"
            try:
                dt = datetime.fromisoformat(s)
            except Exception as e:  # pragma: no cover - passthrough
                raise ValueError(f"Invalid ISO-8601 time: {value}") from e
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            dt = dt.astimezone(timezone.utc)
        else:
            raise TypeError(f"Unsupported time type: {type(value)}")

        nanos = dt.microsecond * 1000
        return _construct_fulldatetime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, nanos)
