"""Transport-free Pixhawk values and decoding; runtime is composed by the legacy subscriber.

Import leaf modules directly. This package must not initialize Qt, MAVLink links,
or device resources when a caller needs only data or coordinate conversions.
"""
