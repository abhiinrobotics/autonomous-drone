"""Hardware abstraction layer.

`interfaces` defines what mission code may depend on; `sitl` and `hw` provide
the two implementations. Mission and phase code imports interfaces only, never
a concrete driver — that is what lets the same mission run in SITL and on the
Raspberry Pi.
"""
