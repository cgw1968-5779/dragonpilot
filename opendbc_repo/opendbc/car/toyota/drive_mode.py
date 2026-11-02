# dragonpilot: Toyota drive mode switch (ECO / NORMAL / SPORT) from GEAR_PACKET (0x3BC).
# Ported from sunnypilot tnpb3 / tncr1 (ToyotaDriveMode). Read straight from CAN every frame and sent in
# carState.driveMode, not through a param file: on 2026-10-22 a /data write stall kept the planner on the
# eco limit for minutes after the driver had switched to SPORT.
#
# Corolla Altis Hybrid and Corolla Cross Hybrid (TSS2, not SecOC): SPORT_ON bit 2, ECON_ON bit 40.
# RAV4 / ES / Highlander TSS2 report sport on SPORT_ON_2 instead, so either sport bit counts.

DRIVE_MODE_UNKNOWN = -1
DRIVE_MODE_ECO = 0
DRIVE_MODE_NORMAL = 1
DRIVE_MODE_SPORT = 2


def drive_mode_from_gear_packet(gear_packet) -> int:
  if "ECON_ON" not in gear_packet and "SPORT_ON" not in gear_packet and "SPORT_ON_2" not in gear_packet:
    return DRIVE_MODE_UNKNOWN
  if gear_packet.get("SPORT_ON", 0) or gear_packet.get("SPORT_ON_2", 0):
    return DRIVE_MODE_SPORT
  if gear_packet.get("ECON_ON", 0):
    return DRIVE_MODE_ECO
  return DRIVE_MODE_NORMAL
