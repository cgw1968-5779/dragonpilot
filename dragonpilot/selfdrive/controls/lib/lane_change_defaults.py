"""Owner's fixed lane change settings (2026-11-02), no settings toggles:
- auto lane change 0.5 s after the blinker (dp_lat_lca_auto_sec was a toggle, default off)
- blocked while BSM sees a car on that side (DesireHelper already does this whenever the car reports BSM;
  Altis and Cross both have enableBsm)
- blocked when there is no normal lane on that side: Road Edge Detection always on (dp_lat_road_edge_detection was a
  toggle, default off). The lane change speed minimum stays the dp_lat_lca_speed setting.
"""
AUTO_LANE_CHANGE_SEC = 0.5
ROAD_EDGE_DETECTION = True
