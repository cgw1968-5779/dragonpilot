from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lat_lca_speed",
    "type": "spin_button_item",
    "title": lambda: tr("Lane Change Assist At:"),
    "description": lambda: tr("Off = Disable LCA.<br>1 mph = 1.2 km/h."),
    "default": "20",
    "min_val": 0,
    "max_val": 100,
    "step": 5,
    "suffix": lambda: tr("mph"),
    "special_value_text": lambda: tr("Off"),
    "flags": "PERSISTENT",
    "param_type": "INT",
  },
]
