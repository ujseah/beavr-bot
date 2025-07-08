"""
Load all constants from ``configs/constants.yaml`` and expose them as
UPPER_SNAKE_CASE symbols so existing `CONST.X` look-ups keep working.
"""

from __future__ import annotations
import os
from importlib import resources
import yaml

# ------------------------------------------------------------------ #
# Locate YAML (allow env-override)
# ------------------------------------------------------------------ #
DEFAULT = resources.files(__package__) / "constants.yaml"
yaml_path = os.getenv("BEAVR_CONSTANTS_YAML", DEFAULT)

with open(yaml_path, "r", encoding="utf-8") as fh:
    _data = yaml.safe_load(fh) or {}

# ------------------------------------------------------------------ #
# Flatten  {section: {key: value}}  →  {KEY: value}
# ------------------------------------------------------------------ #
for section_dict in _data.values():
    for key, value in section_dict.items():
        globals()[key.upper()] = value

# ------------------------------------------------------------------ #
# Backwards-compatibility aliases – remove once all code is migrated
# ------------------------------------------------------------------ #
if "RIGHT_HAND_OCULUS_RECEIVER_PORT" in globals():
    globals()["OCULUS_RECEIVER_PORT"] = globals()["RIGHT_HAND_OCULUS_RECEIVER_PORT"]  # old → new

if "LEFT_HAND_OCULUS_RECEIVER_PORT" in globals():
    globals()["LEFT_HAND_RECEIVER_PORT"] = globals()["LEFT_HAND_OCULUS_RECEIVER_PORT"]  # old → new

__all__ = [k for k in globals() if k.isupper()]