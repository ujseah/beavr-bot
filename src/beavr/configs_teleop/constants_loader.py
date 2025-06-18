"""Utility to override module-level constant values from ``configs/constants.yaml``.

Usage
-----
>>> from beavr.configs_teleop.constants_loader import patch
>>> patch("network", "beavr.configs_teleop.constants")

This reads the YAML only once and updates attributes on the given module so
that existing imports keep working.  Keys in the YAML are case–insensitive and
are converted to UPPER_SNAKE_CASE when applied to the target module.
"""
from __future__ import annotations

import os
from importlib import import_module
from pathlib import Path
from typing import Any, Dict

import yaml

# -----------------------------------------------------------------------------
# Load the YAML only once at import time – it is inexpensive (<1 ms for 100 keys)
# -----------------------------------------------------------------------------

_DEFAULT_YAML = Path(__file__).with_suffix(".yaml")
_YAML_PATH = Path(os.getenv("BEAVR_CONSTANTS_YAML", _DEFAULT_YAML))

if _YAML_PATH.exists():
    with _YAML_PATH.open("r", encoding="utf-8") as fh:
        _DATA: Dict[str, Dict[str, Any]] = yaml.safe_load(fh) or {}
else:
    _DATA = {}

# -----------------------------------------------------------------------------
# Public helper
# -----------------------------------------------------------------------------

def patch(section: str, module_name: str) -> None:  # noqa: D401  (simple helper)
    """Override attributes on *module_name* with values from YAML *section*.

    Parameters
    ----------
    section:
        Top-level key in the YAML (e.g. ``network``, ``teleop``, ``robot``).
    module_name:
        Full dotted path to the module that holds constants (already imported
        or importable via :pyfunc:`importlib.import_module`).

    Only keys present in the YAML section are applied; everything else remains
    untouched so existing defaults continue to work.
    """
    if section not in _DATA:
        return  # silently ignore missing section – nothing to patch

    target_mod = import_module(module_name)

    for key, value in _DATA[section].items():
        setattr(target_mod, key.upper(), value) 