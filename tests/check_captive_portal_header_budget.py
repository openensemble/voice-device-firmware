#!/usr/bin/env python3
"""Regression check for the captive-portal HTTP request-header budget."""

from pathlib import Path
import re


MINIMUM_HEADER_BUDGET = 4096


def configured_header_budget(path: Path) -> int:
    match = re.search(
        r"^CONFIG_HTTPD_MAX_REQ_HDR_LEN=(\d+)$",
        path.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    if not match:
        raise AssertionError("CONFIG_HTTPD_MAX_REQ_HDR_LEN must be explicitly configured")
    return int(match.group(1))


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    for config_name in ("sdkconfig", "sdkconfig.defaults"):
        actual = configured_header_budget(root / config_name)
        assert actual >= MINIMUM_HEADER_BUDGET, (
            f"{config_name}: captive portal request-header budget is too small "
            "for mobile captive portal browsers: "
            f"expected >= {MINIMUM_HEADER_BUDGET}, got {actual}"
        )


if __name__ == "__main__":
    main()
