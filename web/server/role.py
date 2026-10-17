"""Which machine is this: the PC (workstation) or the Jetson (in the arm's base)?

Same answer as the docker Makefile gives, so the page and the containers agree:
SEAME_ROLE (set inside the containers), then COMPOSE_PROFILES (environment or
docker/.env). If none of them says, the CPU decides: the Nano is aarch64.
"""

import os
import platform
from pathlib import Path

DOCKER_ENV = Path(__file__).resolve().parents[2] / "docker" / ".env"


def _dotenv(name):
    try:
        for line in DOCKER_ENV.read_text().splitlines():
            key, _, value = line.partition("=")
            if key.strip() == name:
                return value.split("#")[0].strip().strip("\"'")
    except OSError:
        pass
    return ""


def machine_role():
    """Return "pc" or "jetson"."""
    for raw in (os.environ.get("SEAME_ROLE"),
                os.environ.get("COMPOSE_PROFILES"),
                _dotenv("COMPOSE_PROFILES")):
        value = (raw or "").strip().lower()
        if value.startswith("jetson"):
            return "jetson"
        if value.startswith("pc"):
            return "pc"
    return "jetson" if platform.machine().lower() in ("aarch64", "arm64") else "pc"
