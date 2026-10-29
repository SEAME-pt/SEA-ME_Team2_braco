# Guide: install braco on the PC and the Jetson

Everything runs in Docker, so you only install Docker, `make` and `git`. You run every command from the repo folder.

## PC (Linux)

1. **Install Docker Engine** with Compose 2.20 or newer: [docs.docker.com/engine/install](https://docs.docker.com/engine/install/).
2. **Install the tools:**

   ```bash
   sudo apt-get install -y make git curl python3-venv x11-xserver-utils
   ```

3. **Let your user run Docker without `sudo`:**

   ```bash
   sudo usermod -aG docker $USER
   ```

   Log out and back in.
4. **GPU** (Gazebo and RViz use it). `make` expects an NVIDIA card:
   - NVIDIA: install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
   - Intel / AMD: add `GPU=dri` to every `make` command.
   - No GPU: add `GPU=none` to every `make` command (Gazebo is slow).
5. **Clone the repo:**

   ```bash
   git clone git@github.com:SEAME-pt/SEA-ME_Team2_braco.git
   cd SEA-ME_Team2_braco
   ```

6. **Prepare the machine:** `make setup`
   - It asks `Is this machine the dev PC or the jetson?` → type **`dev`**.
   - Every line it prints must say `[ ok ]`. A `[FAIL]` line says how to fix it.
7. **Start:** `make up`
   - The first time it builds the image: tens of minutes.
8. **Check:** `make net-test` must print `PASS`.
9. **Before your first PR:** `make ci` runs the CI `host` checks locally. All the jobs: [ci.md](ci.md).

## Jetson

1. **Free the arm's serial port** (the Jetson uses it as a console by default):

   ```bash
   sudo systemctl disable --now nvgetty
   ```

   Log out and back in.
2. **Clone the repo:** the same `git clone` and `cd` as on the PC.
3. **Prepare the machine:** `make setup` → type **`jetson`**.
   - Fix the `[warn]` lines too: a desktop running, power mode not MAXN, no swap. They slow the Jetson down.
4. **Get the image**, one of:
   - on the Jetson: `make image` (slow);
   - on the PC: `make jetson-image NANO=user@<jetson-ip>` (faster: builds it on the PC and copies it).
5. **Start:** `make up`
   - It waits until the arm answers. It never moves the arm.
6. **Check:**
   - `make status`: the arm connected, six angles.
   - `make temps`: six temperatures.

## Problems

| Problem | Fix |
|---|---|
| `permission denied` when running `docker` | Log out and back in after step 3 |
| `docker has no NVIDIA runtime` | Install the NVIDIA Container Toolkit, or use `GPU=dri` / `GPU=none` |
| `Run make as your normal user, not root/sudo` | Run `make` without `sudo` |
| `docker compose ... is too old` | Update Docker (Compose 2.20 or newer) |
| Jetson: `/dev/ttyTHS1 not found` | Jetson step 1, then log out and back in |
| Any other `[FAIL]` | The line says how to fix it. More: [TROUBLESHOOTING.md](ROS2/TROUBLESHOOTING.md) |
