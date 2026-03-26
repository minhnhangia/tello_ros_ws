#!/bin/bash

# 1. Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "[ERROR] Please run this script with sudo."
  exit 1
fi

echo "[SYSTEM] Initializing Tello Swarm Routing Matrix..."

# 2. Define the swarm matrix (Name IP TelemetryPort VideoPort)
# We map Tello's native ports (8890/11111) to your ROS2 config ports
DRONES=(
    "tello7  192.168.0.107 8007 11107"
    "tello4  192.168.0.104 8004 11104"
    "tello17 192.168.0.117 8017 11117"
    "tello1  192.168.0.101 8001 11101"
    "tello20 192.168.0.120 8020 11120"
    "tello21 192.168.0.121 8021 11121"
)

# 3. Apply the rules
for drone in "${DRONES[@]}"; do
    # Read the array string into variables
    read -r NAME IP DATA_PORT VIDEO_PORT <<< "$drone"
    
    echo "[CONFIG] Applying routing for $NAME ($IP)..."
    
    # Telemetry (8890 -> ROS2 data_port)
    # Note: We use -I (Insert) instead of -A (Append) to ensure these rules hit first
    iptables -t nat -I PREROUTING -s "$IP" -p udp --dport 8890 -j REDIRECT --to-ports "$DATA_PORT"
    
    # Video (11111 -> ROS2 video_port)
    iptables -t nat -I PREROUTING -s "$IP" -p udp --dport 11111 -j REDIRECT --to-ports "$VIDEO_PORT"
done

echo "[SUCCESS] Active memory iptables updated."

# 4. Make the rules persistent across reboots
echo "[SYSTEM] Checking for persistence packages..."

# Check if iptables-persistent is installed; if not, install it silently
if ! dpkg -l | grep -q iptables-persistent; then
    echo "[SYSTEM] Installing iptables-persistent (DEBIAN_FRONTEND=noninteractive)..."
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq iptables-persistent netfilter-persistent
fi

# Save the current active rules to the OS
echo "[SYSTEM] Saving rules to disk for reboot persistence..."
netfilter-persistent save

echo "[SUCCESS] Swarm network routing is locked in and persistent!"
