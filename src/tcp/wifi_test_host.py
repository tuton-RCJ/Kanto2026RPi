# ---------------------------------------------------------------------------
# WiFi test: HOST side. Run this on the hotspot robot.
# Receives orders from the client, prints them, and replies with DISH_READY
# so both directions of the link are exercised. Ctrl+C to stop.
# ---------------------------------------------------------------------------
import time
from wifi_comunication import WifiPeer

with WifiPeer("host") as peer:
    print("[HOST] Listening. Waiting for the client to connect and send orders...")
    while True:
        order = peer.take_order()
        if order is not None:
            print(f"[HOST] Received order: {order}")
            time.sleep(1)
            print("[HOST] Replying with DISH_READY...")
            acked = peer.send_dish_ready()
            print(f"[HOST] Client acknowledged: {acked}")
        time.sleep(0.1)
