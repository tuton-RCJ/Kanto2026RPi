import time
import threading
from uart_comunication import WirelessPeer

TEST_PORT = "/dev/*****"  

def background_monitor(peer):
    """
    Background thread to monitor incoming data (Order or DishReady) from the peer.
    """
    while peer.running:
        order = peer.take_order()
        if order is not None:
            print(f"\n[Notification] Order received. Amount: {order} JPY\nSelect action: ", end="", flush=True)
            
        if peer.take_dish_ready():
            print(f"\n[Notification] 'Dish Ready' status received.\nSelect action: ", end="", flush=True)
            
        time.sleep(0.5)

def main():
    print("=== ESP-NOW Wireless Serial Communication Test Program ===")
    # Create instance (specifying port and baud rate)
    # Assumes baudrate is adjusted to 115200 in uart_comunication.py
    peer = WirelessPeer(port=TEST_PORT, baudrate=115200)

    # Using the 'with' statement automatically calls start() and stop()
    with peer:
        print("Connecting to XIAO ESP32-C3...")
        
        # Wait for connection to be established
        if not peer.connected_event.wait(5.0):
            print("Connection failed. Please verify the port number and cable connection.")
            return
            
        print("Connected successfully.")

        # Start the background monitoring thread
        monitor_thread = threading.Thread(target=background_monitor, args=(peer,), daemon=True)
        monitor_thread.start()

        time.sleep(1) # Short delay to settle the display

        # Interactive Menu
        while True:
            print("\n--- Menu ---")
            print("1: Send PING (Keep-alive check)")
            print("2: Send ORDER (e.g., 1500)")
            print("3: Send DISH_READY")
            print("Q: Quit")
            choice = input("Select action: ").strip().upper()

            if choice == "1":
                print("Sending PING...")
                success = peer.ping()
                print(f"-> Result: {'Success (DONE)' if success else 'Failed (Timeout/Error)'}")

            elif choice == "2":
                val = input("Enter order amount (e.g., 1000): ")
                try:
                    sum_value = int(val)
                    print(f"Sending ORDER: {sum_value}...")
                    success = peer.send_order(sum_value)
                    print(f"-> Result: {'Success (DONE)' if success else 'Failed (Timeout/Error)'}")
                except ValueError:
                    print("Please enter a valid numeric value.")

            elif choice == "3":
                print("Sending DISH_READY...")
                success = peer.send_dish_ready()
                print(f"-> Result: {'Success (DONE)' if success else 'Failed (Timeout/Error)'}")

            elif choice == "Q":
                print("Exiting...")
                break

            else:
                print("Invalid input. Please try again.")

if __name__ == "__main__":
    main()