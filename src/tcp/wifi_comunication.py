# ---------------------------------------------------------------------------
# WiFi peer link between the two SuperTeam robots (waiter A and chef B).
# One robot runs the hotspot ("host"), the other joins it ("client"). Both use
# a single shared TCP connection to send orders / dish-ready signals and to
# acknowledge the other robot with DONE.
# ---------------------------------------------------------------------------
import socket
import threading
import time
import traceback

DEBUG = True

MODE_HOST = "host"
MODE_CLIENT = "client"

DEFAULT_PORT = 9000
DEFAULT_HOST = "10.42.0.1"

RECV_CHUNK_SIZE = 1024
LISTEN_BACKLOG = 1

CONNECT_TIMEOUT_S = 5.0
READ_TIMEOUT_S = 1.0
ACCEPT_TIMEOUT_S = 1.0

ACK_TIMEOUT_S = 3.0
RECONNECT_DELAY_S = 0.5


class WifiPeer:
    # -----------------------------
    # Initialization and configuration
    # -----------------------------
    def __init__(self, mode, host=DEFAULT_HOST, port=DEFAULT_PORT):
        """
        Two-way WiFi link for the SuperTeam Challenge, over one shared TCP connection. The robot running the hotspot uses mode 'host' (it accepts the connection); the robot joining the hotspot uses mode 'client' (it dials the hotspot's fixed IP). Both robots then send and receive over the same socket, so the host never needs the client's DHCP address.
        ### Parameters:
        - mode: 'host' for the hotspot robot, 'client' for the joining robot.
        - host: Hotspot's IP address (client mode only; ignored for host).
        - port: TCP port to connect to / listen on. Must match on both robots.
        """
        self.mode = mode
        self.host = host
        self.port = port

        self.state_lock = threading.Lock()
        self.received_order = None
        self.dish_ready = False

        self.listen_socket = None
        self.conn = None
        self.running = False
        self.thread = None

        self.send_lock = threading.Lock()
        self.connected_event = threading.Event()
        self.ack_event = threading.Event()
        self.ack_value = False

        # Verb -> handler dispatch table, mirroring the ESP32 function_table.
        self.handlers = {
            "ORDER": self.handle_order,
            "DISH_READY": self.handle_dish_ready,
            "PING": self.handle_ping,
        }

    def __enter__(self):
        """
        Starts the peer link and enables use as a context manager.
        ### Returns:
        - self
        """
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Stops the peer link when exiting the context manager.
        ### Parameters:
        - exc_type: Exception type, if any.
        - exc_val: Exception value, if any.
        - exc_tb: Exception traceback, if any.
        ### Returns:
        - False (exceptions are not suppressed).
        """
        self.stop()
        if exc_type is KeyboardInterrupt:
            if DEBUG:
                print("[INFO] Stopped by Ctrl+C.")
        elif exc_type is not None:
            if DEBUG:
                traceback_string = "".join(
                    traceback.format_exception(exc_type, exc_val, exc_tb)
                )
                print(f"[ERROR] Exception during WiFi peer operation:\n{traceback_string}")
        return False

    def start(self):
        """
        Launches the reader thread that establishes the connection and pumps incoming lines. In host mode it binds the listen socket first; the client dials out from inside the reader loop.
        """
        if self.mode == MODE_HOST:
            self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listen_socket.bind(("0.0.0.0", self.port))
            self.listen_socket.listen(LISTEN_BACKLOG)
            self.listen_socket.settimeout(ACCEPT_TIMEOUT_S)

        self.running = True
        self.thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.thread.start()

        if DEBUG:
            print(f"[WIFI] Started in {self.mode} mode on port {self.port}")

    def stop(self):
        """
        Stops the reader thread and closes both sockets.
        """
        self.running = False
        if self.listen_socket is not None:
            try:
                self.listen_socket.close()
            except Exception:
                pass
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass

    # -----------------------------
    # Connection and receive loop
    # -----------------------------

    def reader_loop(self):
        """
        INTERNAL FUNCTION. DO NOT USE DIRECTLY.
        Establishes the connection, then continuously reads newline-terminated lines and routes each one: DONE/ERR lines complete a pending send, everything else is a command to dispatch and acknowledge. Re-establishes the connection whenever it drops, until stop() is called.
        """
        while self.running:
            if not self.establish():
                continue

            self.connected_event.set()
            recv_buffer = b""
            try:
                while self.running:
                    try:
                        chunk = self.conn.recv(RECV_CHUNK_SIZE)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    recv_buffer += chunk

                    while b"\n" in recv_buffer:
                        line, _, recv_buffer = recv_buffer.partition(b"\n")
                        self.on_line(line.decode("utf-8", errors="ignore").strip())
            except Exception as e:
                if DEBUG:
                    print(f"[ERROR] Reader loop failed: {e}")
            finally:
                self.connected_event.clear()
                try:
                    self.conn.close()
                except Exception:
                    pass
                self.conn = None
                if DEBUG and self.running:
                    print("[WIFI] Connection dropped, re-establishing...")

    def establish(self):
        """
        INTERNAL FUNCTION. DO NOT USE DIRECTLY.
        Gets a live connection: the host accepts one, the client dials the hotspot. Returns False (so the caller retries) on timeout or failure.
        ### Returns:
        - True if self.conn is now connected, False otherwise.
        """
        if self.mode == MODE_HOST:
            try:
                conn, addr = self.listen_socket.accept()
            except socket.timeout:
                return False
            except OSError:
                return False
            if DEBUG:
                print(f"[WIFI] Client connected from {addr[0]}:{addr[1]}")
        else:
            try:
                conn = socket.create_connection(
                    (self.host, self.port), timeout=CONNECT_TIMEOUT_S
                )
            except Exception as e:
                if DEBUG:
                    print(f"[WIFI] Hotspot not reachable yet ({e}), retrying...")
                time.sleep(RECONNECT_DELAY_S)
                return False
            if DEBUG:
                print(f"[WIFI] Connected to hotspot at {self.host}:{self.port}")

        conn.settimeout(READ_TIMEOUT_S)
        self.conn = conn
        return True

    def on_line(self, line):
        """
        INTERNAL FUNCTION. DO NOT USE DIRECTLY.
        Routes one received line. DONE/ERR completes a pending outbound send; any other line is dispatched as an incoming command and answered.
        ### Parameters:
        - line: One stripped line received from the peer.
        """
        if not line:
            return

        upper = line.upper()
        if upper.startswith("DONE"):
            self.ack_value = True
            self.ack_event.set()
        elif upper.startswith("ERR"):
            self.ack_value = False
            self.ack_event.set()
        else:
            if DEBUG:
                print(f"[WIFI] Received: {line}")
            response = self.dispatch(line)
            self.send_raw(response)

    def dispatch(self, message):
        """
        INTERNAL FUNCTION. DO NOT USE DIRECTLY.
        Splits a message into verb and value and returns the matching handler's response. Verb matching is case-insensitive.
        ### Parameters:
        - message: Raw message line, e.g. 'ORDER:0'.
        ### Returns:
        - The response string to send back to the peer.
        """
        verb, _, value = message.partition(":")
        verb = verb.strip().upper()

        handler = self.handlers.get(verb)
        if handler is None:
            if DEBUG:
                print(f"[WIFI] Unknown message: {message}")
            return "ERR:UNKNOWN_COMMAND"
        return handler(value)

    # -----------------------------
    # Command handlers (incoming)
    # -----------------------------

    def handle_order(self, value):
        """
        Stores the dish order (SUM value) the other robot handed over.
        ### Parameters:
        - value: SUM value of the order as a string, one of -2..2.
        ### Returns:
        - 'DONE' on success, 'ERR:BAD_PARAM' if the SUM value is malformed.
        """
        try:
            order = int(value)
        except ValueError as e:
            if DEBUG:
                print(f"[ERROR] Bad ORDER value '{value}': {e}")
            return "ERR:BAD_PARAM"

        with self.state_lock:
            self.received_order = order
        return "DONE"

    def handle_dish_ready(self, value):
        """
        Records that the other robot has finished and handed over the dish.
        ### Parameters:
        - value: Unused.
        ### Returns:
        - 'DONE'.
        """
        with self.state_lock:
            self.dish_ready = True
        return "DONE"

    def handle_ping(self, value):
        """
        Handles a liveness check.
        ### Parameters:
        - value: Unused.
        ### Returns:
        - 'DONE'.
        """
        return "DONE"

    def take_order(self):
        """
        Returns the order (SUM value) received from the other robot, then clears it so it is reported only once. Robot B polls this to learn which dish to cook.
        ### Returns:
        - The SUM value as an int, or None if no new order has arrived.
        """
        with self.state_lock:
            order = self.received_order
            self.received_order = None
        return order

    def take_dish_ready(self):
        """
        Reports whether the other robot has signalled the dish is ready, then clears the flag so it fires only once. Robot A polls this to know the dish can be collected.
        ### Returns:
        - True once after a DISH_READY message, False otherwise.
        """
        with self.state_lock:
            ready = self.dish_ready
            self.dish_ready = False
        return ready

    # -----------------------------
    # Sending (outgoing)
    # -----------------------------

    def send_raw(self, line):
        """
        INTERNAL FUNCTION. DO NOT USE DIRECTLY.
        Writes one line to the shared connection under the send lock, so command sends and the reader thread's DONE replies never interleave on the wire.
        ### Parameters:
        - line: Line to send (newline is appended).
        """
        with self.send_lock:
            if self.conn is None:
                return
            try:
                self.conn.sendall((line + "\n").encode("utf-8"))
            except Exception as e:
                if DEBUG:
                    print(f"[ERROR] Send failed: {e}")

    def send_command(self, command, timeout=ACK_TIMEOUT_S):
        """
        Sends a message to the other robot and blocks until it replies DONE. Assumes one outstanding request at a time (the gameplay is half-duplex).
        ### Parameters:
        - command: Message string, e.g. 'ORDER:0'.
        - timeout: Maximum seconds to wait for connection plus acknowledgement.
        ### Returns:
        - True if the other robot acknowledged, False on timeout or ERR reply.
        """
        deadline = time.monotonic() + timeout
        if not self.connected_event.wait(timeout):
            if DEBUG:
                print("[WIFI] No connection, cannot send.")
            return False

        self.ack_event.clear()
        self.ack_value = False
        self.send_raw(command)

        remaining = deadline - time.monotonic()
        if remaining > 0 and self.ack_event.wait(remaining):
            return self.ack_value

        if DEBUG:
            print("[WIFI] Timed out waiting for acknowledgement.")
        return False

    def send_order(self, sum_value):
        """
        Robot A -> Robot B: hands over the order (which dish to cook) at the handoff tile.
        ### Parameters:
        - sum_value: SUM value of the order (-2..2), identifying the dish.
        ### Returns:
        - True if Robot B acknowledged receipt, False on timeout.
        """
        return self.send_command(f"ORDER:{sum_value}")

    def send_dish_ready(self):
        """
        Robot B -> Robot A: signals that the dish is prepared and handed over.
        ### Returns:
        - True if Robot A acknowledged receipt, False on timeout.
        """
        return self.send_command("DISH_READY:")

    def ping(self):
        """
        Checks that the other robot is alive and responding.
        ### Returns:
        - True if acknowledged, False on timeout.
        """
        return self.send_command("PING:")
