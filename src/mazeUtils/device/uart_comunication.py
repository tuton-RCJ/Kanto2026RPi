import glob
import serial
import threading
import time
import traceback
import platform  # 追加

DEBUG = True

SERIAL_PORT = "/dev/ttyACM0" 
BAUD_RATE = 115200

# Linux用のパス設定
SERIAL_BY_ID_GLOB = "/dev/serial/by-id/usb-Espressif*"

RECV_CHUNK_SIZE = 64
READ_TIMEOUT_S = 0.2
ACK_TIMEOUT_S = 1.0
SEND_RETRIES = 3
RECONNECT_DELAY_S = 0.5

class WirelessPeer:
    def __init__(self, port=None, baudrate=BAUD_RATE):
        self.fallback_port = port or SERIAL_PORT
        self.port = self.fallback_port
        self.baudrate = baudrate
        self.serial = None

        self.state_lock = threading.Lock()
        self.received_order = None
        self.dish_ready = False

        self.send_lock = threading.Lock()
        self.connected_event = threading.Event()
        self.ack_event = threading.Event()
        self.ack_value = False

        self.running = False
        self.thread = None

        self.handlers = {
            "ORDER": self.handle_order,
            "DISH_READY": self.handle_dish_ready,
            "PING": self.handle_ping,
        }

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False

    def start(self):
        self.running = True
        
        self.thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.thread.start()

        if DEBUG:
            print(f"[UART] Started on {self.port} @ {self.baudrate}")

    def stop(self):
        self.running = False
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass

    def find_port(self):
        if platform.system() != "Windows":
            paths = glob.glob(SERIAL_BY_ID_GLOB)
            if paths:
                if DEBUG:
                    print(f"[UART] Found wireless module at {paths[0]}")
                return paths[0]

        if DEBUG:
            print(f"[UART] Using port: {self.fallback_port}")
        return self.fallback_port

    def ensure_connected(self):
        if self.serial is not None:
            return True
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=READ_TIMEOUT_S)
            self.serial.reset_input_buffer()
        except Exception as e:
            if DEBUG:
                print(f"[UART] Module not available yet ({e}), retrying...")
            self.serial = None
            time.sleep(RECONNECT_DELAY_S)
            return False

        self.connected_event.set()
        if DEBUG:
            print(f"[UART] Serial port {self.port} opened successfully.")
        return True

    def reader_loop(self):
        while self.running:
            if not self.ensure_connected():
                continue

            recv_buffer = b""
            try:
                while self.running:
                    chunk = self.serial.read(1)
                    if chunk:
                        chunk += self.serial.read(self.serial.in_waiting)
                        recv_buffer += chunk
                        while b"\n" in recv_buffer:
                            line, _, recv_buffer = recv_buffer.partition(b"\n")
                            self.on_line(line.decode("utf-8", errors="ignore").strip())
            except Exception as e:
                if DEBUG:
                    print(f"[ERROR] Reader loop failed: {e}")
                self.connected_event.clear()
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None

    def on_line(self, line):
        if not line: return
        upper = line.upper()
        if upper.startswith("DONE"):
            self.ack_value = True
            self.ack_event.set()
        elif upper.startswith("ERR"):
            self.ack_value = False
            self.ack_event.set()
        else:
            if DEBUG:
                print(f"[UART] Received: {line}")
            response = self.dispatch(line)
            self.send_raw(response)

    def dispatch(self, message):
        verb, _, value = message.partition(":")
        verb = verb.strip().upper()
        handler = self.handlers.get(verb)
        if handler is None:
            return "ERR:UNKNOWN_COMMAND"
        return handler(value)

    def handle_order(self, value):
        try:
            order = int(value)
            with self.state_lock: self.received_order = order
            return "DONE"
        except: return "ERR:BAD_PARAM"

    def handle_dish_ready(self, value):
        with self.state_lock: self.dish_ready = True
        return "DONE"

    def handle_ping(self, value):
        return "DONE"

    def take_order(self):
        with self.state_lock:
            order = self.received_order
            self.received_order = None
            return order

    def take_dish_ready(self):
        with self.state_lock:
            ready = self.dish_ready
            self.dish_ready = False
            return ready

    def send_raw(self, line):
        with self.send_lock:
            if self.serial is None: return
            try:
                self.serial.write((line + "\n").encode("utf-8"))
                self.serial.flush()
            except Exception as e:
                if DEBUG: print(f"[ERROR] Send failed: {e}")

    def send_command(self, command, timeout=ACK_TIMEOUT_S):
        if not self.connected_event.wait(timeout): return False
        for attempt in range(SEND_RETRIES):
            self.ack_event.clear()
            self.ack_value = False
            self.send_raw(command)
            if self.ack_event.wait(timeout): return self.ack_value
        return False

    def send_order(self, sum_value): return self.send_command(f"ORDER:{sum_value}")
    def send_dish_ready(self): return self.send_command("DISH_READY:")
    def ping(self): return self.send_command("PING:")