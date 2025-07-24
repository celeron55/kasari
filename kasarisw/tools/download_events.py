import socket
import struct
import time
import json
import os
from datetime import datetime
import sys
from parse_events import parse_event

BATCH_FLUSH_SIZE = 4096
BATCH_DATA_SIZE = BATCH_FLUSH_SIZE - 4

def main():
    # Command line argument for IP:port
    if len(sys.argv) > 1:
        host_port = sys.argv[1]
    else:
        host_port = "192.168.1.248:8081"
    host, port_str = host_port.split(':')
    port = int(port_str)

    # Connect with retries for 60 seconds
    sock = None
    for attempt in range(60):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, port))
            sock.settimeout(None)
            print(f"Connected to {host}:{port}")
            break
        except Exception as e:
            print(f"Connection attempt {attempt+1} failed: {e}. Retrying in 1 second...")
            time.sleep(1)
    else:
        print("Failed to connect after 60 attempts.")
        sys.exit(1)

    # Receive data
    received = b''
    last_print = time.time()
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        received += chunk
        now = time.time()
        if now > last_print + 1:
            print(f"Downloaded {len(received)} bytes")
            last_print = now

    sock.close()

    if len(received) % BATCH_FLUSH_SIZE != 0:
        print(f"Warning: Received data length {len(received)} not multiple of {BATCH_FLUSH_SIZE}")

    # Process batches
    batches = []
    for i in range(0, len(received), BATCH_FLUSH_SIZE):
        buf = received[i:i+BATCH_FLUSH_SIZE]
        seq = struct.unpack('<I', buf[0:4])[0]
        if seq != 0xFFFFFFFF:
            batch_data = buf[4:]
            batches.append((seq, batch_data))

    # Sort by seq (assuming no wrap-around)
    batches.sort(key=lambda x: x[0])

    # Concatenate event data
    event_data = b''.join([bd for seq, bd in batches])

    # Parse events
    buffer = event_data
    events = []
    while len(buffer) > 0:
        event, remaining = parse_event(buffer)
        if event:
            events.append(event)
        else:
            if len(remaining) == len(buffer):
                print("Incomplete event at end, skipping remaining bytes")
                break
            buffer = remaining

    print(f"Total events downloaded: {len(events)}")

    # Split into log files
    os.makedirs('log', exist_ok=True)
    timestamp_str = datetime.now().strftime('%Y-%m-%d_%H%M%S')
    n = 1
    current_log = None
    last_ts = None

    for event in events:
        ts = event[1]  # Timestamp is second element
        if last_ts is not None and ts < last_ts - 100000:
            if current_log:
                current_log.close()
            n += 1

        if current_log is None:
            log_path = f"log/kasarisw_{timestamp_str}_{n}.log"
            current_log = open(log_path, 'w')
            print(f"Starting new log file: {log_path}")

        json.dump(event, current_log)
        current_log.write('\n')
        current_log.flush()
        last_ts = ts

    if current_log:
        current_log.close()

if __name__ == "__main__":
    main()
