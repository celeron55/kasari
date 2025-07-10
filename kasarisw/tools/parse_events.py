import sys
import struct

def parse_event(buffer):
    """Parse a single event from the buffer, returning the event and remaining buffer."""
    if len(buffer) < 1:
        return None, buffer
    tag = buffer[0]
    
    if tag == 0:  # Lidar
        if len(buffer) < 25:
            return None, buffer
        data = buffer[1:25]
        timestamp, d1, d2, d3, d4 = struct.unpack('<Qffff', data)
        return ('Lidar', timestamp, d1, d2, d3, d4), buffer[25:]
    
    elif tag == 1:  # Accelerometer
        if len(buffer) < 17:
            return None, buffer
        data = buffer[1:17]
        timestamp, acceleration_y, acceleration_z = struct.unpack('<Qff', data)
        return ('Accelerometer', timestamp, acceleration_y, acceleration_z), buffer[17:]
    
    elif tag == 2:  # Receiver
        if len(buffer) < 11:
            return None, buffer
        timestamp, channel, flag = struct.unpack('<QBB', buffer[1:11])
        if flag == 0:
            return ('Receiver', timestamp, channel, None), buffer[11:]
        elif flag == 1:
            if len(buffer) < 15:
                return None, buffer
            pulse_length = struct.unpack('<f', buffer[11:15])[0]
            return ('Receiver', timestamp, channel, pulse_length), buffer[15:]
        else:
            # Invalid flag, skip min event size (tag + 10 bytes)
            return None, buffer[11:]
    
    elif tag == 3:  # Vbat
        if len(buffer) < 13:
            return None, buffer
        data = buffer[1:13]
        timestamp, voltage = struct.unpack('<Qf', data)
        return ('Vbat', timestamp, voltage), buffer[13:]
    
    else:
        # Unknown tag, skip 1 byte
        return None, buffer[1:]

# Main entry point
if __name__ == "__main__":
    # Initialize an empty buffer
    buffer = b''

    # Main loop to process the stream
    while True:
        # Parse as many as possible, handling skips without premature break
        while len(buffer) >= 1:
            old_len = len(buffer)
            event, buffer = parse_event(buffer)
            if event is not None:
                print(event)
            else:
                if len(buffer) >= old_len:
                    # No progress (incomplete event), wait for more
                    break
                # Else: skipped invalid, continue parsing remaining buffer

        # Read more data from stdin
        chunk = sys.stdin.buffer.read(1024)
        if not chunk:
            break
        buffer += chunk
