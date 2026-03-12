import socket
import struct

UDP_IP = "0.0.0.0"     # Listen on all interfaces
UDP_PORT = 65535        # Match this with your lwIP sender port
ADC_RESOLUTION = 0.1   # 0.1 mV per LSB (100 µV)
NUM_CELLS = 18

def parse_cell_voltages(data):
    """Convert 36-byte raw data into signed mV values."""
    if len(data) < NUM_CELLS * 2:
        print(f"Received {len(data)} bytes, expected {NUM_CELLS*2}")
        return None

    volts_mV = []
    for i in range(NUM_CELLS):
        # Unpack one signed 16-bit value (big-endian)
        raw_val = struct.unpack_from("<h", data, i * 2)[0]  # '<h' = little-endian signed short
        voltage_mV = raw_val * ADC_RESOLUTION
        volts_mV.append(voltage_mV)
    return volts_mV


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(5.0)
    print(f"Listening for BMS UDP packets on port {UDP_PORT}...")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"\nReceived {len(data)} bytes from {addr}")

        voltages = parse_cell_voltages(data)
        if voltages is not None:
            for idx, v in enumerate(voltages, start=1):
                print(f"Cell {idx:02d}: {v:.1f} mV")


if __name__ == "__main__":
    main()
