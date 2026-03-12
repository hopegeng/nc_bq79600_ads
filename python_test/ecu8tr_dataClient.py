import socket
import struct
import zlib
import sys

TOTAL_DATA_SIZE = 43


CELL_MEASUREMENT_FACTOR = 5000.0/65536.0
BLOCK_MEASUREMENT_FACTOR = 60000.0/65536.0
TLE9012_EXT_OT_CONV	= 6250.0/1024.0

INT_TEMP_LSB = 0.66                         # Page 36: formula 7.1 
INT_TEMP_BASE = 547.3

ADC_RESOLUTION = 0.1   # 0.1 mV per LSB (100 µV)
NUM_CELLS = 18

# Constants
IP = "192.168.1.200"
PORT = 8001
MAX_TLE9012_DATA_ELEMENT = 32
EXPECTED_DELIMITER = 0x2424  # Assuming "$$" translates to 0x2424
TOTAL_DATA_SIZE = 104   # 1+1+(3*32) + 4 + 2

BIT_WIDTH = 0

DEBUG = 0

def get_ext_temp( regVal ):
    intc = ( regVal >> 10 ) & 0x03
    result = regVal & 0x3FF
    return( TLE9012_EXT_OT_CONV * pow( 4, intc ) * result )

    

def get_internal_temp( regVal ):
    INTERNAL_TEMP = INT_TEMP_BASE - INT_TEMP_LSB * (float)( regVal & 0x3FF )
    return INTERNAL_TEMP

def find_delimiter(sock):
    buffer = b''
    while len(buffer) < 2:
        buffer += sock.recv(1)
        if buffer[-2:] == struct.pack("H", EXPECTED_DELIMITER):
            return True
    return False

# Data structure to hold received data
class ECU8TRData:
    def __init__(self):
        self.dev = None
        self.cell_volt = []  # List to hold ECU8TR_DATA_BODY_t elements
        self.crc = None
        self.delimiter = None

def connect_to_server(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(30)
    try:
        s.connect((ip, port))
        print("Connected to the server.")
    except socket.timeout:
        print("Connection timed out.")
        s = None
    except Exception as e:
        print(f"Failed to connect to the server: {e}")
        s = None
    return s

def receive_data(sock):
    
    sock.settimeout(120)  # 20 minutes

    while True:
        # --- Receive the exact number of bytes ---
        data = b""

        data = sock.recv( 1024 )
        print("Received the data")

        ecu8 = ECU8TRData()

        # --- Parse delimiter ---
        delimiter = struct.unpack("!H", data[-2:])[0]
        if delimiter != EXPECTED_DELIMITER:
            print("Delimiter mismatch, searching for next packet...")
            find_delimiter(sock)
            continue

        ecu8.delimiter = delimiter

        # --- Parse CRC ---
        ecu8.crc = struct.unpack("!I", data[-6:-2])[0]
        calculated_crc = zlib.crc32(data[:-6]) & 0xFFFFFFFF

        if ecu8.crc != calculated_crc:
            print(f"CRC error: received={ecu8.crc:#x}, calculated={calculated_crc:#x}")
            continue

        # --- Parse dev ---
        ecu8.dev = struct.unpack("!B", data[0:1])[0]

        # --- Parse cell_volt[18] ---
        cell_data = data[1:1 + 18*2]
        for i in range(18):
            offset = i * 2
            val = struct.unpack("!H", cell_data[offset:offset + 2])[0]
            ecu8.cell_volt.append(val)

        return ecu8

class DataReceptionError(Exception):
    pass
       
        
def get_physical( reg, digital ):
        
    return digital * ADC_RESOLUTION


def process_data( ecu8tr_data ):
    # If CRC and delimiter are valid, print the data

    print( "" )
    print(f"Device: {ecu8tr_data.dev}")

    for cell, cell_volt in enumerate(ecu8_data.cell_volt):
        phy = cell_volt * ADC_RESOLUTION
        print( f"cell {cell}:  cell volt in RAW = {cell_volt:#06x}, volt = {int(phy)} mV" )
 
    print( "" )

if __name__ == "__main__":

    sock = connect_to_server(IP, PORT)
    if sock:
        try:
            while True:
                ecu8_data = receive_data(sock)
                if ecu8_data is None:
                    raise DataReceptionError("Failed to receive data correctly.")
                process_data( ecu8_data )
        except DataReceptionError as e:
            if DEBUG:
                raise # re-raise the exception and track back gets printed
            else:
                print("{}: {}".format(type(e).__name__, e))
                
            print(f"Error: {e}")                
        finally:
            sock.close()
            print( "Server socket closed" )
