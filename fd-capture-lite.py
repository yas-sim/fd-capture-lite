# Options: "2D" / "2DD" / "2HD"
MEDIA_TYPE = "2DD"


import sys
import re

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as ex:
    print('This program requires "pyserial". Install it with "pip install pyserial" command., and try again')
    sys.exit()

class bitarray:
    def __init__(self):
        self.data = bytearray()
        self.bit_pos = 7
        self.current_data = 0
        self.total_bits = 0

    def append(self, bit:int):
        self.total_bits += 1
        if bit != 0:
            self.current_data |= (1 << self.bit_pos)

        if self.bit_pos > 0:
            self.bit_pos -= 1
        else:
            self.data.append(self.current_data)
            self.bit_pos = 7
            self.current_data = 0

    def flush(self):
        if self.bit_pos != 7:
            self.data.append(self.current_data)
            self.current_byte = 0


def decode_to_track_image(track_encoded):
    if MEDIA_TYPE == "2D" or MEDIA_TYPE == "2DD":
        samples_per_cell = 8            # 2DD
    else:
        samples_per_cell = 4            # 2HD
    out_buf = bitarray()
    for enc in track_encoded:
        decoded = ord(enc) - 0x20
        for bit_pos in range(6):
            bit = 1 if (decoded & (1 << bit_pos)) != 0 else 0       # track data (LSB first)
            for c in range(samples_per_cell):
                out_buf.append(bit if c==(samples_per_cell>>1) else 0)
    out_buf.flush()
    #print(out_buf.total_bits)
    return out_buf.data

def set_uint64(barray:bytearray, pos:int, val:int):
    val=int(val)
    barray[pos  ] = ( val        & 0x00ff)
    barray[pos+1] = ((val >> 8)  & 0x00ff)
    barray[pos+2] = ((val >> 16) & 0x00ff)
    barray[pos+3] = ((val >> 24) & 0x00ff)
    barray[pos+4] = ((val >> 32) & 0x00ff)
    barray[pos+5] = ((val >> 40) & 0x00ff)
    barray[pos+6] = ((val >> 48) & 0x00ff)
    barray[pos+7] = ((val >> 56) & 0x00ff)

def set_string(barray:bytearray, pos:int, val:str):
    for d, c in enumerate(val):
        barray[pos+d] = ord(c)

def dump_byte_buffer(buff, pos=-1):
    for cnt, dt in enumerate(buff):
        if cnt % 0x10==0:
            print(f'\n{cnt:04x} ', end='')
        if (dt>=0x20 or dt<=0x7e) and pos!=cnt:
            print(f' {dt:02x} ', end='')
        else:
            print(f'*{dt:02x} ', end='')
    print()

def write_mfm_image(track_images, file_name:str):
    mfm_header_pos           = 0
    mfm_track_table_pos      = 0x100
    mfm_track_data_start_pos = 0x1000

    mfm_header = bytearray(8*6)
    set_string(mfm_header, 0,  'MFM_IMG ')
    set_uint64(mfm_header, 8,  mfm_track_table_pos) # Track table offset
    set_uint64(mfm_header, 16, len(track_images))   # number of track images
    set_uint64(mfm_header, 24, 0.2/1e-9)            # spindle rotation time (ns)
    if MEDIA_TYPE == "2D" or MEDIA_TYPE == "2DD":
        set_uint64(mfm_header, 32, 500e3)           # data bit rate (2d/2dd=500k)
    else:
        set_uint64(mfm_header, 32, 1e6)             # data bit rate (2hd=1M)
    set_uint64(mfm_header, 40, 4e6)                 # sample rate (4MHz)

    if MEDIA_TYPE == "2D":
        track_table = bytearray(16*(80+4))
    else:
        track_table = bytearray(16*(160+4))

    track_data_pos = mfm_track_data_start_pos       # start position of actual track image data
    track_data_pos_list = []
    for trk_n, trk in enumerate(track_images):
        track_data_pos_list.append(track_data_pos)              # memo for later use
        set_uint64(track_table, 16*trk_n  , track_data_pos)     # track data offset
        set_uint64(track_table, 16*trk_n+8, len(trk)*8)         # track image data length (bits)
        size = len(trk)
        roundup = ((size//256)+1)*256
        track_data_pos += roundup

    with open(file_name, "wb") as f:
        f.seek(mfm_header_pos)
        f.write(mfm_header)
        f.seek(mfm_track_table_pos)
        f.write(track_table)
        for trk_n, trk in enumerate(track_images):
            f.seek(track_data_pos_list[trk_n])
            f.write(trk)



def detect_arduino():
    ports = list_ports.comports()
    aport = None
    for info in ports:
        if info.vid==0x2341:    # Found Arduino Uno (VID==0x2341)
            aport = info.device
    return aport


def main():
    print("** FD-CAPTURE-LITE")

    # Search an Arduino and open UART
    print('[HOST] Searching for Arduino')
    arduino_port = detect_arduino()
    if arduino_port is None:
        print('[ERROR] Arduino is not found')
        sys.exit(1)
    else:
        print('[HOST] Arduino is found on "{}"'.format(arduino_port))
    try:
        uart = serial.Serial(arduino_port, baudrate=2e6, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    except serial.serialutil.SerialException:
        print(f'[ERROR] {arduino_port} is in use.')
        sys.exit(1)

    uart.reset_input_buffer()
    uart.reset_output_buffer()

    track_buffers = []
    track_count = 0

    print(f"[HOST] Floppy media type setting = {MEDIA_TYPE}")

    reading = False
    while True:
        buf = uart.readline()
        try:
            line = buf.decode('utf-8').rstrip('\n').rstrip('\r')
        except UnicodeDecodeError as ex:
            print()
            match = re.search(r'position (\d+):', str(ex))
            err_pos = int(match.groups()[0])
            dump_byte_buffer(buf, err_pos)
            print(f'[ERROR] Unicode decode error. Pos=0x{err_pos:04x}')
            sys.exit()

        if line[:2]=='++':                      # Signal
            if line[:8]=='++START':
                reading = True
            elif line[:6]=='++END':
                reading = False
                break
        elif line[:2]=='@@':                    # Message
            print(f'[ARDUINO] {line[2:]}')
        elif line[:2]=='==' and reading==True:  # Data
            track_buffers.append(line)
            print(f'{track_count:d} ', end='', flush=True)
            track_count+=1
    uart.close();

    print()    
    print('[HOST] Image read completed.')
    print('[HOST] Converting read data into MFM disk image data')
    track_images = []
    for trk_n, trk in enumerate(track_buffers):
        print(f'{trk_n} ', end='', flush=True)
        track_images.append(decode_to_track_image(trk))
    print()

    image_file_name = 'image.mfm'
    write_mfm_image(track_images, image_file_name)
    print(f'[HOST] Completed -> "{image_file_name}".')

if __name__ == "__main__":
    main()
