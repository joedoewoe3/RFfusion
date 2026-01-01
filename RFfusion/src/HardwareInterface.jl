module HardwareInterface
using SerialPorts

export connect_antenna, read_raw_data, program_antenna

function connect_antenna(port::String; baud=9600)  # E.g., "COM3" or "/dev/ttyUSB0"
    sp = SerialPort(port, baud)
    open(sp)
    return sp
end

function read_raw_data(sp::SerialPort; timeout=1.0)
    # Stub: Read RSS/TDOA packet from UART
    bytes = readavailable(sp, timeout=timeout)
    # Parse to AntennaData (e.g., custom format or direct proto bytes)
    return decode_packet(bytes)  # Tie to PacketUtils
end

function program_antenna(sp::SerialPort, config_bytes::Vector{UInt8})
    write(sp, config_bytes)
    # Confirm with read response
end

# Close: close(sp)
end