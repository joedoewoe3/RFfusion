using SerialPorts

sp = SerialPort("COM10", 115200)
sp2 = SerialPort("COM11", 115200)
sp3 = SerialPort("COM12", 115200)
open(sp)
open(sp2)
open(sp3)

function send_at_command(sp::SerialPort, cmd::String; save_changes::Bool=true, is_query::Bool=false)
    sleep(2)
    write(sp, "+++")
    sleep(2)
    response = readavailable(sp)
    response_str = strip(String(response))  # Strip junk
    println("Enter response: $response_str")
    if !contains(response_str, "OK")
        error("Failed enter: $response_str")
    end
    println("Command mode entered!")

    write(sp, cmd * "\r\n")
    sleep(0.5)
    response = readavailable(sp)
    response_str = strip(String(response))
    println("Response to $cmd: $response_str")
    if contains(response_str, "ERROR")
        error("Command $cmd failed: $response_str")
    end

    if save_changes && !is_query
        write(sp, "ATWR\r\n")
        sleep(0.5)
        wr_response = strip(String(readavailable(sp)))
        println("Save response: $wr_response")
        if !contains(wr_response, "OK")
            error("Failed save: $wr_response")
        end
    end

    write(sp, "ATCN\r\n")
    sleep(0.5)
    exit_response = strip(String(readavailable(sp)))
    println("Exit response: $exit_response")
end

function send_api_at_command(sp::SerialPort, cmd::String; remote_addr::String="")
    # API frame: 7E | length (2B) | type 08 local/17 remote | frame ID | AT cmd (2B) | param | checksum
    frame_id = 0x01  # Arbitrary
    at_cmd = codeunits(cmd[3:end])  # e.g., "ND" as [0x4E, 0x44]
    param = UInt8[]  # For sets; empty for query
    if remote_addr != ""
        type = 0x17  # Remote AT
        dest_addr = hex2bytes(remote_addr)  # e.g., "0013A200426A689B" as 8B
        header = [type, frame_id, dest_addr..., 0xFF, 0xFE, 0x02]  # Remote opts
    else
        type = 0x08  # Local AT
        header = [type, frame_id]
    end
    data = [header..., at_cmd..., param...]
    api_length = hton(UInt16(length(data)))
    checksum = 0xFF - (sum(data) % 0x100)
    frame = [0x7E, reinterpret(UInt8, [api_length])..., data..., checksum]
    write(sp, frame)
    sleep(0.5)
    response = readavailable(sp)
    response_bytes = Vector{UInt8}(codeunits(response))  # To bytes
    println("API response hex: $(bytes2hex(response_bytes))")
    # Parse frame: Validate 7E start, extract payload after length/check
    if !isempty(response_bytes) && response_bytes[1] == 0x7E
        resp_len = ntoh(reinterpret(UInt16, response_bytes[2:3]))
        payload = response_bytes[4:3+resp_len]
        status = payload[end-1]  # 0x00 OK
        if status != 0x00
            error("API status error: $(hex(status))")
        end
        return payload  # Raw data for ND/DB etc.
    end
    return nothing
end



using SerialPorts

function build_transmit_frame(sp::SerialPort, dest_addr::UInt64, frame_id::UInt8, options::UInt8, data::String)
    frame_type = 0x00
    dest_bytes = hton(dest_addr)  # Big-endian 64-bit (htol flips little to big)
    data_bytes = Vector{UInt8}(codeunits(data))
    frame_data = [frame_type, frame_id, dest_bytes..., options, data_bytes...]
    length = ntoh(Base.length(frame_data))  # Big-endian 2 bytes
    checksum = 0xFF - (sum(frame_data) % 0x100)
    frame = [0x7E, length..., frame_data..., checksum]

    write(sp, frame)  # Send the bytes
    println("Sent frame hex: $frame")
    sleep(0.5)  # Wait for response
    response = readavailable(sp)
    println("Response hex: $(bytes2hex(response))")  # Parse 0x8B status next
end

# Usage: Broadcast "Ping!" from coord
sp = open("COM10", 115200)  # Your baud
broadcast_addr = 0x000000000000FFFF
frame_id = 0x01  # Arbitrary, matches in status
options = 0x00  # No ACK
data = "Ping!"
build_transmit_frame(sp, broadcast_addr, frame_id, options, data)

end_at_command(sp, "ATID", is_query=true)  # Expect 1112
send_at_command(sp, "ATBR", is_query=true)  # Expect 1
send_at_command(sp, "ATAP", is_query=true)  # Expect 1
send_at_command(sp, "ATCE", is_query=true)  # 1 on coord, 0 on routers
send_at_command(sp, "ATBD 7", is_query=true)
#send_at_command(sp, "ATNJ FF")  # Indefinite join time to force assoc
send_at_command(sp, "ATWR")
sleep(1)