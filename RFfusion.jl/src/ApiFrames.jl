
function xbee_api_frame_wrap(payload::Vector{UInt8}; frame_id=0x01, dest_addr=0x0000FFFFFFFFFFFF, broadcast_radius=0x00, options=0x00)
    # Transmit Request 0x10: [frame_type, frame_id, dest_64, dest_16=0xFFFE, broadcast_radius, options, data...]
    frame_type = UInt8(0x10)
    dest_16 = UInt8[0xFF, 0xFE] # Unknown network addr
    dest_addr_bytes = reinterpret(UInt8, [hton(UInt64(dest_addr))])[1:8] # Big-endian 64-bit
    inner = vcat(frame_type, UInt8(frame_id), dest_addr_bytes, dest_16, UInt8(broadcast_radius), UInt8(options), payload)
    len_bytes = reinterpret(UInt8, [hton(UInt16(length(inner)))])[1:2] # MSB first
    sum_low = sum(inner) % 0x100  # Mod 256 for low byte (avoids large sum trunc error)
    checksum = 0xFF - UInt8(sum_low)  # XBee checksum on low byte
    framed = vcat(UInt8(0x7E), len_bytes, inner, checksum)
    return framed
end

# TODO: Integrate CRC if needed (e.g., append post-frame for streaming verify)

function parse_xbee_frame(framed_bytes::Vector{UInt8})
    if length(framed_bytes) < 4 || framed_bytes[1] != 0x7E
        error("Invalid XBee frame start or too short: len=$(length(framed_bytes))")
    end

    len = ntoh(reinterpret(UInt16, framed_bytes[2:3])[1])
    if length(framed_bytes) != len + 4  # Exact match (your send is full frame)
        error("Frame len mismatch: expected $(len+4), got $(length(framed_bytes))")
    end

    inner = framed_bytes[4:3+len]
    checksum_received = framed_bytes[4+len]

    sum_low = sum(inner) % 0x100
    checksum_calc = 0xFF - UInt8(sum_low)
    if checksum_received != checksum_calc
        error("XBee checksum mismatch: calc=0x$(string(checksum_calc, base=16)), received=0x$(string(checksum_received, base=16))")
    end

    return inner
end
