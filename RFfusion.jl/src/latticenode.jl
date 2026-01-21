#First bring in c library

const LIB_RF = joinpath(@__DIR__,"../lib", "libRFWrapper.so")
Libc.Libdl.dlopen(LIB_RF)  # Preload to validate

const rf_open = (port::String, baud::Int) -> ccall((:rf_open, LIB_RF), Ptr{Cvoid}, (Cstring, Cint), port, baud)
const rf_write_packet = (handle::Ptr{Cvoid}, data::Vector{UInt8}) -> ccall((:rf_write_packet, LIB_RF), Cint, (Ptr{Cvoid}, Ptr{UInt8}, Csize_t), handle, data, length(data))
const rf_close = (handle::Ptr{Cvoid}) -> ccall((:rf_close, LIB_RF), Cvoid, (Ptr{Cvoid},), handle)
#const rf_read_packet = (handle::Ptr{Cvoid}, max_len::Csize_t) -> ccall((:rf_read_packet, LIB_RF), Cint, (Ptr{Cvoid}, Ptr{UInt8}, Csize_t), handle, Vector{UInt8}(undef, max_len), max_len)  # Add to ccalls if in C
#const rf_read_packet = (handle::Ptr{Cvoid}, max_len) -> ccall((:rf_read_packet, LIB_RF), Cint, (Ptr{Cvoid}, Ptr{UInt8}, Csize_t), handle, Vector{UInt8}(undef, max_len), Csize_t(max_len))  # Cast max_len to Csize_t
const rf_read_packet = (handle::Ptr{Cvoid}, buf::Vector{UInt8}, max_len) -> ccall((:rf_read_packet, LIB_RF), Cint, (Ptr{Cvoid}, Ptr{UInt8}, Csize_t), handle, buf, Csize_t(max_len))  # Buf as arg, cast max_len


struct LatticeNode
    id::Int #simple
    port::String #"COM" or "/dev/ttyUSB0"
    baud::Int
    proto_config::Dict{String, Any}  #we have our utf8 and our bytes fields
    handle::Ptr{Cvoid} #  handle::Union{SerialPort, Ptr{Cvoid}}
    mac::String
end

#=
struct LatticeNode
    id::Int
    ip::String  # IPv6 addr
    port::Int  # UDP port, e.g., 5000
    proto_config::Dict{String, Any}  # Your proto fields
    sock::UDPSocket  # Open UDP sock instead of fd Ptr
end

=#
#dev_dir = "/dev"
#tty_ports = filter(x -> occursin(r"^tty(USB|ACM)", x), readdir(dev_dir))
#full_ports = ["/dev/$p" for p in sort(tty_ports)]

function build_lattice_dict()
    dev_dir = "/dev"
    tty_ports = filter(x -> occursin(r"^tty(USB|ACM)", x), readdir(dev_dir))
    full_ports = ["/dev/$p" for p in sort(tty_ports)]

        default_proto = Dict(
            "utf8_fields" => ["node_label", "timestamp", "status_msg"],
            "encoding" => "proto3",
            "bytes_fields" => ["rf_packet", "firmware_update", "spectrum_bytes"]
            )

        dict = Dict{Int, LatticeNode}()
        for (i, port) in enumerate(full_ports)
            node = LatticeNode(i, port, 115200, default_proto, Ptr{Cvoid}(0), "")
            dict[i] = node
        end

        return dict
    end

    const lattice_dict = build_lattice_dict()  # Const global—build once
    println("Lattice Dict Setup: ", lattice_dict)  # Inspect

    # Build the dict TODO Dynamic port detection OS agnostic for the antenna_data__field_descriptors
    #lattice_dict = Dict{Int, LatticeNode}()
    #for (i, port) in enumerate(full_ports)  # i=1 to 6, ports like "/dev/ttyUSB0"
    # Handle starts as C_NULL; open later with ccall
    #    node = LatticeNode(i, port, 115200, default_proto, Ptr{Cvoid}(0))
    #    lattice_dict[i] = node
    #end

    function open_node(node)
        if node.handle != Ptr{Cvoid}(0)
            rf_close(node.handle)
        end
        handle = rf_open(node.port, node.baud)
        if handle == Ptr{Cvoid}(0)
            error("Open failed for $(node.port)")
            end
            new_node = LatticeNode(node.id, node.port, node.baud, node.proto_config, handle, node.mac)
            return new_node
        end

    function close_node(node::LatticeNode)
        if node.handle != Ptr{Cvoid}(0)
            rf_close(node.handle)
        end
    end

    # Send a proto-encoded packet (Julia encodes UTF-8/bytes per proto, C writes raw bytes)
    function send_packet_serial(node_id::Int, packet::RFpacket_pb.RFPacket)  # Tweak RFpacket to your gen'd struct name
        node = lattice_dict[node_id]
        node = open_node(node)  # Open if needed
        lattice_dict[node_id] = node # persist i.e hold open

        # Encode via PacketUtils ( CRC is set up... have to do that now, no TODOs for this )
        bytes = encode_packet(packet)  # Assumes it returns Vector{UInt8}; tweak if IOBuffer
        # Update in send_packet (replace framed_bytes = bytes)

        framed_bytes = xbee_api_frame_wrap(bytes; dest_addr=0x000000000000FFFF)  # Broadcast example; tweak per node

        println(length(framed_bytes))

        res = rf_write_packet(node.handle, framed_bytes)   # C write

        if res != 0
            #error("C write failed for node `$("node_id—code")` $res; check antenna response")
            error("C write failed for node $node_id-code $res; check antenna response")
            end
            println("Packet sent to LatticeNode $node_id: $(packet.node_label)")
        end

        # Receive from serial (blocks with timeout)
        function receive_packet_serial(node_id::Int; max_len=1200, timeout_sec=2.0, use_crc=false, use_blob_crc=false)
            node = lattice_dict[node_id]
            node = open_node(node)  # Open if needed
            lattice_dict[node_id] = node

            buf = Vector{UInt8}(undef, max_len)
            start_time = time()
            inner_bytes = nothing

            while isnothing(inner_bytes) && (time() - start_time < timeout_sec)
                bytes_read = rf_read_packet(node.handle, buf, max_len)  # C read
                if bytes_read < 0
                    if Libc.errno() == Libc.EAGAIN  # No data, non-blocking normal
                        sleep(0.01)
                        continue
                    end
                    error("C read failed for node $node_id-code $bytes_read; check antenna")
                    end

                    if bytes_read > 0
                        framed_bytes = buf[1:bytes_read]
                        println("Received framed len: $(length(framed_bytes))")
                        println("Framed bytes head hex: ", join([@sprintf("%02x ", b) for b in framed_bytes[1:min(32, end)]]))

                        try
                            temp_inner = parse_xbee_frame(framed_bytes)
                            if temp_inner[1] == 0x90  # Data RX
                                inner_bytes = temp_inner
                                break
                            else
                                println("Skipped non-data frame type: 0x$(string(temp_inner[1], base=16))")
                            end
                            catch e
                            println("Parse error: $e - skipping bad frame")
                        end
                        end
                        sleep(0.01)
                    end
                    if isnothing(inner_bytes)
                        error("Timeout receiving data from node $node_id after $timeout_sec sec (saw status, but no 0x90 RX)")
                    end

                    println("Inner bytes len: $(length(inner_bytes))")
                    println("Inner bytes head hex: ", join([@sprintf("%02x ", b) for b in inner_bytes[1:min(32, end)]]))

                        pkt = decode_packet(inner_bytes; use_crc=use_crc, use_blob_crc=use_blob_crc)
                        println("Raw packet received from LatticeNode $node_id: $(pkt.node_label)")

                        return pkt
                    end

                    #=
                    # Overload for raw bytes (no encode—direct frame/write)
                    function send_packet_serial(node_id::Int, bytes::Vector{UInt8})
                    node = lattice_dict[node_id]
                    node = open_node(node)  # Open if needed
                    lattice_dict[node_id] = node

                    framed_bytes = xbee_api_frame(bytes; dest_addr=0x000000000000FFFF)  # Broadcast; tweak per node

                    res = rf_write_packet(node.handle, framed_bytes)

                    if res != 0
                    error("C write failed for node $node_id—code $res; check antenna response")
                    end
                    println("Raw bytes sent to LatticeNode $node_id")
                    end


                    =#
#=
                    # Full SignalPacket (header + kind + oneof payload + crc32)
                    #   test_packet = SignalPacket(
                    #      test_header,  # Header
                    #     PayloadKind.RAW,  # Enum (0 for RAW)
                    #    UInt32(0)  # crc32 placeholder (calc real via hash or zlib.crc32(bytes) if needed)
                    #   )
                    #test_packet.raw = test_raw  # Set the oneof (ProtoBuf allows post-construct)

                    # Now send it (from Step 4 func)
                    #send_packet(1, test_packet)  # To node 1; watch for success println

=#

#=
function init_lattice_dict()
    serial_dict = build_serial_lattice_dict()  # Your old serial setup
    ips = discover_antennas()  # Grab UDP ips
    lattice_dict = Dict{String, LatticeNode}()
    for (i, ip) in enumerate(ips)
        label = "latticenode_$i"
        # UDP node (baud -> port)
        lattice_dict["udp_" * label] = LatticeNode(i, :udp, ip, 5000, Dict("utf8_fields" => ["node_label", "timestamp", "status_msg"], "encoding" => "proto3", "bytes_fields" => ["rf_packet", "firmware_update", "spectrum_bytes"]), Nothing)
        end
        merge!(lattice_dict, Dict("serial_" * "latticenode_$k" => v for (k,v) in serial_dict))  # Add serial as separate keys to avoid confusion
        end
    return lattice_dict
end

=#
#=
function query_at(node_id::Int, at_cmd::String; timeout_sec=2.0)
    node = lattice_dict[node_id]
    node = open_node(node)  # Open if needed
    lattice_dict[node_id] = node

    # Build AT frame (0x08 local for now; 0x17 for remote if peer)
    frame_id = 0x01
    at_bytes = codeunits(at_cmd)  # e.g., [0x53, 0x48] for "SH"
    data = [0x08, frame_id, at_bytes...]
    api_length = hton(UInt16(length(data)))
    checksum = 0xFF - (sum(data) % 0x100)
    frame = [0x7E, reinterpret(UInt8, [api_length])..., data..., checksum]

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
        end
        # Read response (0x88 AT resp)
        buf = Vector{UInt8}(undef, 256)  # Safe over NP
        start_time = time()
        while time() - start_time < timeout_sec
            bytes_read = rf_read_packet(node.handle, buf, 256)
            if bytes_read > 0
                resp_bytes = buf[1:bytes_read]
                if resp_bytes[1] == 0x7E
                    resp_len = ntoh(reinterpret(UInt16, resp_bytes[2:3])[1])
                    payload = resp_bytes[4:3+resp_len]
                    if payload[1] == 0x88 && payload[3:4] == at_bytes  # Type/ID/AT match
                        status = payload[5]
                        if status == 0x00
                            return payload[6:end]  # Value bytes (e.g., 4B for SH)
                        else
                            error("AT $at_cmd status error: 0x$(string(status, base=16))")
                        end
                    end
                end
            end
            sleep(0.01)
        end
        error("Timeout on AT $at_cmd for node $node_id")
        end

        function auto_fill_lattice_dict!()
            for (id, node) in lattice_dict
                try
                    # Grab SH/SL (4B each, hex string MAC)
                    sh_bytes = query_at(id, "SH")
                    sl_bytes = query_at(id, "SL")
                    sh = bytes2hex(sh_bytes)
                    sl = bytes2hex(sl_bytes)
                    mac = sh * sl  # Full 64-bit hex

                    # NI label
                    ni_bytes = query_at(id, "NI")
                    ni = String(ni_bytes[1:end-1])  # Drop null if any

                    # BD baud confirm (1B, value 0-A)
                    bd_byte = query_at(id, "BD")
                    bd = reinterpret(UInt8, bd_byte)[1]  # Code to baud (e.g., 7=115200)

                    # VR firmware version (2B hex)
                    vr_bytes = query_at(id, "VR")
                    vr = bytes2hex(vr_bytes)

                    # Update node (extend struct if needed, or Dict in lattice_dict)
                    lattice_dict[id] = merge(node, (mac=mac, ni=ni, bd=bd, vr=vr))  # If Dict; or new struct field
                    println("Filled node $id: MAC $mac, NI $ni, BD $bd, VR $vr")
                    catch e
                    println("Query fail on node $id: $e-manual fill?")
                end
            end
        end

        # Cal
=#

#=
function query_at(node_id::Int, at_cmd::String; timeout_sec=5.0)
    node = lattice_dict[node_id]
    node = open_node(node) # Open if needed
    lattice_dict[node_id] = node
    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[0x08, frame_id, at_bytes...]  # Unescaped payload

    # For AP=2: Escape the data (length is of escaped data)
    escaped_data = UInt8[]
    for b in data
        if b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end

    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
    end

    buf = Vector{UInt8}(undef, 512)
    start_time = time()
    total_bytes = 0
    while time() - start_time < timeout_sec
        bytes_read = rf_read_packet(node.handle, buf, 512)
        if bytes_read > 0
            total_bytes += bytes_read
            resp_bytes = buf[1:bytes_read]
            println("Read $(bytes_read) bytes raw: $(bytes2hex(resp_bytes))")  # Debug raw

            # Find start 0x7E (skip garbage if any)
            idx = findfirst(==(0x7E), resp_bytes)
            if isnothing(idx)
                println("No frame start")
                continue
            end
            resp_bytes = resp_bytes[idx:end]

            if length(resp_bytes) < 4
                continue  # Too short for length
            end

            # Length is of ESCAPED payload
            resp_len = ntoh(reinterpret(UInt16, resp_bytes[2:3])[1])

            # Need at least 3 (hdr) + resp_len (escaped payload) + 1 (checksum)
            min_needed = 3 + resp_len + 1
            if length(resp_bytes) < min_needed
                println("Incomplete frame: have $(length(resp_bytes)), need $min_needed")
                continue
            end

            escaped_payload = resp_bytes[4:3+resp_len]
            checksum_received = resp_bytes[3+resp_len+1]

            # Unescape the payload
            payload = UInt8[]
            i = 1
            while i <= length(escaped_payload)
                if escaped_payload[i] == 0x7D
                    if i < length(escaped_payload)
                        push!(payload, escaped_payload[i+1] ⊻ 0x20)
                        i += 2
                    else
                        println("Invalid escape at end")
                        break
                    end
                else
                    push!(payload, escaped_payload[i])
                    i += 1
                end
            end

            # Checksum over UNESCAPED payload
            checksum_calc = UInt8(0xFF - (sum(payload) % 0x100))

            recovery_success = false
            if checksum_received != checksum_calc
                println("Initial checksum fail: got 0x$(string(checksum_received, base=16)), calc 0x$(string(checksum_calc, base=16))")
                # Recovery: If extra byte and next byte matches calc, assume len undercount by 1
                if length(resp_bytes) >= min_needed + 1
                    next_byte = resp_bytes[min_needed + 1]
                    if next_byte == checksum_calc
                        # Add the old checksum_received as last escaped byte
                        push!(escaped_payload, checksum_received)
                        # Re-unescape the new escaped_payload
                        payload = UInt8[]
                        i = 1
                        while i <= length(escaped_payload)
                            if escaped_payload[i] == 0x7D
                                if i < length(escaped_payload)
                                    push!(payload, escaped_payload[i+1] ⊻ 0x20)
                                    i += 2
                                else
                                    break
                                end
                            else
                                push!(payload, escaped_payload[i])
                                i += 1
                            end
                        end
                        # Re-calc checksum
                        checksum_calc = UInt8(0xFF - (sum(payload) % 0x100))
                        if next_byte == checksum_calc
                            checksum_received = next_byte
                            resp_len += 1  # Update for consistency
                            println("Recovery success: Assumed len undercount by 1, new checksum matches")
                            recovery_success = true
                        end
                    end
                end
                if !recovery_success
                    continue
                end
            end

            println("Unescaped payload: $(bytes2hex(payload))")  # Debug

            if length(payload) >= 5 && payload[1] == 0x88 && payload[2] == frame_id && payload[3:4] == at_bytes
                status = payload[5]
                if status == 0x00
                    value = payload[6:end]
                    println("Success: Value $(bytes2hex(value))")
                    return value
                else
                    error("AT $at_cmd status error: 0x$(string(status, base=16))")
                end
            else
                println("Not matching AT resp")
            end
        elseif bytes_read == 0
            # No data
        else
            error("Read error $bytes_read")
        end
        sleep(0.05)
    end
    println("Timeout after $total_bytes total bytes")
    error("Timeout on AT $at_cmd for node $node_id")
end
=#
#=
function query_at(node_id::Int, at_cmd::String; timeout_sec=5.0)
    node = lattice_dict[node_id]
    node = open_node(node) # Open if needed
    lattice_dict[node_id] = node
    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[0x08, frame_id, at_bytes...]  # Unescaped payload

    # For AP=2: Escape the data (length is of escaped data)
    escaped_data = UInt8[]
    for b in data
        if b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end

    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
        end
        buf = Vector{UInt8}()  # Persistent buffer across reads
        start_time = time()
        while time() - start_time < timeout_sec
            temp_buf = Vector{UInt8}(undef, 512)
            bytes_read = rf_read_packet(node.handle, temp_buf, 512)
            if bytes_read > 0
                append!(buf, temp_buf[1:bytes_read])
                println("Appended $bytes_read bytes, total buf len $(length(buf))")

                # Try parse from buf
                i = 1
                while i <= length(buf)
                    if buf[i] == 0x7E
                        if length(buf) - i < 3  # Need len
                            break
                        end
                        len = ntoh(reinterpret(UInt16, buf[i+1:i+2])[1])
                        i += 3  # To start of payload

                        unescaped = UInt8[]
                        escaped = false
                        start_i = i
                        while length(unescaped) < len && i <= length(buf)
                            b = buf[i]
                            if escaped
                                push!(unescaped, b ⊻ 0x20)
                                escaped = false
                                elseif b == 0x7D
                                escaped = true
                            else
                                push!(unescaped, b)
                            end
                            i += 1
                        end

                        if length(unescaped) < len
                            # Partial, continue accumulating
                            break
                        end

                        if i > length(buf)
                            break  # No checksum yet
                        end
                        checksum_received = buf[i]
                        i += 1

                        checksum_calc = UInt8(0xFF - (sum(unescaped) % 0x100))
                        if checksum_received != checksum_calc
                            println("Checksum fail: got 0x$(string(checksum_received, base=16)), calc 0x$(string(checksum_calc, base=16))")
                            # Skip bad frame, continue from i
                            continue
                        end

                        println("Unescaped payload: $(bytes2hex(unescaped))")
                        payload = unescaped  # frame_type + data = unescaped

                        if payload[1] == 0x88 && payload[2] == frame_id && payload[3:2+length(at_bytes)] == at_bytes
                            status = payload[3+length(at_bytes)]
                            if status == 0x00
                                value = payload[4+length(at_bytes):end]
                                println("Success: Value $(bytes2hex(value))")
                                return value
                            else
                                error("Status 0x$(string(status, base=16))")
                            end
                        else
                            println("Not matching AT resp")
                        end

                        # Remove parsed frame from buf
                        parsed_end = start_i + (i - start_i) - 1  # Adjust for increments
                        buf = buf[i:end]  # Shift left
                    else
                        i += 1  # Skip non-7E
                    end
                end
            end
            sleep(0.05)
        end
        error("Timeout on AT $at_cmd for node $node_id")
        end
=#
#=
function query_at(node_id::Int, at_cmd::String; timeout_sec=10.0, api_mode=2)
    node = lattice_dict[node_id]
    node = open_node(node) # Open if needed
    lattice_dict[node_id] = node
    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[0x08, frame_id, at_bytes...]  # Unescaped payload

    escaped_data = UInt8[]
    for b in data
        if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end

    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
        end

        buf = Vector{UInt8}()  # Accumulate all read bytes
        start_time = time()
        while time() - start_time < timeout_sec
            temp_buf = Vector{UInt8}(undef, 512)
            bytes_read = rf_read_packet(node.handle, temp_buf, 512)
            if bytes_read > 0
                append!(buf, temp_buf[1:bytes_read])
            end

            # Parse from buf (consume processed)
            i = 1
            while i <= length(buf)
                if buf[i] == 0x7E
                    # Unescape len (2 unescaped bytes)
                    len_bytes, new_i = unescape_n(buf, i+1, 2)
                    if isnothing(len_bytes)
                        break  # Not enough
                    end
                    len = ntoh(reinterpret(UInt16, len_bytes)[1])
                    i = new_i

                    # Unescape data (len unescaped bytes)
                    unescaped_data, new_i = unescape_n(buf, i, len)
                    if isnothing(unescaped_data)
                        break
                    end
                    i = new_i

                    # Unescape checksum (1 unescaped byte)
                    checksum_bytes, new_i = unescape_n(buf, i, 1)
                    if isnothing(checksum_bytes)
                        break
                    end
                    checksum_received = checksum_bytes[1]
                    i = new_i

                    # Verify
                    checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                    if checksum_received != checksum_calc
                        # Bad, skip to next 7E
                        i = findnext(==(0x7E), buf, i)
                        if isnothing(i)
                            break
                        end
                        continue
                    end

                    # Success, extract
                    if unescaped_data[1] == 0x88 && unescaped_data[2] == frame_id && unescaped_data[3:2+length(at_bytes)] == at_bytes
                        status = unescaped_data[3+length(at_bytes)]
                        if status == 0x00
                            value = unescaped_data[4+length(at_bytes):end]
                            return value
                        else
                            error("Status 0x$(string(status, base=16))")
                        end
                    end

                    # Not ours, continue
                else
                    i += 1
                end
            end

            # Remove processed (up to i-1)
            buf = buf[i:end]

            sleep(0.05)
        end
        error("Timeout on AT $at_cmd for node $node_id")
        end
=#
#=
        function query_at(node_id::Int, at_cmd::String; timeout_sec=10.0, api_mode=2, remote=false, dest_id=nothing)
            node = lattice_dict[node_id]  # Sender node
            if remote
                if isnothing(dest_id)
                    error("dest_id required for remote")
                    end
                    dest_node = lattice_dict[dest_id]
                    if isempty(dest_node.mac)
                        error("MAC not set for dest_id $dest_id")
                        end
                        dest_mac_bytes = hex2bytes(dest_node.mac)  # 8B UInt8[]
                    end

                    frame_id = 0x01::UInt8
                    at_bytes = Vector{UInt8}(codeunits(at_cmd))
                    data = UInt8[remote ? 0x17 : 0x08, frame_id]
                    if remote
                        append!(data, dest_mac_bytes)
                        append!(data, [0xFF, 0xFE])  # Unknown 16-bit addr
                        push!(data, 0x00)  # Options: 0x00 no apply, 0x02 to apply changes (for set)
                    end
                    append!(data, at_bytes)  # No value for query

                    # Escape if AP=2
                    escaped_data = UInt8[]
                    for b in data
                        if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
                            push!(escaped_data, 0x7D)
                            push!(escaped_data, b ⊻ 0x20)
                        else
                            push!(escaped_data, b)
                        end
                    end

                    api_length = hton(UInt16(length(escaped_data)))
                    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
                    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]

                    res = rf_write_packet(node.handle, frame)
                    if res != 0
                        error("Write failed for AT $at_cmd on node $node_id")
                        end

                        buf = Vector{UInt8}()
                        start_time = time()
                        while time() - start_time < timeout_sec
                            temp_buf = Vector{UInt8}(undef, 512)
                            bytes_read = rf_read_packet(node.handle, temp_buf, 512)
                            if bytes_read > 0
                                append!(buf, temp_buf[1:bytes_read])
                            end

                            i = 1
                            while i <= length(buf)
                                if buf[i] == 0x7E
                                    len_bytes, new_i = unescape_n(buf, i+1, 2, api_mode)
                                    if isnothing(len_bytes)
                                        break
                                    end
                                    len = ntoh(reinterpret(UInt16, len_bytes)[1])
                                    i = new_i

                                    unescaped_data, new_i = unescape_n(buf, i, len, api_mode)
                                    if isnothing(unescaped_data)
                                        break
                                    end
                                    i = new_i

                                    checksum_bytes, new_i = unescape_n(buf, i, 1, api_mode)
                                    if isnothing(checksum_bytes)
                                        break
                                    end
                                    checksum_received = checksum_bytes[1]
                                    i = new_i

                                    checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                                    if checksum_received != checksum_calc
                                        # Bad, skip to next 7E
                                        i = findnext(==(0x7E), buf, i)
                                        if isnothing(i)
                                            break
                                        end
                                        continue
                                    end

                                    resp_type = unescaped_data[1]
                                    if unescaped_data[1] == (remote ? 0x97 : 0x88) && unescaped_data[2] == frame_id && unescaped_data[3:2+length(at_bytes)] == at_bytes
                                        offset = remote ? (1 + 8 + 2 + 2) : (1 + 2)  # Skip src addr/net/at for remote
                                        status = unescaped_data[offset + length(at_bytes)]
                                        if status == 0x00
                                            value = unescaped_data[offset+1 + length(at_bytes):end]
                                            return value
                                        else   # Success, extract

                                            error("AT status error: 0x$(string(status, base=16))")
                                        end
                                    end
                                else
                                    i += 1
                                end
                            end
                            buf = buf[i:end]

                            sleep(0.05)
                        end
                        error("Timeout on AT $at_cmd for node $node_id")
                        end
=#
#=
function query_at(node_id::Int, at_cmd::String; timeout_sec=10.0, api_mode=2, remote=false, dest_id=nothing)
    node = lattice_dict[node_id]  # Sender
    if remote
        if isnothing(dest_id)
            error("dest_id required for remote")
        end
        dest_node = lattice_dict[dest_id]
        if isempty(dest_node.mac)
            error("MAC not set for dest_id $dest_id")
        end
        dest_mac_bytes = hex2bytes(dest_node.mac)  # 8B
    end

    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[remote ? 0x17 : 0x08, frame_id]
    if remote
        append!(data, dest_mac_bytes)
        append!(data, [0xFF, 0xFE])  # Unknown net addr
        push!(data, 0x00)  # Options (0x02 for set apply)
    end
    append!(data, at_bytes)  # No value for query

    # Escape
    escaped_data = UInt8[]
    for b in data
        if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end

    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
    end

    buf = Vector{UInt8}()
    start_time = time()
    while time() - start_time < timeout_sec
        temp_buf = Vector{UInt8}(undef, 512)
        bytes_read = rf_read_packet(node.handle, temp_buf, 512)
        if bytes_read > 0
            append!(buf, temp_buf[1:bytes_read])
        end

        i = 1
        while i <= length(buf)
            if buf[i] == 0x7E
                len_bytes, new_i = unescape_n(buf, i+1, 2, api_mode)
                if isnothing(len_bytes)
                    break
                end
                len = ntoh(reinterpret(UInt16, len_bytes)[1])
                i = new_i

                unescaped_data, new_i = unescape_n(buf, i, len, api_mode)
                if isnothing(unescaped_data)
                    break
                end
                i = new_i

                checksum_bytes, new_i = unescape_n(buf, i, 1, api_mode)
                if isnothing(checksum_bytes)
                    break
                end
                checksum_received = checksum_bytes[1]
                i = new_i

                checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                if checksum_received != checksum_calc
                    i = findnext(==(0x7E), buf, i)
                    if isnothing(i)
                        break
                    end
                    continue
                end

                if unescaped_data[1] == (remote ? 0x97 : 0x88) && unescaped_data[2] == frame_id
                    at_start = remote ? 3 + 8 + 2 : 3  # 1-based pos for at_cmd
                    if unescaped_data[at_start:at_start + length(at_bytes) - 1] == at_bytes
                        status_pos = at_start + length(at_bytes)
                        status = unescaped_data[status_pos]
                        if status == 0x00
                            value = unescaped_data[status_pos + 1:end]
                            return value
                        else
                            error("AT status error: 0x$(string(status, base=16))")
                        end
                    end
                end
            else
                i += 1
            end
        end
        buf = buf[i:end]
        sleep(0.05)
    end
    error("Timeout on AT $at_cmd for node $node_id")
end
=#

function query_at(node_id::Int, at_cmd::String; timeout_sec=10.0, api_mode=2, remote=false, dest_id=nothing)
    node = lattice_dict[node_id]  # Sender
    if remote
        if isnothing(dest_id)
            error("dest_id required for remote")
        end
        dest_node = lattice_dict[dest_id]
        if isempty(dest_node.mac)
            error("MAC not set for dest_id $dest_id")
        end
            dest_mac_bytes = hex2bytes(dest_node.mac)  # 8B
    end
    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[remote ? 0x17 : 0x08, frame_id]
    if remote
        append!(data, dest_mac_bytes)
        append!(data, [0xFF, 0xFE])  # Unknown net addr
        push!(data, 0x00)  # Options (0x02 for set apply)
    end
    append!(data, at_bytes)  # No value for query
    escaped_data = UInt8[]
    for b in data
        if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end
    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]
    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed for AT $at_cmd on node $node_id")
    end
    buf = Vector{UInt8}()
    start_time = time()
    while time() - start_time < timeout_sec
        temp_buf = Vector{UInt8}(undef, 512)
        bytes_read = rf_read_packet(node.handle, temp_buf, 512)
        if bytes_read > 0
            append!(buf, temp_buf[1:bytes_read])
            println("Raw bytes read: $(bytes2hex(temp_buf[1:bytes_read]))")
        end
        i = 1
        while i <= length(buf)
            if buf[i] == 0x7E
                len_bytes, new_i = unescape_n(buf, i+1, 2, api_mode)
                if isnothing(len_bytes)
                    break
                end
                len = ntoh(reinterpret(UInt16, len_bytes)[1])
                i = new_i
                unescaped_data, new_i = unescape_n(buf, i, len, api_mode)
                if isnothing(unescaped_data)
                    break
                end
                i = new_i
                checksum_bytes, new_i = unescape_n(buf, i, 1, api_mode)
                if isnothing(checksum_bytes)
                    break
                end
                checksum_received = checksum_bytes[1]
                i = new_i
                checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                if checksum_received != checksum_calc
                    i = findnext(==(0x7E), buf, i)
                    if isnothing(i)
                        break
                    end
                continue
                end
                println("Unexpected frame type: 0x$(string(unescaped_data[1], base=16)) - Unescaped: $(bytes2hex(unescaped_data))")
                if unescaped_data[1] == (remote ? 0x97 : 0x88) && unescaped_data[2] == frame_id
                    at_start = remote ? 3 + 8 + 2 : 3  # 1-based pos for at_cmd
                    if unescaped_data[at_start:at_start + length(at_bytes) - 1] == at_bytes
                        status_pos = at_start + length(at_bytes)
                        status = unescaped_data[status_pos]
                        if status == 0x00
                            value = unescaped_data[status_pos + 1:end]
                            return value
                        else
                            error("AT status error: 0x$(string(status, base=16))")
                        end
                    end
                end
                else
                    i += 1
                end
            end
        buf = buf[i:end]
        sleep(0.05)
        end
    error("Timeout on AT $at_cmd for node $node_id")
end
#=
        # Helper: Read and unescape exactly N unescaped bytes from buf[start_i:end], return bytes or nothing if not enough, and new start_i
        function unescape_n(buf, start_i, n)
            unesc = UInt8[]
            i = start_i
            escaped = false
            while length(unesc) < n && i <= length(buf)
                b = buf[i]
                if escaped
                    push!(unesc, b ⊻ 0x20)
                    escaped = false
                    elseif b == 0x7D
                    escaped = true
                else
                    push!(unesc, b)
                end
                i += 1
            end
            length(unesc) == n ? (unesc, i) : nothing
        end
=#

  function unescape_n(buf, start_i, n, api_mode)
      if api_mode != 2
          if length(buf) - start_i + 1 < n
              return nothing
          end
          return buf[start_i:start_i+n-1], start_i + n
      end
      unesc = UInt8[]
      i = start_i
      escaped = false
      while length(unesc) < n && i <= length(buf)
          b = buf[i]
          if escaped
              push!(unesc, b ⊻ 0x20)
              escaped = false
              elseif b == 0x7D
              escaped = true
          else
              push!(unesc, b)
          end
          i += 1
      end
      length(unesc) == n ? (unesc, i) : nothing
  end

        function auto_fill_lattice_dict!()
            failed_ids = Int[]
            for (id, node) in lattice_dict
                try
                    sh_bytes = query_at(id, "SH")
                    sl_bytes = query_at(id, "SL")
                    mac = bytes2hex([sh_bytes; sl_bytes])  # Full 64-bit hex, e.g., "0013a200426a6895"

                    ni_bytes = query_at(id, "NI")
                    ni = String(ni_bytes)  # Direct, no nulls in your sample

                    bd_bytes = query_at(id, "BD")
                    bd = ntoh(reinterpret(UInt32, bd_bytes)[1])  # e.g., 7::UInt32

                    vr_bytes = query_at(id, "VR")
                    vr = bytes2hex(vr_bytes)  # e.g., "9000"

                    # Merge (assume node is Dict or mutable)
                    lattice_dict[id] = merge(node, (;mac, ni, bd, vr))
                    println("Filled node $id: MAC $mac, NI $ni, BD $bd, VR $vr")
                    catch e
                    println("Query fail on node $id: $e")
                    push!(failed_ids, id)
                end
            end
            return failed_ids  # Retry list if needed
        end


function set_at(node_id::Int, at_cmd::String, value::Vector{UInt8}; timeout_sec=10.0, api_mode=2, remote=false, dest_id=nothing, apply_changes=false)
    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[remote ? 0x17 : 0x08, frame_id, at_bytes..., value...]
    if remote
        push!(data, apply_changes ? 0x02 : 0x00)
    end
#   if apply_changes && !remote
#       append!(data, 0x02)  # Apply (for remote, or local WR implied)
#   end
    escaped_data = UInt8[]
    for b in data
        if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
            push!(escaped_data, 0x7D)
            push!(escaped_data, b ⊻ 0x20)
        else
            push!(escaped_data, b)
        end
    end
    api_length = hton(UInt16(length(escaped_data)))
    checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
    frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]
    node = lattice_dict[node_id]
    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed")
    end
    resp = query_at(node_id, at_cmd; timeout_sec, api_mode, remote, dest_id)
    if apply_changes && !remote
        set_at(node_id, "WR", UInt8[]; timeout_sec, api_mode, remote, dest_id)  # Save
    end
    return resp
end



#=
function set_at(node_id::Int, at_cmd::String, value::Vector{UInt8}; api_mode=2, remote=false, dest_id=nothing, apply_changes=false)
    node = lattice_dict[node_id]
    if remote
        if isnothing(dest_id)
            error("dest_id required")
        end
        dest_node = lattice_dict[dest_id]
        dest_mac_bytes = hex2bytes(dest_node.mac)
    end

    frame_id = 0x01::UInt8
    at_bytes = Vector{UInt8}(codeunits(at_cmd))
    data = UInt8[remote ? 0x17 : 0x08, frame_id]
    if remote
        append!(data, dest_mac_bytes)
        append!(data, [0xFF, 0xFE])
        push!(data, apply_changes ? 0x02 : 0x00)  # Apply or just queue
    end
    append!(data, at_bytes)
    append!(data, value)

    # Escape, length, checksum, write as in query_at

    res = rf_write_packet(node.handle, frame)
    if res != 0
        error("Write failed")
    end

    # Use query_at logic for resp, but since set, value empty on success
    resp = query_at(node_id, at_cmd; timeout_sec=5.0, remote=remote, dest_id=dest_id)
    return resp  # [] on success
end
=#

#Insane debug master function
function query_atm(node_id::Int, at_cmd::String; timeout_sec=10.0, api_mode=2, remote=false, dest_id=nothing)
    node = lattice_dict[node_id] # Sender
    if remote
        if isnothing(dest_id)
            error("dest_id required for remote")
            end
            dest_node = lattice_dict[dest_id]
            if isempty(dest_node.mac)
                error("MAC not set for dest_id $dest_id")
                end
                dest_mac_bytes = hex2bytes(dest_node.mac) # 8B
            end
            frame_id = 0x01::UInt8
            at_bytes = Vector{UInt8}(codeunits(at_cmd))
            data = UInt8[remote ? 0x17 : 0x08, frame_id]
            if remote
                append!(data, dest_mac_bytes)
                append!(data, [0xFF, 0xFE]) # Unknown net addr
                push!(data, 0x00) # Options (0x02 for set apply)
            end
            append!(data, at_bytes) # No value for query
            escaped_data = UInt8[]
            for b in data
                if api_mode == 2 && b in [0x7E, 0x7D, 0x11, 0x13]
                    push!(escaped_data, 0x7D)
                    push!(escaped_data, b ⊻ 0x20)
                else
                    push!(escaped_data, b)
                end
            end
            api_length = hton(UInt16(length(escaped_data)))
            checksum = UInt8(0xFF - (sum(escaped_data) % 0x100))
            frame = UInt8[0x7E, reinterpret(UInt8, [api_length])..., escaped_data..., checksum]
            res = rf_write_packet(node.handle, frame)
            if res != 0
                error("Write failed for AT $at_cmd on node $node_id")
                end

                # Separate poll for TX status 0x8B
                tx_start = time()
                tx_buf = Vector{UInt8}()
                while time() - tx_start < 5.0
                    temp_buf = Vector{UInt8}(undef, 512)
                    bytes_read = rf_read_packet(node.handle, temp_buf, 512)
                    if bytes_read > 0
                        println("TX Raw bytes read: $(bytes2hex(temp_buf[1:bytes_read]))")  # Debug
                        append!(tx_buf, temp_buf[1:bytes_read])
                    end
                    i = 1
                    while i <= length(tx_buf)
                        if tx_buf[i] == 0x7E
                            len_bytes, new_i = unescape_n(tx_buf, i+1, 2, api_mode)
                            if isnothing(len_bytes)
                                break
                            end
                            len = ntoh(reinterpret(UInt16, len_bytes)[1])
                            i = new_i
                            unescaped_data, new_i = unescape_n(tx_buf, i, len, api_mode)
                            if isnothing(unescaped_data)
                                break
                            end
                            i = new_i
                            checksum_bytes, new_i = unescape_n(tx_buf, i, 1, api_mode)
                            if isnothing(checksum_bytes)
                                break
                            end
                            checksum_received = checksum_bytes[1]
                            i = new_i
                            checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                            if checksum_received == checksum_calc && unescaped_data[1] == 0x8B && unescaped_data[2] == frame_id
                                status = unescaped_data[7]
                                println("TX Status: 0x$(string(status, base=16))")
                                tx_buf = tx_buf[i:end]  # Consume
                                break
                            end
                            i += 1
                        end
                    end
                    tx_buf = tx_buf[i:end]
                    sleep(0.05)
                end

                # Main loop for 0x97/0x88
                buf = Vector{UInt8}()
                start_time = time()
                while time() - start_time < timeout_sec
                    temp_buf = Vector{UInt8}(undef, 512)
                    bytes_read = rf_read_packet(node.handle, temp_buf, 512)
                    if bytes_read > 0
                        println("Raw bytes read: $(bytes2hex(temp_buf[1:bytes_read]))")
                        append!(buf, temp_buf[1:bytes_read])
                    end
                    if length(buf) > 1024  # Safety
                        println("Buf overflow clear")
                        buf = []
                    end
                    i = 1
                    while i <= length(buf)
                        if buf[i] == 0x7E
                            len_bytes, new_i = unescape_n(buf, i+1, 2, api_mode)
                            if isnothing(len_bytes)
                                break
                            end
                            len = ntoh(reinterpret(UInt16, len_bytes)[1])
                            if len > 256 || len < 4  # Safety
                                i += 1
                                continue
                            end
                            i = new_i
                            unescaped_data, new_i = unescape_n(buf, i, len, api_mode)
                            if isnothing(unescaped_data)
                                break
                            end
                            i = new_i
                            checksum_bytes, new_i = unescape_n(buf, i, 1, api_mode)
                            if isnothing(checksum_bytes)
                                break
                            end
                            checksum_received = checksum_bytes[1]
                            i = new_i
                            checksum_calc = UInt8(0xFF - (sum(unescaped_data) % 0x100))
                            if checksum_received != checksum_calc
                                println("Bad checksum skip")
                                i += 1
                                continue
                            end

                            if unescaped_data[1] == (remote ? 0x97 : 0x88) && unescaped_data[2] == frame_id
                                at_start = remote ? 3 + 8 + 2 : 3
                                if unescaped_data[at_start:at_start + length(at_bytes) - 1] == at_bytes
                                    status_pos = at_start + length(at_bytes)
                                    status = unescaped_data[status_pos]
                                    if status == 0x00
                                        value = unescaped_data[status_pos + 1:end]
                                        return value
                                    else
                                        error("AT status error: 0x$(string(status, base=16))")
                                    end
                                end
                            end
                        else
                              println("Unexpected frame type: 0x$(string(unescaped_data[1], base=16)) - Unescaped: $(bytes2hex(unescaped_data))")
                            i += 1
                        end
                    end
                    println("Unescaped: $(bytes2hex(unescaped_data))")
                    buf = buf[i:end]
                    sleep(0.05)
                end
                error("Timeout on AT $at_cmd for node $node_id")
                end
