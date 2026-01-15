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
            node = LatticeNode(i, port, 115200, default_proto, Ptr{Cvoid}(0))
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

    function open_node(node::LatticeNode)
        if node.handle == Ptr{Cvoid}(0)  # Not open
            handle = rf_open(node.port, node.baud)
            if handle == Ptr{Cvoid}(0)
                error("C open failed on $(node.port)-check dmesg or baud mismatch!")
            end
            return LatticeNode(node.id, node.port, node.baud, node.proto_config, handle)
        end
        return node
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

                    # Full SignalPacket (header + kind + oneof payload + crc32)
                    #   test_packet = SignalPacket(
                    #      test_header,  # Header
                    #     PayloadKind.RAW,  # Enum (0 for RAW)
                    #    UInt32(0)  # crc32 placeholder (calc real via hash or zlib.crc32(bytes) if needed)
                    #   )
                    #test_packet.raw = test_raw  # Set the oneof (ProtoBuf allows post-construct)

                    # Now send it (from Step 4 func)
                    #send_packet(1, test_packet)  # To node 1; watch for success println

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
