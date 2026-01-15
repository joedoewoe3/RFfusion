

function send_packet_udp(bytes::Vector{UInt8}, target_ip::String, target_port::Int)
    sock = UDPSocket()
    try
        send(sock, parse(IPv4, target_ip), target_port, bytes)
    finally
        close(sock)
    end
end

function send_packet_udp(packet::RFpacket_pb.RFPacket, target_ip::String, target_port::Int; use_crc=false, use_frame=false)
    bytes = encode_packet(packet, use_crc=use_crc)  # From PacketUtils—bytes blob

    if use_frame
        bytes = xbee_api_frame(bytes)  # Optional wrap for XBee API if proxy
    end

    send_packet_udp(bytes, target_ip, target_port)  # Call bytes overload
    println("UDP Packet sent to $target_ip:$target_port: $(packet.node_label)")
end

function receive_packet(listen_port::Int; timeout_ms=5000)
    sock = UDPSocket()
    bind(sock, IPv4(0), listen_port)
    try
        # Wait for data with timeout
        ready = wait_fd(sock.fd, readable=true, timeout=timeout_ms/1000)
        if ready
            bytes, src = recvfrom(sock)
            return decode_packet(bytes), src
        else
            error("Timeout waiting for packet")
        end
    finally
        close(sock)
    end
end


    # Receive from UDP (non-blocking, with timeout)
function receive_packet_udp(port::Int; max_len=1200, timeout_sec=5.0, use_crc=false, use_blob_crc=false)
    sock = UDPSocket()
    bind(sock, IPv4(0), port) || error("UDP bind failed on port $port")

    start_time = time()
    bytes = nothing

    while isnothing(bytes) && (time() - start_time < timeout_sec)
        if isreadable(sock)
            bytes = recv(sock)
            break
        end
        sleep(0.01)
    end

    close(sock)

    if isnothing(bytes)
            error("Timeout receiving UDP on port $port after $timeout_sec sec")
    end

    # Assume bytes are full packet (no frame for UDP—add if needed)
    pkt = decode_packet(bytes; use_crc=use_crc, use_blob_crc=use_blob_crc)
    println("Raw UDP packet received on port $port: $(pkt.node_label)")

    return pkt
end



function discover_antennas(broadcast_port::Int=5000, timeout::Float64=5.0)
    # Create IPv6 UDP socket
    sock = UDPSocket()

    # Get Ptr UVHandle and ccall uv_fileno for Int fd
    uv_handle = sock.handle.handle  # Ptr{Cvoid} to UVHandle
    fd = ccall((:uv_fileno, Base.Libdl.dlsym(Base.libuv_jl_so, :uv_fileno)), Cint, (Ptr{Cvoid},), uv_handle)

    # Set non-blocking mode via ccall to fcntl
    flags = ccall((:fcntl, Libc.libc), Cint, (Cint, Cint), fd, F_GETFL)
    ccall((:fcntl, Libc.libc), Cint, (Cint, Cint, Cint), fd, F_SETFL, flags | O_NONBLOCK)

    # Join multicast group for discovery (all-nodes link-local)
    join_multicast_group(sock, IPv6("ff02::1"))

    # Broadcast discovery message ("ANTENNA_PING")
    message = "ANTENNA_PING"
    broadcast_addr = IPv6("ff02::1")
    send(sock, broadcast_addr, broadcast_port, message)

    discovered_ips = String[]
    start_time = time()
    while time() - start_time < timeout
        try
            data, addr = recvfrom(sock)
            if data == "ANTENNA_PONG"  # Nodes respond with pong
                ip_str = string(addr.host)  # Grab IPv6 string
                if !(ip_str in discovered_ips)  # Dedup
                    push!(discovered_ips, ip_str)
                    println("Discovered antenna at $ip_str")
                end
            end
            catch e
            if !(e isa UVError && e.code == UV_EAGAIN)
                rethrow()  # Rethrow non-wouldblock errors
            end
        end
        sleep(0.01)  # Light poll to avoid CPU spin
    end

    leave_multicast_group(sock, IPv6("ff02::1"))
    close(sock)
    return discovered_ips
end
# Run it and print IPv6 addrs for your 6 antennas

#=

    # Overload for raw bytes (no encode—direct UDP send)
function send_packet_udp(bytes::Vector{UInt8}, target_ip::String, target_port::Int; use_crc=false, use_frame=false)
        if use_frame
            bytes = xbee_api_frame(bytes)  # Optional wrap
        end

        # No need for use_crc here (already appended if from encode)
        sock = UDPSocket()
        try
            send(sock, parse(IPv4, target_ip), target_port, bytes)
        finally
            close(sock)
        end
        println("Raw bytes sent to $target_ip:$target_port")
end

# Threaded listener for continuous receive (e.g., from antennas)
function start_listener(listen_port::Int, callback::Function)
    Threads.@spawn begin
        while true
            pkt, src = receive_packet(listen_port)
            callback(pkt, src)  # e.g., feed to mekf
        end
    end
end
=#

#=
struct IPv6 <: IPAddr
host::UInt128
IPv6(host::UInt128) = new(host)
IPv6(a::UInt16,b::UInt16,c::UInt16,d::UInt16,
e::UInt16,f::UInt16,g::UInt16,h::UInt16) = new(UInt128(a)<<(7*16)|
UInt128(b)<<(6*16)|
UInt128(c)<<(5*16)|
UInt128(d)<<(4*16)|
UInt128(e)<<(3*16)|
UInt128(f)<<(2*16)|
UInt128(g)<<(1*16)|
h)
function IPv6(a::Integer,b::Integer,c::Integer,d::Integer,
e::Integer,f::Integer,g::Integer,h::Integer)
if !(0<=a<=0xFFFF && 0<=b<=0xFFFF && 0<=c<=0xFFFF && 0<=d<=0xFFFF &&
0<=e<=0xFFFF && 0<=f<=0xFFFF && 0<=g<=0xFFFF && 0<=h<=0xFFFF)
throw(ArgumentError("IPv6 field out of range (must be 0-65535)"))
end
IPv6(UInt16(a),UInt16(b),UInt16(c),UInt16(d),
UInt16(e),UInt16(f),UInt16(g),UInt16(h))
end
end
=#
function send_packet(node_id::Int, packet::RFpacket_pb.RFPacket; use_udp=false)
    label = use_udp ? "udp_latticenode_$node_id" : "serial_latticenode_$node_id"
    node = lattice_dict[label]
    node = open_node(node)
    lattice_dict[label] = node

    bytes = encode_packet(packet)

    if use_udp
        send_packet_udp(bytes, node.address, node.port_baud)
    else
        framed_bytes = xbee_api_frame_wrap(bytes; dest_addr=0x000000000000FFFF)
        rf_write_packet(node.handle, framed_bytes)
    end
    println("Packet sent to $label: $(packet.node_label)")
end
