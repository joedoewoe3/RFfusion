using Base.Filesystem
dev_dir = "/dev"
tty_ports = filter(x -> occursin(r"^tty(USB|ACM)", x), readdir(dev_dir))
full_ports = ["/dev/$p" for p in sort(tty_ports)]

using SerialPorts # the goal is to say goodbye to this

struct LatticeNode
    id::Int #simple
    port::String #"COM" or "/dev/ttyUSB0"
    baud::Int
    proto_config::Dict{String, Any}  #we have our utf8 and our bytes fields
    handle::Union{SerialPort, Ptr{Cvoid}}
end

function open_node(node::LatticeNode)
    if node.handle isa Ptr{Cvoid}  # If using C wrapper
        # ccall(:your_c_open_func, Ptr{Cvoid}, (Cstring, Cint), node.port, node.baud)
    else
        sp = SerialPort(node.port, node.baud)
        open(sp)
        return LatticeNode(node.id, node.port, node.baud, node.proto_config, sp)
    end
end

#First bring in c library

# Load your C lib (assume compiled to libRFWrapper.so in project dir or LD_LIBRARY_PATH)
const LIB_RF = joinpath(@__DIR__,"c", "libRFWrapper.so")
Libc.Libdl.dlopen(LIB_RF)  # Preload to validate

# Your RFPacket.proto defaults (tweak to match exact fields)
default_proto = Dict(
    "utf8_fields" => ["node_label", "timestamp", "status_msg"],  # Structured UTF-8 strings
    "bytes_fields" => ["rf_packet", "firmware_update", "spectrum_bytes"],  # Raw binary bytes
    "encoding" => "proto3"  # For ref; C wrapper might enforce
    )

# Build the dict
lattice_dict = Dict{Int, LatticeNode}()
for (i, port) in enumerate(full_ports)  # i=1 to 6, ports like "/dev/ttyUSB0"
    # Handle starts as C_NULL; open later with ccall
    node = LatticeNode(i, port, 115200, default_proto, Ptr{Cvoid}(0))
    lattice_dict[i] = node
end

println("Lattice Dict Setup: ", lattice_dict)  # Inspect; should show 6 nodes with null handles

#Julia proto RF packet now we are doing combo work

using ProtoBuf
include(joinpath(@__DIR__, "src", "RFpacket_pb.jl"))  # Adjust if path differs
using .RFpacket_pb

const rf_open = (port::String, baud::Int) -> ccall((:rf_open, LIB_RF), Ptr{Cvoid}, (Cstring, Cint), port, baud)
const rf_write_packet = (handle::Ptr{Cvoid}, data::Vector{UInt8}) -> ccall((:rf_write_packet, LIB_RF), Cint, (Ptr{Cvoid}, Ptr{UInt8}, Csize_t), handle, data, length(data))
const rf_close = (handle::Ptr{Cvoid}) -> ccall((:rf_close, LIB_RF), Cvoid, (Ptr{Cvoid},), handle)

# Open node handle via C (updates struct—reassign to dict if needed)
function open_node(node::LatticeNode)
    if node.handle == Ptr{Cvoid}(0)  # Not open
        handle = rf_open(node.port, node.baud)
        if handle == Ptr{Cvoid}(0)
            error("C open failed on $(node.port)—check dmesg or baud mismatch!")
        end
        return LatticeNode(node.id, node.port, node.baud, node.proto_config, handle)
    end
    return node
end

# Close for cleanup
function close_node(node::LatticeNode)
    if node.handle != Ptr{Cvoid}(0)
        rf_close(node.handle)
    end
end


# Send a proto-encoded packet (Julia encodes UTF-8/bytes per proto, C writes raw bytes)
function send_packet(node_id::Int, packet::RFPacket)  # Tweak RFpacket to your gen'd struct name
    node = lattice_dict[node_id]
    node = open_node(node)  # Open if needed
    lattice_dict[node_id] = node

    # Encode (ProtoBuf auto-handles UTF-8 strings vs. bytes fields)
    encoded_io = IOBuffer()
    PacketUtils.encode_packet(encoded_io, packet)   ## IMPORTANT SHIT THIS IS
    bytes = take!(encoded_io)

    # Right here: bytes shell for AP frames or whatever low-level (e.g., if DIGI needs headers/footers)
    # Example: header_bytes = UInt8[0xAA, 0xBB] (sync/start); footer_bytes = UInt8[0xCC, 0xDD] (checksum/end)
    # framed_bytes = [header_bytes; bytes; footer_bytes]
    # For now, use raw bytes; tweak if comms fail
    framed_bytes = bytes  # Placeholder—expand for your flow
    res = rf_write_packet(node.handle, framed_bytes)
    if res != 0
        error("C write failed for node $node_id code $res; check antenna response")
        end
        println("Packet sent to LatticeNode $node_id: $(packet.node_label)")
    end


    using Dates  # For timestamp calc

    # Calc example timestamp_us (microsec since UNIX epoch; tweak for your sync)
    #now_us = UInt64(floor(Dates.datetime2unix(Dates.now()) * 1e6))  # ~1.704e15 for 2026-ish
    dt = DateTime(2026, 1, 3, 12, 0, 0)  # Adjust hour/min/sec for "now"
    now_unix = Dates.datetime2unix(dt)
    now_us = UInt64(floor(now_unix * 1e6))  # Microsecs

    # Positional constructor (order: version, stream_id, sequence, timestamp_us, flags, sync_marker)
    test_header = PacketHeader(
        UInt32(1),
        UInt32(1),  # Group for nodes 1-3
        UInt32(0),  # Sequence start
        now_us,
        UInt32(0),  # No flags
        UInt8[0x1A, 0xCF, 0xFC, 0x1D]  # Sync pattern
        )

    test_packet = RFPacket(
        test_header,                  # header
        "LatticeNode1-Test",          # node_label (UTF-8 string)
        "Init OTA - Triality Test",   # status_msg (UTF-8)
        UInt8[0x01, 0x02, 0x03],      # rf_packet (bytes, e.g., dummy AOA/TDOA/RSS blob)
        UInt8[],                      # firmware_update (empty bytes)
        UInt8[0xAA, 0xBB, 0xCC]       # spectrum_bytes (bytes)
        # If adding: aoa_deg=45.0, tdoa_ns=1234, rss_dbm=-60 (uncomment in .proto first)
        )

   send_packet(1, test_packet) # Repeated AntennaData

   for id in 1:6
   send_packet(id, test_packet)
end
for id in 1:6
    close_node(lattice_dict[id])
    lattice_dict[id] = LatticeNode(lattice_dict[id].id, lattice_dict[id].port, lattice_dict[id].baud, lattice_dict[id].proto_config, Ptr{Cvoid}(0))  # Reset
end



    # Full SignalPacket (header + kind + oneof payload + crc32)
 #   test_packet = SignalPacket(
  #      test_header,  # Header
   #     PayloadKind.RAW,  # Enum (0 for RAW)
    #    UInt32(0)  # crc32 placeholder (calc real via hash or zlib.crc32(bytes) if needed)
     #   )
    #test_packet.raw = test_raw  # Set the oneof (ProtoBuf allows post-construct)

    # Now send it (from Step 4 func)
    #send_packet(1, test_packet)  # To node 1; watch for success println
