module PacketUtils
using ProtoBuf
include("RFpacket_pb.jl")  # Adjust if path differs
using .RFpacket_pb
using Base: time_ns  # For timestamps
using CRC  # Add Pkg.add("CRC") if needed for real crc32

export create_raw_packet, encode_packet, decode_packet, compute_crc32

function current_timestamp_us()
    time_ns() ÷ 1000  # ns to us
end

function compute_crc32(data::Vector{UInt8})
    crc(CRC_32_CRC32, data)  # Real CRC; fallback to 0xDEADBEEF if testing
end

function create_raw_packet(antennas::Vector{RFpacket_pb.AntennaData}; stream_id=1, sequence=1, flags=0, sync_marker=[0x1A, 0xCF, 0xFC, 0x1D])
    hdr = RFpacket_pb.PacketHeader(
        version=1,
        stream_id=stream_id,
        sequence=sequence,
        timestamp_us=current_timestamp_us(),
        flags=flags,
        sync_marker=Vector{UInt8}(sync_marker)
    )
    raw = RFpacket_pb.RawSample(antennas=antennas)
    pkt = RFpacket_pb.SignalPacket(
        header=hdr,
        kind=RFpacket_pb.PayloadKind.RAW,
        payload=OneOf(:raw, raw),
        crc32=0  # Update after encode
    )
    return pkt
end

function encode_packet(pkt::RFpacket_pb.SignalPacket)
    io = IOBuffer()
    writeproto(io, pkt)
    bytes = take!(io)
    pkt.crc32 = compute_crc32(bytes)  # Set CRC post-encode (checksum the payload/header)
    # Re-encode with CRC (optional if receiver computes it)
    seekstart(io)
    writeproto(io, pkt)
    return take!(io)
end

function decode_packet(bytes::Vector{UInt8})
    io = IOBuffer(bytes)
    pkt = readproto(io, RFpacket_pb.SignalPacket())
    # Verify CRC (optional: if compute_crc32(bytes[1:end-4]) != pkt.crc32, throw error)
    return pkt
end

# Example: Extend for other kinds (e.g., filtered from mekf output)
function create_filtered_packet(state::Vector{Float64}, cov_flat::Vector{Float64}; model_version=1, param_hash=UInt8[])
    # Stub: Convert Float64 to Float32 if needed for proto
    #hdr = RFpacket_pb.PacketHeader(...)  # Same as above
    filtered = RFpacket_pb.FilteredState(x=Float32.(state), P_flat=Float32.(cov_flat), model_version=model_version, param_hash=param_hash)
    pkt = RFpacket_pb.SignalPacket(header=hdr, kind=RFpacket_pb.PayloadKind.FILTERED, payload=OneOf(:filtered, filtered), crc32=0)
    return pkt
end

# Quick test block (comment out for prod)
if abspath(PROGRAM_FILE) == @__FILE__
    ant1 = RFpacket_pb.AntennaData(name="LatticeNode1", rss=-65.0, tdoa=[0.0, 1.2e-6], aoa=[0.0, 90.0, 0.0], units="dBm/us/deg")
    ant2 = RFpacket_pb.AntennaData(name="LatticeNode2", rss=-70.0, tdoa=[1.5e-6], aoa=[45.0, 0.0, 0.0], units="dBm/us/deg")
    pkt = create_raw_packet([ant1, ant2])

    bytes = encode_packet(pkt)
    println("Encoded bytes length: ", length(bytes))

    decoded = decode_packet(bytes)
    println("Decoded first antenna name: ", decoded.payload[].antennas[1].name)
    println("Decoded timestamp: ", decoded.header.timestamp_us)
end

end  # module