function current_timestamp_us()
    time_ns() ÷ 1000  # ns to us
end

function compute_crc32(data::Vector{UInt8})
    crc_func = crc(CRC_32_C)  # Returns a function f(data) → UInt32
    return crc_func(data)     # Or directly: crc(CRC_32_C)(data)
end
#Nova mentioned that create_filtered_packet and create_raw_packet are good stubs but make_header is good if I eradicate those two functions. I'll keep the thought in mind

function create_filtered_packet(state::Vector{Float64}, cov_flat::Vector{Float64}, cr_bounds::Vector{Float64}; model_version=1, header=PacketHeader(UInt32(1), UInt32(1), UInt32(0), UInt64(0), UInt32(0), UInt8[]), label="FusedState", msg="E8/CR Update")
    #pack data to bytes: Flat [state; covariance_flat; Cramer-Rao_bound] as doubles ^^or in any form we wish
    fused_data = vcat(state, cov_flat, cr_bounds) #vector of 3
    rf_blob = reinterpret(UInt8, fused_data) #make needed byte array for sendoff. total_bytes = length(rf_blob)*8)

    #TODO: Calc layout offsets for parse state_length = length(state)*8 bytes. minding 32 bit for GPU, or BigInt for primes whatever
    pkt = RFPacket(
        header,
        label,    #LatticeNodeX
        msg,      #Init Fusion or perhapes `$("node_id—code")`
        rf_blob,  #fused bytes
        UInt8[],  #Firmware space for O TA
        UInt8[],  #An empty Spectrum Domain
        )
    return pkt
end

# In PacketUtils.jl (simple .proto version—bytes blob for AOA/TDOA/RSS)
function make_header(stream_id::UInt32=UInt32(1), sequence::UInt32=UInt32(0), flags::UInt32=UInt32(0), sync_marker::Vector{UInt8}=UInt8[0x1A, 0xCF, 0xFC, 0x1D])
    now_us = UInt64(floor(datetime2unix(now()) * 1e6))  # Real microsecs; or fixed for tests
    PacketHeader(
        UInt32(1),  # version
        stream_id,
        sequence,
        now_us,
        flags,
        sync_marker  # 4 bytes; add more for robustness if RF noisy
        )
end

function create_raw_packet(aoa::Vector{Float64}, tdoa::Vector{Float64}, rss::Vector{Float64}; stream_id=1, sequence=0, flags=0, label="RawTriality", msg="AOA/TDOA/RSS Data", model_version=1)
    fused_data = vcat(aoa, tdoa, rss)  # Flat Vector{Float64} (e.g., [aoa1, aoa2, tdoa1, ..., rss1, ...])
    rf_blob = reinterpret(UInt8, fused_data)  # Bytes pack—length = length(fused_data)*8

    header = make_header(UInt32(stream_id), UInt32(sequence), UInt32(flags))  # Filled header

    pkt = RFPacket(
        header,
        label,
        msg,
        rf_blob,  # Packed raw bytes
        UInt8[],  # Firmware empty
        UInt8[]   # Spectrum empty (or pack similar if needed)
        )
    return pkt
end

# encode_packet using gigantic numbers ....orCRC32.


#=
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
=#
# In PacketUtils.jl (tweak for simple .proto)
#=
function create_raw_packet(aoa::Vector{Float64}, tdoa::Vector{Float64}, rss::Vector{Float64}; RFpacket_pb.RFPacket)
    rf_blob = reinterpret(UInt8, [aoa; tdoa; rss])  # Flat pack doubles
    pkt = RFPacket(header=RFpacket_pb.PacketHeader, rf_packet=rf_blob, ...)
    return pkt
end

=#
# encode_packet stays same (handles bytes auto)

function encode_packet(pkt::RFpacket_pb.RFPacket; use_crc=false, use_blob_crc=false)
    if use_blob_crc
        blob_crc = compute_crc32(pkt.rf_packet)
        crc_bytes = reinterpret(UInt8, [hton(UInt32(blob_crc))]) # Big-end 4 bytes

        # Create new pkt with updated spectrum_bytes (immutable fix—copy fields)
        pkt = RFPacket(
            pkt.header,
            pkt.node_label,
            pkt.status_msg,
            pkt.rf_packet,
            pkt.firmware_update,
            vcat(pkt.spectrum_bytes, crc_bytes) # Append to spectrum for blob check
            )
    end

    io = IOBuffer()
    encode(ProtoEncoder(io), pkt) # Gen'd encode to io (use PB.encode if undef)
    bytes = take!(io) # Vector{UInt8}

    if use_crc
        crc_val = compute_crc32(bytes)
        crc_bytes = reinterpret(UInt8, [hton(UInt32(crc_val))]) # Big-end 4 bytes
        bytes = vcat(bytes, crc_bytes) # Append for full packet check
    end

    return bytes
end

function decode_packet(bytes::Vector{UInt8}; use_crc=false, use_blob_crc=false)
    if use_crc
        if length(bytes) < 4
            error("Bytes too short for CRC")
            end
            crc_received = ntoh(reinterpret(UInt32, bytes[end-3:end])[1])
            crc_calc = compute_crc32(bytes[1:end-4])
            if crc_received != crc_calc
                error("CRC mismatch—packet corrupt")
            end
            bytes = bytes[1:end-4]  # Strip CRC
        end

        io = IOBuffer(bytes)
        pkt = decode(ProtoDecoder(io), RFPacket)  # Gen'd decode

        if use_blob_crc
            if length(pkt.spectrum_bytes) < 4
                error("Spectrum too short for blob CRC")
                end
                blob_crc_received = ntoh(reinterpret(UInt32, pkt.spectrum_bytes[end-3:end])[1])
                blob_crc_calc = compute_crc32(pkt.rf_packet)
                if blob_crc_received != blob_crc_calc
                    error("Blob CRC mismatch—fused data corrupt")
                end
                pkt.spectrum_bytes = pkt.spectrum_bytes[1:end-4]  # Strip CRC (immutable, but since returning pkt, okay for read-only)
            end
    return pkt
end

#=
function encode_packet(pkt::RFpacket_pb.RFPacket; use_crc=false)
    io = IOBuffer()
    encode(ProtoEncoder(io), pkt)  # Gen'd encode to io
    bytes = take!(io)  # Vector{UInt8}

    if use_crc
        crc_val = compute_crc32(bytes)
        crc_bytes = reinterpret(UInt8, [hton(UInt32(crc_val))])  # Big-end 4 bytes
        bytes = vcat(bytes, crc_bytes)  # Append for decode check
    end

    return bytes
end

function decode_packet(bytes::Vector{UInt8}; use_crc=false)
    if use_crc
        if length(bytes) < 4
            error("Bytes too short for CRC")
            end
            crc_received = ntoh(reinterpret(UInt32, bytes[end-3:end])[1])
            crc_calc = compute_crc32(bytes[1:end-4])
            if crc_received != crc_calc
                error("CRC mismatch—packet corrupt")
            end
            bytes = bytes[1:end-4]  # Strip CRC
        end

        io = IOBuffer(bytes)
        pkt = decode(ProtoDecoder(io), RFPacket)  # Gen'd decode
        return pkt
    end
=#


#=
# Example: Extend for other kinds (e.g., filtered from mekf output)
function create_filtered_packet(state::Vector{Float64}, cov_flat::Vector{Float64}; model_version=1, param_hash=UInt8[])
    # Stub: Convert Float64 to Float32 if needed for proto
    #hdr = RFpacket_pb.PacketHeader(...)  # Same as above
    filtered = RFpacket_pb.FilteredState(x=Float32.(state), P_flat=Float32.(cov_flat), model_version=model_version, param_hash=param_hash)
    pkt = RFpacket_pb.SignalPacket(header=hdr, kind=RFpacket_pb.PayloadKind.FILTERED, payload=OneOf(:filtered, filtered), crc32=0)
    return pkt
end
=#





