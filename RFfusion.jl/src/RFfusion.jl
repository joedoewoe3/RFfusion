module RFfusion

#__precompile__(false)


using ProtoBuf
using Libdl  # For lib loading checks
using CRC
using LinearAlgebra
using LinearAlgebra: pinv
#using LinearAlgebra: diagm
using Manifolds # For manifolds mode
using Rotations # For manifolds mode
using Octonions # For octonions mode
using Quaternions # For embeds
using Random # For randn
using Plots # For viz
using Dates
using SerialPorts # TODO: Ditch SerialPorts.jl as it uses a lot of slow python. Use our c wrapper instead
using Sockets
using Printf
using Base: Libc

Base.getindex(o::OctonionF64, i::Int) = getproperty(o, [:s, :v1, :v2, :v3, :v4, :v5, :v6, :v7][i])
Base.getindex(o::OctonionF64, r::UnitRange) = [o[i] for i in r]
function to_vec(o::OctonionF64)
       [o.s, o.v1, o.v2, o.v3, o.v4, o.v5, o.v6, o.v7]
end

include("RFpacket_pb.jl")
include("latticenode.jl")  # Your serial/lattice core
include("ApiFrames.jl") # after latticenode for serial
include("PacketUtils.jl")
include("ArraySpeak.jl")
include("RFfusionCore.jl")  # Fusion sims/filter (if separate; merge if small)

using .RFpacket_pb


# Public API
export send_packet_serial, receive_packet_serial, open_node, close_node, lattice_dict, query_at, auto_fill_lattice_dict!, unescape_n, set_at, query_atm # init_lattice_dict  From latticenode.jl
export create_raw_packet, create_filtered_packet, encode_packet, decode_packet, compute_crc32, make_header,  receive_data_ap_two, send_data_ap_two   # From PacketUtils.jl
export send_packet_udp, receive_packet_udp, start_listener, discover_antennas #From ArraySpeak which will deal with UDP communications
export joseph_filter, sensor_fusion_ring_mekf, simulate_fusion, project_to_6d, e8_root_lattice, pad_to_oct, create_fused_packet # , process_incoming   From RFfusionCore.jl
export xbee_api_frame_wrap, parse_xbee_frame #ApiFrames.jl

# Module Init (runs on using RFfusion—opens nodes auto)


function __init__()
    for id in keys(lattice_dict)
        lattice_dict[id] = open_node(lattice_dict[id])
        sh_bytes = query_at(id, "SH")
        sl_bytes = query_at(id, "SL")
        mac = bytes2hex([sh_bytes; sl_bytes])
        node = lattice_dict[id]
        new_node = LatticeNode(node.id, node.port, node.baud, node.proto_config, node.handle, mac)
        lattice_dict[id] = new_node
    end
    println("Lattice nodes opened.")
end
#=
function __init__()
    global lattice_dict = init_lattice_dict()
    for (label, node) in lattice_dict
        lattice_dict[label] = open_node(node)  # Open serial or UDP
    end
    println("Lattice nodes opened (serial/udp): ", lattice_dict)
end
=#

end # module RFfusion
