using RFfusion, Octonions, LinearAlgebra #, Plots   Add Plots for viz

# Params: SO(8) dim=8, oct mode
dim, mode = 8, "e8"
Q, R = diagm(ones(dim)), diagm(ones(dim))  # Noise covs (8x8)
#state_prev = OctonionF64(zeros(8)...)  # Single oct: real=0, v1-7=0
P_prev = diagm(ones(dim))  # 8x8 cov
ω, dt = randn(3), 1e-3  # Ang vel, timestep
v_pred = [0.0, 0.0, -9.80665, zeros(dim-3)...]
v_meas =  [0.0, 0.0, -9.80665, zeros(dim-3)...]
state_prev = OctonionF64(zeros(8)...)

state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode)

pkt = RFfusion.create_fused_packet(state_new, vec(P_new), vec(CR))

send_packet_serial(1, pkt)
function send_data_ap_two(1, dest_id::Int, packet::RFpacket_pb.RFPacket; api_mode=2, broadcast=false)

raw_pkt = receive_packet_serial(1; timeout_sec=2.0)  # Or UDP variant

ips = discover_antennas()
println("Grabbed IPv6 IPs: ", ips)

#true_state = OctonionF64(randn(8)...)  # For err sim


# Magnetism: AOA as cross in imag v1-3
function magnetism_update(o::OctonionF64, aoa::Vector{Float64})
    B = [o.v1, o.v2, o.v3]  # B-field
    rot = cross(B, aoa)     # Lorentz twist
    return OctonionF64(o.s, rot[1], rot[2], rot[3], o.v4, o.v5, o.v6, o.v7)
end

# Pad 3D/6D to oct (embed RSS/AOA/TDOA)
function pad_to_oct(v::Vector{Float64})
    len = length(v)
    if len == 3  # e.g., AOA
        return OctonionF64(0.0, v[1], v[2], v[3], 0.0, 0.0, 0.0, 0.0)
        elseif len == 6  # pos+vel
        return OctonionF64(v[1], v[2], v[3], v[4], v[5], v[6], 0.0, 0.0)  # Adjust as needed
    else  # Full 8
        return OctonionF64(v...)
    end
end

start_listener(12345, process_incoming)  # UDP for incoming

states, Ps, CRs = OctonionF64[], Matrix{Float64}[], Matrix{Float64}[]  # Collect for viz
for step in 1:100
    # Stub meas/pred: Real from incoming pkt (flat vectors)
    v_meas_flat, v_pred_flat = randn(8), randn(8)
    v_meas = pad_to_oct(v_meas_flat)
    v_pred = pad_to_oct(v_pred_flat)

    # Fusion: Get new state (oct), P, CR
    state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode)

    # Magnetism: Embed AOA (stub randn(3); real from pkt)
    state_new = magnetism_update(state_new, randn(3))

    # Joseph: Stabilized update (impl in RFfusionCore.jl)
    filtered = joseph_filter(state_new)  # Assume returns oct

    # Packet: Flat state to vector for proto
    state_flat = [filtered.s, filtered.v1, filtered.v2, filtered.v3, filtered.v4, filtered.v5, filtered.v6, filtered.v7]
    pkt = create_filtered_packet(state_flat, vec(P_new), vec(CR); label="SO8Fusion", msg="Magnetism Update")
    send_packet(1, pkt)  # To node 1

    # Collect
    push!(states, state_new)
    push!(Ps, P_new)
    push!(CRs, CR)

    state_prev, P_prev = state_new, P_new
    sleep(dt)  # Real-time
end

# Viz: Fixed abs on single oct each
err_norms = [abs(state - true_state) for state in states]  # abs(o) works!
    p_traces = [tr(P) for P in Ps]  # Cov trace
        cr_traces = [tr(cr) for cr in CRs]  # CR trace
            p1 = plot(1:100, log10.(p_traces), label="P Trace", title="Fusion Traces")
            p2 = plot(1:100, err_norms, label="Err Norm")
            plot(p1, p2, layout=(2,1))
            savefig("fusion_viz.png")
