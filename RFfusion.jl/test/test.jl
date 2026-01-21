# Dummy fusion inputs (tweak for your dim/mode)
ω = rand(3); dt = 0.1; state_prev = rand(QuatRotation); v_meas = rand(3); v_pred = rand(3); P_prev = Diagonal(rand(9)); Q = Diagonal(rand(9)); R = Diagonal(rand(3)); dim=3; mode="manifolds"; ip="127.0.0.1"; port=1234

# Fuse & send (calls mekf, creates/encodes pkt, UDP sends—epic loop close)
state_new, P_new, CR = fuse_and_send(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode, ip, port)

# For serial to antennas (integrate with latticenode): pkt = create_filtered_packet(...); latticenode.send_packet(1, pkt)


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
start_listener(1234, process_incoming)  # Port for incoming

# Sim update for E8 (dim=8, mode="e8")
# Test E8 sim (200 steps)
p_traces, cr_traces, err_norms = simulate_fusion(2000, 8, "e8")
println("Sample P traces: ", p_traces[1:5])



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
if abspath(PROGRAM_FILE) == @__FILE__
    # Dummy fused for filtered
    state = rand(8); cov_flat = vec(rand(8,8)); cr_bounds = rand(5)
    pkt = create_filtered_packet(state, cov_flat, cr_bounds)
    bytes = encode_packet(pkt, use_crc=true)
    println("Encoded bytes length: ", length(bytes))  # ~ (8+64+5)*8 + header ~600 + CRC 4

    decoded = decode_packet(bytes, use_crc=true)
    println("Decoded label: ", decoded.node_label)  # "FusedState"
    fused_back = reinterpret(Float64, decoded.rf_packet)  # Unpack to doubles
    println("Unpacked state length: ", length(fused_back))  # 8+64+5 = 77
end

using RFfusion  # Loads all; opens nodes

# Theorem: G = SO(3) × Aff(1)^k (RSS/TDOA/AOA fusion; dim=3+2k). Lie alg g=so(3)⊕aff(1)^k closes Ad_g(X)=gXg⁻¹.
# Multipath: Formula derives infinity sym group via non-commute cycle: [RSS scale, TDOA shift, AOA rot] → adj close.
# e.g., RSS ∝ 1/TDOA (geom space); AOA in SO(3). Oct embed for SO(8) (magnetism: imag parts as B-field rot).

# Params (k=6 antennas)
dim, mode = 8, "octonions"  # SO(8) proj; magnetism in imag oct (SO(3) fail → oct v1-v3 as Bx,By,Bz)
Q, R = diagm(ones(dim)), diagm(ones(dim))  # Proc/meas noise
state_prev, P_prev = zeros(dim), diagm(ones(dim))  # Init
ω, dt = randn(3), 1e-3  # Ang vel, step

# Listener: Feed incoming to fusion (e.g., v_meas from pkt.rf_packet: parse as [AOA; TDOA; RSS])
start_listener(12345, process_incoming)  # UDP port; callbacks to mekf

# Main Loop: Sim/send fused (target_ip/port for remote; here loopback)
for step in 1:100
    v_meas, v_pred = randn(dim), randn(dim)  # Stub; real from recv
    state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode)

    # Multipath: Joseph filter snaps to E8 (prime weights adj close cycle: RSS→TDOA→AOA non-commute)
    filtered = joseph_filter(state_new)

    # Packet/Send (node 1; broadcast frame)
    pkt = create_filtered_packet(filtered, vec(P_new), vec(CR); label="FusedNode1", msg="SO8 Magnetism")
    send_packet(1, pkt)  # To lattice 1; frames/send via C

    state_prev, P_prev = state_new, P_new
    sleep(dt)
end

# Viz: Traces/err (SO(8) proj to 6D)
p_traces, cr_traces, err_norms = simulate_fusion(ω, dt, 100, dim, mode)
  # If module; strip if flat


  using RFfusion

  # G=SO(3)×Aff(1)^6 (k=6), dim=3+12=15; but SO(8) proj for magnetism
  dim, mode = 3, octonions  # Full oct for SO(8)
  Q, R = diagm(ones(dim)), diagm(ones(dim))
  state_prev, P_prev = zeros(dim), diagm(ones(dim))
  ω, dt = randn(3), 1e-3

  # Magnetism stub: AOA as cross in imag (update in mekf)
  function magnetism_update(o::OctonionF64, aoa::Vector{Float64})
      B = [o.v1, o.v2, o.v3]  # B-field from imag
      rot = cross(B, aoa)     # Lorentz-like
      return OctonionF64(o.s, rot[1], rot[2], rot[3], o.v4, o.v5, o.v6, o.v7)
  end

  start_listener(12345, process_incoming)  # Feed to mekf

  for step in 1:100
      v_meas, v_pred = randn(dim), randn(dim)  # From pkt
      state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode)
      state_new = magnetism_update(state_new, randn(3))  # AOA embed
      filtered = joseph_filter(state_new)  # Hopfield min
      pkt = create_filtered_packet(filtered, vec(P_new), vec(CR); label="SO8Fusion", msg="Magnetism")
      send_packet(1, pkt)  # Framed via XBee API
      state_prev, P_prev = state_new, P_new
      sleep(dt)
  end

  p_traces, cr_traces, err_norms = simulate_fusion(ω, dt, 100, dim, mode)  # Viz

  using RFfusion, Octonions, LinearAlgebra

  dim, mode = 8, "octonions"
  Q, R = diagm(ones(dim)), diagm(ones(dim))
  state_prev, P_prev = zeros(OctonionF64, dim), diagm(ones(dim))  # Vector of Oct? No, state is single OctonionF64 (real+7imag=8D)
  ω, dt = randn(3), 1e-3
  true_state = OctonionF64(randn(8)...)  # Stub

  function magnetism_update(o::OctonionF64, aoa::Vector{Float64})
      B = [o.v1, o.v2, o.v3]
      rot = cross(B, aoa)
      return OctonionF64(o.s, rot[1], rot[2], rot[3], o.v4, o.v5, o.v6, o.v7)
  end

  start_listener(12345, process_incoming)

  states, Ps, CRs = OctonionF64[], Matrix{Float64}[], Matrix{Float64}[]
  for step in 1:100
      v_meas, v_pred = randn(8), randn(8)  # Flat; embed to oct
      v_meas_oct = pad_to_oct(v_meas[1:3]); v_meas_oct = OctonionF64(v_meas_oct.s, v_meas_oct.v1, ..., v_meas[8])  # Full
      # Similar for v_pred_oct, state_prev Octonion
      state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas_oct, v_pred_oct, P_prev, Q, R, dim, mode)
      state_new = magnetism_update(state_new, randn(3))
      filtered = joseph_filter(state_new)  # Single Octonion
      pkt = create_filtered_packet([filtered.s, filtered.v1, ..., filtered.v7], vec(P_new), vec(CR); label="SO8Fusion", msg="Magnetism")
      send_packet(1, pkt)
      push!(states, state_new); push!(Ps, P_new); push!(CRs, CR)
      state_prev, P_prev = state_new, P_new
      sleep(dt)
  end

  err_norms = [abs(state - true_state) for state in states]  # Fixed: abs on single Oct each
      p_traces, cr_traces = [tr(P) for P in Ps], [tr(CR) for CR in CRs]  # simulate_fusion equiv
