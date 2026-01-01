module RFfusion

__precompile__(false)
include("RFpacket_pb.jl")
using .RFpacket_pb
include("PacketUtils.jl")
include("ArraySpeak.jl")
include("HardwareInterface.jl") 

using .HardwareInterface
using Manifolds # For manifolds mode
using Rotations # For manifolds mode
using Octonions # For octonions mode
using Quaternions # For embeds
using LinearAlgebra
using Random # For randn
using Plots # For viz
using LinearAlgebra: pinv


ENV["GKSwtype"] = "nul"

export joseph_filter, sensor_fusion_ring_mekf, simulate_fusion, project_to_6d, e8_root_lattice, pad_to_oct


# Prime generator stub: First N primes for weighting (your human role—tie to exact arith in filter)
function generate_primes(n::Int)
    primes = Int[]
    candidate = 2
    while length(primes) < n
        if all(candidate % p != 0 for p in primes)
            push!(primes, candidate)
        end
        candidate += (candidate == 2 ? 1 : 2)
        end
        return primes
    end

    # E8 roots as Float64 for compat
    function e8_root_lattice()
        roots = Vector{Vector{Float64}}()
        # Integer roots: ±e_i ± e_j for i<j (112)
        for i in 1:8, j in i+1:8, s1 in [-1.0, 1.0], s2 in [-1.0, 1.0]
            r = zeros(Float64, 8)
            r[i] = s1
            r[j] = s2
            push!(roots, r)
        end
        # Half-integer roots: ±1/2 (e1 ±e2 ±...±e8) with even minuses (128)
        for sign_pattern in Iterators.product(fill([-1,1], 8)...)
            if count(==(-1), sign_pattern) % 2 == 0
                r = [Float64(s)/2 for s in sign_pattern]
                    push!(roots, r)
                end
            end
            return roots # Full 240 Vector{Vector{Float64}}
        end

        # Global const for roots/matrix (hoisted for scope fix—240 roots, but we gen 248 primes for E8 dim flair)
        const E8_ROOTS = e8_root_lattice()
        const ROOT_MATRIX = hcat(E8_ROOTS...) # 8x240 Matrix{Float64}
        const PRIME_WEIGHTS = generate_primes(248) # First 248 primes—use first 240 for roots, extra for Cartan/future

        # Joseph filter: Snap off-lattice to nearest true form on E8 (primes weight proj for exact tie-in)
        #function joseph_filter(h, root_matrix=ROOT_MATRIX, prime_weights=PRIME_WEIGHTS[1:size(root_matrix, 2)])
        # Weight by primes: Diag scale for "prime-divisor patterns" in proj (cyclic octo tie)
        #    W = diagm(prime_weights .* ones(length(prime_weights))) # Diag primes (or stub scalar if not)
        #    scaled_h = W * h  # Prime-scale input (your generator vibe)
        #    proj = root_matrix * (pinv(root_matrix' * root_matrix) * root_matrix' * scaled_h)  # Proper frame proj (fixed from uniform /240—handles non-tight)
        #    return pinv(W) * proj  # Unscale, snap complete (better than orig for overcomplete span)
        #end

        function joseph_filter(h, root_matrix=ROOT_MATRIX, prime_weights=PRIME_WEIGHTS[1:size(root_matrix, 2)])
            Gram = root_matrix' * root_matrix  # 240x240
            coeffs = pinv(Gram) * (root_matrix' * h)  # 240x1 coeffs for least-squares proj onto root span
            W = diagm(prime_weights)  # 240x240 diag primes—weights each root's contribution
            scaled_coeffs = W * coeffs  # 240x240 * 240x1 = 240x1
            proj = root_matrix * scaled_coeffs  # 8x240 * 240x1 = 8x1
            return proj  # Snapped with prime-weighted certainty
        end


        function pad_to_oct(v_3d::Vector{Float64})
            return Octonion(0.0, v_3d[1], v_3d[2], v_3d[3], 0.0, 0.0, 0.0, 0.0) # Embed to v1-v3
        end

        function sensor_fusion_ring_mekf(ω::Vector{Float64}, dt::Float64, state_prev::Any, v_measured::Vector{Float64}, v_predicted::Vector{Float64}, P_Previous::AbstractMatrix{Float64}, Q::AbstractMatrix{Float64}, R::AbstractMatrix{Float64}, dim::Int=3, mode::String="manifolds")
            if mode == "manifolds"
                @assert dim >= 3 "Manifolds mode requires dim >= 3"
                @assert length(v_measured) == dim && length(v_predicted) == dim "Vectors R^dim"
                tangent_dim = (dim * (dim-1)) ÷ 2
                SO_dim = SpecialOrthogonal(dim)
                e = Identity(SO_dim)
                function pad_vee(ω_3d::Vector{Float64}, dim::Int)
                    if dim == 3
                        return ω_3d
                    end
                    vee_dim = (dim*(dim-1)) ÷ 2
                    ω_padded = zeros(vee_dim)
                    ω_padded[1] = ω_3d[1] # (1,2)
                    ω_padded[2] = ω_3d[2] # (1,3)
                    ω_padded[dim] = ω_3d[3] # (2,3)
                    return ω_padded
                end
                ω_padded = pad_vee(ω, dim)
                Ω = hat(SO_dim, e, ω_padded * dt)
                ΔR = exp(SO_dim, e, Ω)
                R_gyro = ΔR * state_prev # state_prev = R_prev
                F = Matrix(I, dim, dim) + Ω
                P_Predicted = F * P_Previous * F' + Q
                h = R_gyro * v_predicted
                proj_h = (dot(h, v_measured) / (norm(h)^2 + 1e-8)) * h
                err = v_measured - proj_h
                H = - (h * h' / norm(h)^2 - Matrix(I, dim, dim))
                S = H * P_Predicted * H' + R
                K = P_Predicted * H' / S
                δω = K * err
                δω_tan = pad_vee(δω[1:3], dim) # Embed core update
                δR = exp(SO_dim, e, hat(SO_dim, e, δω_tan))
                state_fused = δR * R_gyro # Output R_fused (matrix)
                P_fused = (Matrix(I, dim, dim) - K * H) * P_Predicted * (Matrix(I, dim, dim) - K * H)' + K * R * K'
                J = H' * pinv(Matrix(R)) * H
                H_contraction = dim > 3 ? norm(h)^2 / (dim * (norm(h)^3 + 1e-6)) : norm(cross(h[1:3], cross(h[1:3], h[1:3]))) / (norm(h)^3 + 1e-6)
                curvature_factor = Matrix(I, dim, dim) + (1/12) * H_contraction * norm(P_fused) * Matrix(I, dim, dim)
                CR_bound = curvature_factor * pinv(J) * curvature_factor'
                    println("Approx Fisher J inside: ", J)
                return state_fused, P_fused, CR_bound # state_fused = R_fused (matrix)
                elseif mode == "octonions"
                @assert dim == 7 "Octo mode fixed to 7D imaginaries"
                @assert length(v_measured) == dim && length(v_predicted) == dim "Vectors R^7 (pure imaginaries)"
                ω_oct = pad_to_oct(ω) * dt / 2.0
                Δo = (Octonion(1.0) + ω_oct) / abs(Octonion(1.0) + ω_oct)
                o_gyro = Δo * state_prev # state_prev = o_prev (octonion)
                o_gyro /= abs(o_gyro)
                P_dim = 7
                Ω_approx = zeros(P_dim, P_dim)
                Ω_approx[1:3, 1:3] = [0 -ω[3] ω[2]; ω[3] 0 -ω[1]; -ω[2] ω[1] 0] * dt
                F = Matrix(I, P_dim, P_dim) + Ω_approx
                P_Predicted = F * P_Previous * F' + Q
                v_pred_oct = pad_to_oct(v_predicted[1:3])
                h_oct = o_gyro * v_pred_oct * conj(o_gyro)
                h = [h_oct.v1, h_oct.v2, h_oct.v3, h_oct.v4, h_oct.v5, h_oct.v6, h_oct.v7] # Extract imaginaries
                proj_h = (dot(h, v_measured) / (norm(h)^2 + 1e-8)) * h
                err = v_measured - proj_h
                H = - (h * h' / norm(h)^2 - Matrix(I, dim, dim))
                S = H * P_Predicted * H' + R
                K = P_Predicted * H' / S
                δω = K * err
                δω_oct = pad_to_oct(δω[1:3])
                δo = (Octonion(1.0) + δω_oct / 2.0) / abs(Octonion(1.0) + δω_oct / 2.0)
                state_fused = δo * o_gyro
                state_fused /= abs(state_fused) # Output o_fused (octonion)
                P_fused = (Matrix(I, P_dim, P_dim) - K * H) * P_Predicted * (Matrix(I, P_dim, P_dim) - K * H)' + K * R * K'
                J = H' * pinv(Matrix(R)) * H
                H_contraction = norm(h)^2 / (dim * (norm(h)^3 + 1e-6))
                curvature_factor = Matrix(I, dim, dim) + (1/12) * H_contraction * norm(P_fused) * Matrix(I, dim, dim)
                CR_bound = curvature_factor * pinv(J) * curvature_factor'
                    println("Approx Fisher J inside: ", J)
                return state_fused, P_fused, CR_bound
                elseif mode == "e8"
                @assert dim == 8 "E8 mode on 8D lattice embed (roots in R^8)"
                @assert length(v_measured) == dim && length(v_predicted) == dim "Vectors R^8 for E8 embed"
                    # State as unit octo self-ref in E8 (stub: use octo for embed, adj for update)
                    o_prev = state_prev # Assume octo for embed
                    ω_oct = pad_to_oct(ω) * dt / 2.0
                    Δo = (Octonion(1.0) + ω_oct) / abs(Octonion(1.0) + ω_oct)
                    o_gyro = Δo * o_prev
                    o_gyro /= abs(o_gyro)
                    # upgraded to all variance embeded within. no more covariance baby
                    P_dim = 8
                    variances_prev = diag(P_Previous) # shink to variances (discarding off-diagnols, for they are fictional)
                    Ω_approx = zeros(P_dim, P_dim)
                    Ω_approx[1:3, 1:3] = [0 -ω[3] ω[2]; ω[3] 0 -ω[1]; -ω[2] ω[1] 0] * dt
                    F = Matrix(I, P_dim, P_dim) + Ω_approx
                    P_Predicted = F * P_Previous * F' + Q
                    variances_pred = diag(F * P_Predicted * F' + Q) #variances
                    v_pred_oct = pad_to_oct(v_predicted[1:3])
                    h_oct = o_gyro * v_pred_oct * conj(o_gyro)
                    h = [real(h_oct), h_oct.v1, h_oct.v2, h_oct.v3, h_oct.v4, h_oct.v5, h_oct.v6, h_oct.v7] # 8D self-ref
                    h_proj = joseph_filter(h) # Snap with named concept (uses global ROOT_MATRIX/PRIMES)
                    proj_h = (dot(h, v_measured) / (norm(h)^2 + 1e-8)) * h_proj #snap complete
                    err = v_measured - proj_h
                    H = - (h * h' / norm(h)^2 - Matrix(I, dim, dim))
                    S_var = diag(H * P_Predicted * H' + R) #THE physical variance
                    K_var = variances_pred ./ S_var # Scalar gain per dim
                    δω = K_var .* err # Reality
                    δω_oct = pad_to_oct(δω[1:3])
                    δo = (Octonion(1.0) + δω_oct / 2.0) / abs(Octonion(1.0) + δω_oct / 2.0)
                    state_fused = δo * o_gyro
                    state_fused /= abs(state_fused)
                    variances_fused = (1 .- K_var) .* variances_pred + K_var .* diag(R) #Joseph Filter
                    #P_fused = (Matrix(I, P_dim, P_dim) - K * H) * P_Predicted * (Matrix(I, P_dim, P_dim) - K * H)' + K * R * K'
                    P_fused = diagm(variances_fused) # old form commented out just in case
                    J = H' * pinv(Matrix(R)) * H
                    H_contraction = norm(h)^2 / (dim * (norm(h)^3 + 1e-6))
                    curvature_factor = Matrix(I, dim, dim) + (1/12) * H_contraction * norm(P_fused) * Matrix(I, dim, dim)
                    CR_bound = curvature_factor * pinv(J) * curvature_factor'
                        println("Approx Fisher J inside: ", J)
                    return state_fused, P_fused, CR_bound # o_fused for E8 embed
                else
                    error("Mode must be 'manifolds', 'octonions', or 'e8'")
                end
            end

            # Sim update for E8 (dim=8, mode="e8")
            function simulate_fusion(steps::Int=200, dim::Int=8, mode::String="e8")
                # Init for e8: Unit octo embed
                if mode == "e8"
                    state_prev = Octonion(1.0)
                end
                if mode == "manifolds"
                    state_prev = Matrix(I, dim, dim)
                    elseif mode == "octonions"
                    state_prev = Octonion(1.0)
                end
                ω_base = [0.1, 0.2, 0.3] # Base gyro
                dt = 0.01
                v_predicted = [0.0, 0.0, -9.80665, zeros(dim-3)...] # Predicted ref
                P_previous = 0.001 * Matrix(I, dim, dim)
                Q = 0.001 * Matrix(I, dim, dim)
                R = 0.005 * Matrix(I, dim, dim)
                R[4:end, 4:end] .+= diagm(1e6 * ones(dim-3))
                # Collections for traces
                p_traces = zeros(steps)
                cr_traces = zeros(steps)
                err_norms = zeros(steps)
                for i in 1:steps
                    ω = ω_base + randn(3) * 0.01 # Noisy gyro each packet
                    v_measured = v_predicted + randn(dim) * 0.005 # Noisy measured each step
                    state_new, P_new, CR_bound = sensor_fusion_ring_mekf(ω, dt, state_prev, v_measured, v_predicted, P_previous, Q, R, dim, mode)
                    p_traces[i] = tr(P_new)
                    cr_traces[i] = tr(CR_bound)
                    # Err norm post-fusion
                    if mode == "manifolds"
                        h_fused = state_new * v_predicted
                        elseif mode == "octonions"
                        v_pred_oct = pad_to_oct(v_predicted[1:3])
                        h_oct = state_new * v_pred_oct * conj(state_new)
                        h_fused = [h_oct.v1, h_oct.v2, h_oct.v3, h_oct.v4, h_oct.v5, h_oct.v6, h_oct.v7]
                        elseif mode == "e8"
                        v_pred_oct = pad_to_oct(v_predicted[1:3])
                        h_oct = state_new * v_pred_oct * conj(state_new)
                        h = [real(h_oct), h_oct.v1, h_oct.v2, h_oct.v3, h_oct.v4, h_oct.v5, h_oct.v6, h_oct.v7]
                        h_fused = joseph_filter(h) # Use global/filter for consistent snap (fixed scope)
                    end
                    proj_h = (dot(h_fused, v_measured) / (norm(h_fused)^2 + 1e-8)) * h_fused
                    err_norms[i] = norm(v_measured - proj_h)
                    # Update
                    state_prev = state_new
                    P_previous = P_new
                end
                # Viz with log y
                p1 = plot(1:steps, [p_traces cr_traces], label=["Trace(P)" "Trace(CR)"], yscale=:log10, xlabel="Time Steps", ylabel="Log Value", title="$mode Mode Fusion ($dim D) P/CR Traces")
                p2 = plot(1:steps, err_norms, label="Err Norm", xlabel="Time Steps", ylabel="Value", title="Err Norms")
                p = plot(p1, p2, layout=(2,1), size=(800,600))
                display(p)
                savefig(p, "fusion_sim_$(mode)_$(dim)d.png")
                return p_traces, cr_traces, err_norms
            end

            # Test E8 sim (200 steps)
            p_traces, cr_traces, err_norms = simulate_fusion(200, 8, "e8")
            println("Sample P traces: ", p_traces[1:5])

            function project_to_6d(o_fused::OctonionF64)
                # Extract 7D pure imaginaries
                imaginaries = [o_fused.v1, o_fused.v2, o_fused.v3, o_fused.v4, o_fused.v5, o_fused.v6, o_fused.v7]
                # Quotient to 6D: Orthogonal proj mimicking G2/SU(3) ~ S^6 (stub: drop v7, norm the rest; real triality later)
                six_d = imaginaries[1:6]  # Or PCA: using LinearAlgebra; U = svd(randn(7,7)).U; U[1:6, :] * imaginaries
                return six_d / (norm(six_d) + 1e-8)  # Unit vector in 6D certainty space
            end

function fuse_and_send(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode, target_ip, port)
    state_new, P_new, CR = sensor_fusion_ring_mekf(ω, dt, state_prev, v_meas, v_pred, P_prev, Q, R, dim, mode)
    pkt = create_filtered_packet(state_new.x, vec(P_new), model_version=1)  # Adjust for your state
    bytes = encode_packet(pkt)
    send_packet(bytes, target_ip, port)
    return state_new, P_new, CR
end

# Listener callback stub: Decode and process incoming raw
function process_incoming(pkt::SignalPacket, src)
    if pkt.kind == PayloadKind.RAW
        antennas = pkt.payload[].antennas
        # Extract RSS/TDOA to v_meas for mekf
        println("Received from $src: Antenna1 RSS = $(antennas[1].rss)")
        # Feed to fusion...
    end
end

# Start listener in main
start_listener(1234, process_incoming)  # Port for incoming


end # module RFfusion
