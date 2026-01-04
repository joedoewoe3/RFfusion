module ArraySpeak
using Sockets


export send_packet, receive_packet, start_listener

function send_packet(bytes::Vector{UInt8}, target_ip::String, target_port::Int)
    sock = UDPSocket()
    try
        send(sock, parse(IPv4, target_ip), target_port, bytes)
    finally
        close(sock)
    end
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

# Threaded listener for continuous receive (e.g., from antennas)
function start_listener(listen_port::Int, callback::Function)
    Threads.@spawn begin
        while true
            pkt, src = receive_packet(listen_port)
            callback(pkt, src)  # e.g., feed to mekf
        end
    end
end

end  # module