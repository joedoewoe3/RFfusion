using ProtoBuf
protojl(joinpath(@__DIR__, "..", "proto/RFpacket.proto"), joinpath(@__DIR__, "..", "proto"), joinpath(@__DIR__, "..", "src"))
using ProtoBuf
protojl("RFpacket.proto", ".", ".")

include("RFpacket_pb.jl")
# Now you can create/test a packet: pkt = SignalPacket() etc
