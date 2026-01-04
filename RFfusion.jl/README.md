Requirements to build: protobuf

Windows:

choco install protobuf

Linux:

sudo apt install protobuf-compiler

openSUSE 

sudo zypper install protobuf

Then:
-open julia in RFfusion/ dir.

Julia] activate .
julia> PPkg.build()


if protoc is not found, uncomment ENV["PROTOC"] = "path/to/protoc.exe" in RFfusion/deps/build.jl

Dev Workflow:
Edit proto/RFpacket.proto as needed.
Run julia --project=. -e 'using Pkg; Pkg.build("RFFusion")' (or from REPL: ]build RFFusion).
This triggers deps/build.jl, regenerating src/RFpacket_pb.jl.
Commit the updated _pb.jl to Git (avoids users needing to build every time; common in packages like Elly.jl).
Test: ]activate .; using RFFusion; test_packet = RFPacket(...) etc.

User Workflow (Streamlined Install):
Pkg.add("https://github.com/youruser/RFFusion.jl.git") (or register in General registry for public).
Build runs auto if needed (but since _pb.jl is committed, it's fast/no-op).
Use: using RFFusion; # access RFPacket, etc.
