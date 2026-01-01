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
