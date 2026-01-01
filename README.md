# RFfusion
3D SO3, 7D octonions, and 8D E8 rf fusion. Joseph filter uses E8 lattice and Primes to filter falsehoods. Tip of the ladder. It is the most generalized filter possible.  

Requirements to build: protobuf

Windows:

choco install protobuf

Linux:

sudo apt install protobuf-compiler

openSUSE

sudo zypper install protobuf

Then: -open julia in RFfusion/ dir.

Julia] activate . julia> PPkg.build()

if protoc is not found, uncomment ENV["PROTOC"] = "path/to/protoc.exe" in RFfusion/deps/build.jl
