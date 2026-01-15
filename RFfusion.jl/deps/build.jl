using ProtoBuf
using Libdl  # For lib loading checks

# Paths (relative to package root)
base_dir = dirname(@__DIR__)  # Points to package root
proto_dir = joinpath(base_dir, "proto")
src_dir = joinpath(base_dir, "src")
c_src_dir = joinpath(base_dir, "src")  # From your adjust, C files in src (or src/proto? Adjust if subdir)
lib_dir = joinpath(base_dir, "lib")    # Output dir for .so (create if needed)
mkpath(lib_dir)  # Ensure lib dir exists

# Step 0: Delete old generated files to force regen each build
old_pb_jl = joinpath(src_dir, "RFpacket_pb.jl")
old_pb_c = joinpath(c_src_dir, "RFpacket.pb-c.c")
old_pb_h = joinpath(c_src_dir, "RFpacket.pb-c.h")
old_lib = joinpath(lib_dir, "libRFWrapper.so")
for file in [old_pb_jl, old_pb_c, old_pb_h, old_lib]
    isfile(file) && rm(file)
end

# Step 1: Generate C protobuf code (protoc --c_out)
proto_file = joinpath(proto_dir, "RFpacket.proto")
cmd_c = `protoc -I=$proto_dir --c_out=$c_src_dir $proto_file`
cmd_csharp = `protoc -I=$proto_dir --csharp_out=$c_src_dir $proto_file`
try
    run(cmd_c)
    println("Generated C protobuf code: $old_pb_c and $old_pb_h")
    catch e
    error("protoc --c_out failed: $e. Ensure protoc is installed and on PATH.")
end
try
    run(cmd_csharp)
    println("Generated C protobuf code: $old_pb_c and $old_pb_h")
    catch e
    error("protoc --c_out failed: $e. Ensure protoc is installed and on PATH.")
end
# Step 2: Generate Julia protobuf code (protojl)
protojl(joinpath(".", "RFpacket.proto"), proto_dir, src_dir)
println("Generated Julia protobuf code: $old_pb_jl")

# Step 3: Compile C wrapper to libRFWrapper.so (always since deleted)
c_files = ["rf_wrapper.c", "RFpacket.pb-c.c"]  # Your sources
lib_path = joinpath(lib_dir, "libRFWrapper.so")
cmd_compile = `gcc -shared -fPIC -o $lib_path $(joinpath.(c_src_dir, c_files)) -lprotobuf-c`  # Or clang
try
    run(cmd_compile)
    println("Compiled libRFWrapper.so successfully.")
    catch e
    error("C compilation failed: $e. Ensure gcc and libprotobuf-c-dev are installed.")
end

# Step 4: Verify load (helps debug during build)
try
    dlopen(lib_path)
    println("libRFWrapper.so loaded OK.")
    catch e
    error("Failed to load compiled lib: $e")
end

# Include generated pb (ensures it's fresh for REPL/tests)
include(old_pb_jl)
