using ProtoBuf
using Libdl  # For lib loading checks

# Paths (relative to package root)
base_dir = dirname(@__DIR__)  # Points to package root
proto_dir = joinpath(base_dir, "proto")
src_dir = joinpath(base_dir, "src")
c_src_dir = joinpath(@__DIR__, "src")  # deps/src for C files
lib_dir = joinpath(base_dir, "lib")    # Output dir for .so (create if needed)
mkpath(lib_dir)  # Ensure lib dir exists

# Step 1: Generate Julia protobuf code (your original)
#<pre style='color:#1f1c1b;background-color:#ffffff;'>
#    <span style='color:#644a9b;'>protojl</span>(<span style='color:#644a9b;'>joinpath</span>(proto_dir, <span style='color:#bf0303;'>&quot;RFpacket.proto&quot;</span>), #proto_dir, src_dir)</pre>
protojl("./RFpacket.proto", proto_dir, src_dir)


# Step 2: Compile C wrapper to libRFWrapper.so
c_files = ["rf_wrapper.c", "RFpacket.pb-c.c"]  # Your sources
lib_ext = Sys.iswindows() ? "dll" : Sys.isapple() ? "dylib" : "so"
lib_path = joinpath(lib_dir, "libRFWrapper.so")

# Check if already built (simple dep check; improve with mtimes if needed)
if isfile(lib_path)
    # Compile command (adapt from your gcc line; assumes protobuf-c dev installed)
    cmd = Sys.iswindows() ? `cl /LD /Fe$lib_path ...` : `clang -shared -fPIC -o $lib_path $(joinpath.(c_src_dir, c_files)) -lprotobuf-c`  # Adapt compiler

    try
        run(cmd)
        println("Compiled libRFWrapper.so successfully.")
        catch e
        error("C compilation failed: $e. Ensure gcc and libprotobuf-c-dev are installed.")
    end
end

# Optional: Verify load (helps debug during build)
try
    dlopen(lib_path)
    println("libRFWrapper.so loaded OK.")
    catch e
    error("Failed to load compiled lib: $e")
end

# Include generated pb (ensures it's fresh for REPL/tests)
include(joinpath(src_dir, "RFpacket_pb.jl"))
