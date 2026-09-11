#!/bin/bash

# get directory of this script relative to root
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ "$1" == "--a380x" ]; then
  AIRCRAFT_FLAG="A380X"
else
  AIRCRAFT_FLAG="A32NX"
fi

OUTPUT="${DIR}/out/terronnd_${AIRCRAFT_FLAG}.wasm"

if [ "$2" == "--debug" ]; then
  CLANG_ARGS="-g"
else
  WASMLD_ARGS="--strip-debug"
fi

set -e

# create temporary folder for o files
mkdir -p "${DIR}/obj/${AIRCRAFT_FLAG}"
pushd "${DIR}/obj/${AIRCRAFT_FLAG}"

# compile c++ code
clang++ \
  -c \
  ${CLANG_ARGS} \
  -std=c++20 \
  -W \
  -Wall \
  -Wextra \
  -Wc++20-compat \
  -Wno-unused-command-line-argument \
  -Wno-ignored-attributes \
  -Wno-macro-redefined \
  -Wshadow \
  -Wdouble-promotion \
  -Wundef \
  -Wconversion \
  --sysroot "${MSFS_SDK}/WASM/wasi-sysroot" \
  -target wasm32-unknown-wasi \
  -D_MSFS_WASM=1 \
  -D__wasi__ \
  -D_LIBCC_NO_EXCEPTIONS \
  -D_LIBCPP_HAS_NO_THREADS \
  -D_WINDLL \
  -D_MBCS \
  -D${AIRCRAFT_FLAG} \
  -mthread-model single \
  -fno-exceptions \
  -fms-extensions \
  -fvisibility=hidden \
  -fno-common \
  -fstack-usage \
  -fdata-sections \
  -fno-stack-protector \
  -fstack-size-section \
  -mbulk-memory \
  -Werror=return-type \
  -O2 \
  -I "${MSFS_SDK}/WASM/include" \
  -I "${MSFS_SDK}/SimConnect SDK/include" \
  -I "${DIR}/../cpp-msfs-framework/lib/" \
  "${DIR}/src/main.cpp" \
  "${DIR}/src/nanovg/nanovg.cpp" \
  "${DIR}/src/navigationdisplay/collection.cpp" \
  "${DIR}/src/navigationdisplay/displaybase.cpp" \
  "${DIR}/src/simconnect/connection.cpp" \

if [ "${AIRCRAFT_FLAG}" == "A380X" ]; then
  # local terrain-on-ND rendering, used as a fallback when SimBridge isn't connected -- see
  # fbw-common/src/wasm/terronnd/src/localterrain. A380X-only (see the #ifdef A380X guards in
  # navigationdisplay/{collection,display}.h), so these are only ever compiled into the A380X module.
  clang++ \
    -c \
    ${CLANG_ARGS} \
    -std=c++20 \
    -W \
    -Wall \
    -Wextra \
    -Wc++20-compat \
    -Wno-unused-command-line-argument \
    -Wno-ignored-attributes \
    -Wno-macro-redefined \
    -Wshadow \
    -Wdouble-promotion \
    -Wundef \
    -Wconversion \
    --sysroot "${MSFS_SDK}/WASM/wasi-sysroot" \
    -target wasm32-unknown-wasi \
    -D_MSFS_WASM=1 \
    -D__wasi__ \
    -D_LIBCC_NO_EXCEPTIONS \
    -D_LIBCPP_HAS_NO_THREADS \
    -D_WINDLL \
    -D_MBCS \
    -D${AIRCRAFT_FLAG} \
    -mthread-model single \
    -fno-exceptions \
    -fms-extensions \
    -fvisibility=hidden \
    -fno-common \
    -fstack-usage \
    -fdata-sections \
    -fno-stack-protector \
    -fstack-size-section \
    -mbulk-memory \
    -Werror=return-type \
    -O2 \
    -I "${MSFS_SDK}/WASM/include" \
    -I "${MSFS_SDK}/SimConnect SDK/include" \
    -I "${DIR}/../cpp-msfs-framework/lib/" \
    -I "${DIR}/../fbw_common/src/zlib" \
    "${DIR}/src/localterrain/terrainmap.cpp" \
    "${DIR}/src/localterrain/ndrenderer.cpp" \

  # vendored zlib (gzip inflate for terrain.map's compressed tiles) -- plain C, compiled separately from the
  # C++20 sources above since it isn't C++.
  clang \
    -c \
    ${CLANG_ARGS} \
    -std=c17 \
    -Wno-unused-command-line-argument \
    --sysroot "${MSFS_SDK}/WASM/wasi-sysroot" \
    -target wasm32-unknown-wasi \
    -D_MSFS_WASM=1 \
    -D__wasi__ \
    -mthread-model single \
    -fno-common \
    -fdata-sections \
    -fno-stack-protector \
    -mbulk-memory \
    -O2 \
    -I "${DIR}/../fbw_common/src/zlib" \
    "${DIR}/../fbw_common/src/zlib/inflate.c" \
    "${DIR}/../fbw_common/src/zlib/inftrees.c" \
    "${DIR}/../fbw_common/src/zlib/inffast.c" \
    "${DIR}/../fbw_common/src/zlib/zutil.c" \
    "${DIR}/../fbw_common/src/zlib/adler32.c" \
    "${DIR}/../fbw_common/src/zlib/crc32.c" \

fi

# restore directory
popd

# create the output folder
mkdir -p "${DIR}/out"

# link modules
wasm-ld \
  --no-entry \
  --allow-undefined \
  -L "${MSFS_SDK}/WASM/wasi-sysroot/lib/wasm32-wasi" \
  -lc "${MSFS_SDK}/WASM/wasi-sysroot/lib/wasm32-wasi/libclang_rt.builtins-wasm32.a" \
  --export __wasm_call_ctors \
  --export-dynamic \
  --export malloc \
  --export free \
  --export __wasm_call_ctors \
  --export mallinfo \
  --export mchunkit_begin \
  --export mchunkit_next \
  --export get_pages_state \
  --export mark_decommit_pages \
  --export-table \
  --gc-sections \
  ${WASMLD_ARGS} \
  -O2 \
  -lc++ -lc++abi \
  ${DIR}/obj/${AIRCRAFT_FLAG}/*.o \
  -o $OUTPUT
