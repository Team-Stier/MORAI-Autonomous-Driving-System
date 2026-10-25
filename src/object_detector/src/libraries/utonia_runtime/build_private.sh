#!/bin/sh
set -eu
rospack_src=${1:-/tmp/utonia-rospack-probe/rospack-2.6.4}
python_prefix=${2:-/tmp/utonia-python}
private_lib=${3:-/tmp/utonia-rospack-probe/lib}
mkdir -p "$private_lib"
g++ -std=c++14 -O2 -fPIC -shared -DROSPACK_API_BACKCOMPAT_V1 \
  -I"$rospack_src/include" -I"$python_prefix/include/python3.12" \
  "$rospack_src/src/rospack.cpp" "$rospack_src/src/rospack_backcompat.cpp" \
  "$rospack_src/src/rospack_cmdline.cpp" "$rospack_src/src/utils.cpp" \
  -L"$python_prefix/lib" -lpython3.12 -ltinyxml2 \
  -lboost_filesystem -lboost_program_options -lboost_system \
  -Wl,-soname,librospack.so -Wl,--disable-new-dtags,-rpath,'$ORIGIN' \
  -o "$private_lib/librospack.so"
