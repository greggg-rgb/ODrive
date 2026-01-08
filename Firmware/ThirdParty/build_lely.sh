#!/bin/bash

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd)
SRC_DIR="$SCRIPT_DIR/lely-core"
INSTALL_DIR="$SCRIPT_DIR/lely-build-output"
BUILD_DIR="$SRC_DIR/build"

# Force-delete the build folder and install and relevant files to ensure a fresh configuration
rm -rf "$BUILD_DIR"
rm -rf "$INSTALL_DIR"

# Ensure the source directory actually exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Error: $SRC_DIR not found. Did you clone the submodules?"
    exit 1
fi

if [ -d "$SRC_DIR/.git" ] || [ -f "$SRC_DIR/.git" ]; then
    echo "Cleaning lely-core source tree via git..."
    cd "$SRC_DIR"
    git clean -xfd
    git checkout .
fi

# Flags provided by user
MY_CFLAGS="-g -fno-builtin -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__NEWLIB__ -D__timespec_defined -D_TIMESPEC_DEFINED"
MY_LDFLAGS="--specs=nosys.specs"


cd $SRC_DIR
autoreconf -i

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
if [ ! -f Makefile ]; then
    ../configure --host=arm-none-eabi \
        CFLAGS="$MY_CFLAGS" CXXFLAGS="$MY_CFLAGS" LDFLAGS="$MY_LDFLAGS" \
        --disable-cxx --disable-shared --disable-tools --disable-tests --disable-python \
        --disable-cython --disable-threads --disable-daemon --disable-stdio \
        --disable-master --disable-gw --disable-gw-txt \
        --prefix="$INSTALL_DIR" \
        ac_cv_struct_timespec=yes \
        ac_cv_struct_itimerspec=yes
fi

# Build and Install to the local install folder
make -j$(nproc)
make install

# Cleanup build directory to avoid pollution
rm -rf "$BUILD_DIR"