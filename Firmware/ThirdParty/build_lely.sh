#!/bin/bash

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd)
SRC_DIR="$SCRIPT_DIR/lely-core"
INSTALL_DIR="$SCRIPT_DIR/lely-build-output"
BUILD_DIR="$SRC_DIR/build"

# Ensure the source directory actually exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Error: $SRC_DIR not found. Did you clone the submodules?"
    exit 1
fi

# Flags provided by user
MY_CFLAGS="-g -fno-builtin -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__NEWLIB__"
MY_LDFLAGS="--specs=nosys.specs"

mkdir -p $BUILD_DIR
cd $BUILD_DIR

# 1. Configure (only if Makefile doesn't exist to save time)
if [ ! -f Makefile ]; then
    ../configure --host=arm-none-eabi \
        CFLAGS="$MY_CFLAGS" CXXFLAGS="$MY_CFLAGS" LDFLAGS="$MY_LDFLAGS" \
        --disable-shared --disable-tools --disable-tests --disable-python \
        --disable-cython --disable-threads --disable-daemon --disable-stdio \
        --disable-master --disable-gw --disable-gw-txt \
        --prefix="$INSTALL_DIR"
fi

# 2. Build and Install to the local install folder
make -j$(nproc)
make install