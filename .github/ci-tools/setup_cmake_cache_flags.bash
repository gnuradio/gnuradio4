#!/bin/bash
# Copyright 2026 Marcus Müller
source "$(dirname "$(realpath "$0")")/common.bash"

type -p sccache > /dev/null || bail_with_message 'skipping sccache setup' 'sccache not found'
[[ -z "${SCCACHE_S3_BUCKET}" ]] && bail_with_message 'skipping sccache setup' 'S3 bucket not defined (empty/missing SCCACHE_S3_BUCKET env var)'

SCCACHE_BIN="$(type -p sccache)"

add_output C_LAUNCHER   "-DCMAKE_C_COMPILER_LAUNCHER=${SCCACHE_BIN}"
add_output CXX_LAUNCHER "-DCMAKE_CXX_COMPILER_LAUNCHER=${SCCACHE_BIN}"

mkdir -p ~/.config/sccache

>> ~/.config/sccache/config cat << EOF
[cache.s3]
endpoint = "${SCCACHE_S3_ENDPOINT}"
bucket = "${SCCACHE_S3_BUCKET}"
use_ssl = true
server_side_encryption = false
no_credentials = false
region = "${SCCACHE_S3_REGION}"
EOF

add_output SCCACHE_CONF "$(realpath ~/.config/sccache/config)"

bail_with_message 'enabling sccache' "Using ${SCCACHE_BIN} ($(${SCCACHE_BIN} --version)) as sccache binary."
