#!/usr/bin/env bash
# Copyright 2026 Marcus Müller
source "$(dirname "$(realpath "$0")")/common.bash"

type -p sccache > /dev/null || bail_with_message 'sccache' 'skipping sccache setup: sccache not found'
# [[ -z "${SCCACHE_S3_BUCKET}" ]] && bail_with_message 'skipping sccache setup' 'S3 bucket not defined (empty/missing SCCACHE_S3_BUCKET env var)'

printf '::group::S3 setup\n'
if [[ -z "${SCCACHE_S3_BUCKET}" ]] ; then
  gh_message "sccache" "using default S3 bucket"
  export SCCACHE_S3_BUCKET="gr4-sccache"
  add_env SCCACHE_S3_BUCKET "${SCCACHE_S3_BUCKET}"
fi
if [[ -z "${SCCACHE_S3_ENDPOINT}" ]] ; then
  gh_message "sccache" "using default S3 endpoint"
  export SCCACHE_S3_ENDPOINT="s3.us-west-002.backblazeb2.com"
  add_env SCCACHE_S3_ENDPOINT "${SCCACHE_S3_ENDPOINT}"
fi
if [[ -z "${SCCACHE_S3_REGION}" ]] ; then
  gh_message "sccache" "using default S3 region"
  export SCCACHE_S3_REGION="auto"
  add_env SCCACHE_S3_REGION "${SCCACHE_S3_REGION}"
fi
aws_ro="false"
if [[ -z "${AWS_ACCESS_KEY_ID}" ]] ; then
  # set up default bucket here
  gh_message "sccache" "AWS key ID not set. Using default read-only key ID & key"
  export AWS_ACCESS_KEY_ID="0021090e73dcc12000000000c"
  add_env AWS_ACCESS_KEY_ID "${AWS_ACCESS_KEY_ID}"
  aws_ro="true"
  # Can't have a key without a key id.
  export AWS_SECRET_ACCESS_KEY="K002P+ZeW31+o4HIhDz8CXZg6OpFI4k"
fi
add_env AWS_SECRET_ACCESS_KEY "${AWS_SECRET_ACCESS_KEY}"
printf '::endgroup::'

SCCACHE_BIN="$(type -p sccache)"
SCCACHE_CONF="${HOME}/.config/sccache/config"
add_output C_LAUNCHER   "-DCMAKE_C_COMPILER_LAUNCHER=${SCCACHE_BIN}"
add_output CXX_LAUNCHER "-DCMAKE_CXX_COMPILER_LAUNCHER=${SCCACHE_BIN}"

if [[ ! -e "${SCCACHE_CONF}" ]]; then
  gh_message 'sccache' "Creating sccache config in ${SCCACHE_CONF}"
  mkdir -p ~/.config/sccache

>> "${SCCACHE_CONF}" cat << EOF
[cache.s3]
endpoint = "${SCCACHE_S3_ENDPOINT}"
bucket = "${SCCACHE_S3_BUCKET}"
use_ssl = true
server_side_encryption = false
no_credentials = ${aws_ro}
region = "${SCCACHE_S3_REGION}"
EOF
fi

add_output SCCACHE_CONF "${SCCACHE_CONF}"
add_env SCCACHE_ERROR_LOG "/tmp/local_sccache.log"
# These could be set here, but they do spam the compile output slightly.
# add_env SCCACHE_LOG debug
# Instead, we just set the environment locally with debug logging on
SCCACHE_LOG=debug sccache --start-server

bail_with_message 'sccache' "Using ${SCCACHE_BIN} ($(${SCCACHE_BIN} --version)) as sccache binary."
