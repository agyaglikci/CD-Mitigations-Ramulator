#!/bin/bash
cd "$(dirname "$0")"
docker run -it --rm --platform linux/amd64 \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    -v "$(pwd)":/workspace -w /workspace chronus-env
