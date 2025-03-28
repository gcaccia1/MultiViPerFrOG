#!/bin/bash
docker build --platform=linux/amd64 -t ubuntu_2004_noetic -f docker/cpu.Dockerfile .
docker run -it --rm ubuntu_2004_noetic bash