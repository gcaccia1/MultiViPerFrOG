#!/bin/bash

# needed for doxygen-latex in ros install
# see https://askubuntu.com/a/1403761
apt update && apt install -y locales \
 && export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 LANGUAGE=en_US:en \
 && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen \
 && locale-gen \
 && apt install -y doxygen-latex