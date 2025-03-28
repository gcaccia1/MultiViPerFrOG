#!/bin/bash

cd /tmp \
 && git clone https://github.com/jlblancoc/nanoflann.git \
 && mkdir -p /home/catkin_wp_ceres/include/ \
 && cp nanoflann/include/nanoflann.hpp /home/catkin_wp_ceres/src/multiviperfrog/include/nanoflann.hpp
