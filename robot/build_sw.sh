#!/bin/bash

rm -r dockerid.id
docker run \
        -it --cidfile dockerid.id \
        -v ${PWD}/ws:/home/lois \
        --privileged \
        --network host \
        --cap-add SYS_TIME \
        lois_galactic /home/lois/build_sw.sh
