#!/bin/bash

cd /media/data/dev/linux/hpp/src/anymal-rbprm/script/relative_foot_positions;
./run.sh relativeFootPositionQuasiFlat.py
/media/data/blender-2.82-linux64/blender --background --python reduce.py

for f in ./output/*red.obj; do mv "$f" "${f%.obj}uced.obj"; done
cp ./output/* /media/data/dev/linux/hpp/src/sl1m/stand_alone_scenarios/constraints_files/anymal/

cd -
