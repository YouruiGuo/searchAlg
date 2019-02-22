#!/bin/bash

for i in {0..99}
do
  echo $i
  python astar_alg.py 3d $i
  #time python idastar_alg.py stp $i
  #python nbs_alg.py $i
done
