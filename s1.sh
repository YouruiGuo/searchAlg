#!/bin/bash

for i in {0..4}
do
  echo $i
  #python astar_alg.py 3d $i
  nice -n 19 time python idastar_alg.py stp $i
  #python nbs_alg.py $i
done
