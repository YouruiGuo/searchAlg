#!/bin/bash

for i in {20..24}
do
  echo $i
  #python astar_alg.py 3d $i
  nice -n 19 time python idastar_alg_retest.py stp $i
  #python nbs_alg.py $i
done
