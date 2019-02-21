#!/bin/bash

for i in {10..19}
do
  echo $i
  #python astar_alg.py stp $i
  #time python idastar_alg.py stp $i
  nice -n 19 python nbs_alg.py $i
done
