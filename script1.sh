#!/bin/bash

for i in {0..9}
do
  echo $i
  #python astar_alg.py stp $i
  #nice -n 19 time python idastar_alg.py stp $i
  nice -n 19 python nbs_alg.py $i
done
