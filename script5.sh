#!/bin/bash

for i in {40..49}
do
  echo $i
  #python astar_alg.py stp $i
  #time python idastar_alg.py stp $i
  python nbs_alg.py $i
done
