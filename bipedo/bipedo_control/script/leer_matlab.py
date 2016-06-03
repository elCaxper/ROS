#!/usr/bin/env python
from scipy.io import loadmat
x = loadmat('estruc_piernas.mat')
# print x
piernas_der = x['estruc_piernaDer']
art1_der =piernas_der[0,0]
art2_der =piernas_der[0,1]
art3_der =piernas_der[0,2]
art4_der =piernas_der[0,3]
art5_der =piernas_der[0,4]
art6_der =piernas_der[0,5]

print art1_der['senialCompleta'].shape[1]

for i in range(0,art1_der['senialCompleta'].shape[1],10):
    print art1_der['tiempo_completo'][0,i]
# lat = x['senialCompleta']
# one-liner to read a single variable
