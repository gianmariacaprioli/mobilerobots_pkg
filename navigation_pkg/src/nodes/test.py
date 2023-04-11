#!/usr/bin/env python3
input_a = float(input("Posizione di partenza in float: "))
input_b = float(input("Posizione finale in float: "))

res = input_a - input_b

if ( (input_b - input_a) < 0.5 and (input_b - input_a) > -0.5):
    print (str(res) +" il risultato è compreso all'interno degli estremi")

