# -*- coding: utf-8 -*-
import os
import re
import Arc
print(os.getcwd())
class graph:
    def read(self):
        reader=open("gdb19.dat","r+")
        #read from here
        line=reader.readline() #line 1: "NOMBRE : ***"
        segments=line.strip().split(" ")
        dataName=segments[2]
        reader.readline() #line 2: "COMENTARIO : *** (cota superior)"
        line=reader.readline() #line 3: "VERTICES : ***"
        segments=line.strip().split(" ")
        numNodes=int(segments[2])
        line=reader.readline() #line 4: "ARISTAS_REQ : ***"
        segments=line.strip().split(" ")
        numReq=int(segments[2])
        line=reader.readline() #line 5: "ARISTAS_NOREQ : ***"
        segments=line.strip().split(" ")
        numNonReq=int(segments[2])
        line = reader.readline() #line 6: "VEHICULOS : ***"
        segments = line.strip().split(" ")
        vehiculous = int(segments[2])
        line=reader.readline() #line 7: "CAPACIDAD : ***"
        segments=line.strip().split(" ")
        capacity=float(segments[2])
        reader.readline() #line 8: "TIPO_COSTES_ARISTAS : EXPLICITOS"
        reader.readline() #line 9: "COSTE_TOTAL_REQ : ***"
        reader.readline() #line 10: "LISTA_ARISTAS_REQ :"

        i=0
        arcList = []
        while i<numReq:
            line=reader.readline()
            segments=re.split('[(),\s]', line.strip())
            numbers=[]
            for seg in segments:
                if seg.isdigit():
                    numbers.append(seg)
            arc1=Arc.Arc()
            arc2=Arc.Arc()
            arc1.setHead(int(numbers[0]) - 1) #因为编号是从1开始的，为后续方便，统一从0开始
            arc1.setTail(int(numbers[1]) - 1)
            arc1.setSerCost(float(numbers[2]))
            arc1.setDeadCost(float(numbers[2]))
            arc1.setDemand(float(numbers[3]))
            arc1.setConnect(1) #1表示连通，默认是连通的
            arc2.setHead(int(numbers[1]) - 1)
            arc2.setTail(int(numbers[0]) - 1)
            arc2.setSerCost(float(numbers[2]))
            arc2.setDeadCost(float(numbers[2]))
            arc2.setDemand(float(numbers[3]))
            arc2.setConnect(1) #1表示连通，默认是连通的
            arcList.append(arc1)
            arcList.append(arc2)
            i += 1

        line=reader.readline() #last line: "DEPOSITO :"
        segments=line.strip().split(" ")
        depot=int(segments[-1]) - 1
        reader.close()
        return numNodes, numReq, arcList, capacity, depot, vehiculous