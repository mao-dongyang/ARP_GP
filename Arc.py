class Arc:
    def setHead(self, head):
        self.head=head
    def getHead(self):
        return self.head

    def setTail(self, tail):
        self.tail=tail
    def getTail(self):
        return self.tail

    def setSerCost(self, serCost): #serving cost
        self.serCost=serCost
    def getSerCost(self):
        return self.serCost

    def setDeadCost(self, deadCost): #deadheading cost
        self.deadCost = deadCost
    def getDeadCost(self):
        return self.deadCost

    def setStoDeadCost(self, stoDeadCost): #stochastic
        self.stoDeadCost = stoDeadCost
    def getStoDeadCost(self):
        return self.stoDeadCost

    def setConnect(self, connect):
        self.connect = connect
    def getConnect(self):
        return self.connect

    def setDemand(self, demand):
        self.demand=demand
    def getDemand(self):
        return self.demand

    def setStoDemand(self, stoDemand): #stochastic
        self.stoDemand = stoDemand
    def getStoDemand(self):
        return self.stoDemand

    def setHValue(self, hValue):
        self.hValue=hValue
    def getHValue(self):
        return self.hValue

    def setRDF(self,rdf):#remaining demand fraction
        self.rdf = rdf
    def getRDF(self):
        return self.rdf