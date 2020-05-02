class PriorityQueue:
    def __init__(self):
        self.queue = [] # [ ("A",(1,2)), ("B", (1,3)) ] # [ (1,2,"A"), (1,3,"B") ]

    def push(self, newElem):
        bInserted = False
        for i in range(len(self.queue)):
            elem = self.queue[i]

            if(elem[1][0] >= newElem[1][0]):
                for j in range(i, len(self.queue)):
                    elem2 = self.queue[j]
            
                    if( elem2[1][0] > newElem[1][0] or elem2[1][1] >= newElem[1][1]): # promary idx is greater OR secondary idx is gt-or-eq
                        self.queue.insert(j, newElem)
                        bInserted = True
                        break
            # Break outer loop if elem inserted
            if (bInserted):
                break     
            
        #If not inserted by end of queue
        if (not bInserted):
            self.queue.append(newElem)

    def display(self):
        for elem in self.queue:
            print(elem)

    def pop(self):
        return self.queue.pop(0) # Using inbuild pop function of a list in python

    def top(self):
        if(len(self.queue) > 0):
            return self.queue[0]
        else:
            return []

    def topKey(self):
        if(len(self.queue) > 0):
            tmp = self.queue[0]
            return tmp[1]
        else:
            return []

    def remove(self, node):
        for i in range(len(self.queue)):
            nd = self.queue[i]
            if nd[0] == node[0]:
                self.queue.pop(i)
                break

    def update(self, node):
        self.remove(node)
        self.push(node)

    def getNames(self):
        return [node[0] for node in self.queue]



def testPriorityQueue():
    pq = PriorityQueue()
    #pq.push( ("A", (1,2)) )
    #pq.push( ("B", (1,2)) )
    #pq.push( ("C", (2,2)) )
    #pq.push( ("D", (0,2)) )
    #pq.push( ("E", (0,2)) )

    #pq.push( ("F", (2,3)) )
    #pq.push( ("G", (5,5)) )
    #pq.push( ("H", (5,4)) )
    #pq.push( ("I", (5,5)) )

    #pq.pop()
    #pq.pop()
    #pq.pop()

    pq.push( ("A", (1,2)) )
    pq.push( ("B", (1,2)) )
    pq.push( ("C", (2,2)) )
    pq.push( ("D", (0,2)) )
    pq.push( ("E", (0,2)) )

    pq.remove( ("A", (1,2)) )
    pq.update( ("B", (5,5)) )

    pq.display()


def getAlphaOrder(a, b):
    if a < b:
        return a+b
    else:
        return b+a