from fastapi import FastAPI

app = FastAPI()

import sys
'''
Reading/Importing/Parsing input graph from Adj Matrix file "test.txt"
'''


with open('Maatrix') as f:
    
    lines = f.read().splitlines()



emptyGraph = []
for l in lines:
    emptyGraph.append( list(map(int, l.split(','))) )


#Applying Dijkstra algorithm 
#Empty graph es una lista de la matrix separadad con comas
    


class Graph():

	def __init__(self, vertices):
		#Recibe 847 que son los nodos de la función que es la longitud de emoty graph
		self.V = vertices

		#Crea una lista como matriz pero es reemplzada por la que le pasamos
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]

	
	def printweight(self, dist, target):
		return dist[target]



	def getShortestPath(self,distances,previous,targetIndex):
		path = []

		#Handle errors if path does not exist
		if(distances[targetIndex]==sys.maxsize):
			return path

		at = targetIndex
		#Goes to index of previous list to find what element came before
		while at is not None :
			path.append(at)
			at = previous[at]
		path.reverse()

		return path


	# A utility function to find the vertex with
	# minimum distance value, from the set of vertices
	# not yet included in shortest path tree
	def minDistance(self, dist, sptSet):

		# Initialize minimum distance for next node
		min = sys.maxsize

		# Search not nearest vertex not in the
		# shortest path tree

		#Itera 847 veces
		for u in range(self.V):
			#Si el valor es menor al infinito y no hemos pasado por ese nodo

			if dist[u] < min and sptSet[u] == False:
				min = dist[u]
				# U es la posición del nodo 
				min_index = u

		return min_index

	# Function that implements Dijkstra's single source
	# shortest path algorithm for a graph represented
	# using adjacency matrix representation
	def dijkstra(self, src, target):

		#Lista de distancias con infinitos
		dist = [sys.maxsize] * self.V

		#Lista de caminos previos por nodo [Para conocer la ruta]
		prev = [None]*self.V


		#Inicializa el source
		dist[src] = 0

		#Lista si ya pasamos por ahí
		sptSet = [False] * self.V


		#Itera 847 veces
		for cout in range(self.V):

			# Pick the minimum distance vertex from
			# the set of vertices not yet processed.
			# x is always equal to src in first iteration
			x = self.minDistance(dist, sptSet)
			#print(x)

			# Put the minimum distance vertex in the
			# shortest path tree
			sptSet[x] = True

			# Update dist value of the adjacent vertices
			# of the picked vertex only if the current
			# distance is greater than new distance and
			# the vertex in not in the shortest path tree

			#Itera 847 veces
			for y in range(self.V):
				#Revisa los neighbors de la matriz
				'''
				self.graph[x][y] > 0
					Revisa si la matriz de 847*847 es un neighbor del posible nodo
				
				sptSet[y] == False
					Revisa si es un nodo que ya hemos visitado
				
				dist[y] > dist[x] + self.graph[x][y]
					Revisa si en la lista de infinitos en el peso del nuevo nodo es mayor a la suma del nodo anterior con los anteriores para llegar a ese punto

				'''
				if self.graph[x][y] > 0 and sptSet[y] == False and	dist[y] > dist[x] + self.graph[x][y]:
					#Adds the previous node to the path list
					prev[y] =  x
					
					#The new node's weight is inserted into the list as the sum of the current node with the weight of the path to arrive there
					dist[y] = dist[x] + self.graph[x][y]

		#self.printSolution(dist)
		#print(self.printweight(dist, target))
		#print(self.getShortestPath(dist,prev,target))
		return self.printweight(dist, target), self.getShortestPath(dist,prev,target)

@app.get("/prueba/{ini}/{end}/")
def hello(ini, end):

    
    g = Graph(len(emptyGraph))

    g.graph = emptyGraph

    prueba = g.dijkstra(int(ini), int(end))

    return {"puntos": prueba[1], "peso": prueba[0]}