import sys
sys.path.append('/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion')
sys.path.append('/mnt/c/Users/Dylan/Desktop/NewWindAware/source/path_generation')
from grid_construction import Grid
from python_path_reception import memory_trick
import numpy as np
from copy import deepcopy

def generate_random_vectors(num_points, vector_length):
    # Define the minimum and maximum values for each dimension
    min_vals = np.array([-73, 43, 80])
    max_vals = np.array([-74, 45, 8000])

    # Generate random numbers between 0 and 1, then scale and shift them
    random_vectors = np.random.rand(num_points, vector_length)
    scaled_vectors = min_vals + (max_vals - min_vals) * random_vectors

    return scaled_vectors

num_points = 2048  # Adjust as needed
vector_length = 3

# Generate random data for q0s and q1s
q0s = generate_random_vectors(num_points, vector_length)
q1s = generate_random_vectors(num_points, vector_length)
q0s = np.concatenate((q0s, np.array([[-73.8767,40.8513,802400]])))
q1s = np.concatenate((q1s, np.array([[-73.8571,40.7721,50]])))

from montecarlo.node import Node
from montecarlo.montecarlo import MonteCarlo

grid = Grid()
grid.add_region("n43w074")

def norm(pointA, pointB):
	return ((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2 + (pointB[2] - pointA[2])**2)**(1/2)

best_yet = -np.inf
class PointSet:
	def __init__(self, q0, q1):
		self.q0s = q0
		self.q1s = q1
		self.end = q1

	def give_options(self):
		samples = 10
		angles = [k*2*np.pi/samples for k in range(1,samples)]
		points = []
		for element in angles:
			points.append(self.intersection + (np.random.rand()+0.1)*10*np.array([np.sin(element), np.cos(element), 0]))
		return points

	def judge_path(self, path, desired_point):
		global grid
		intersections = np.array(grid.path_collision(path)).reshape(-1, 5)
		point_final = np.array(intersections[0][0:3])
		point_final[2] = point_final[2]*364173
		self.intersection = point_final
		return -(intersections.shape[0])

	def get_sample_points(self):
		point_array = []
		for a, b in zip(self.q0s, self.q1s):
			point_array.append([a, b])
		return point_array

	def merge_paths(self, example_paths):
		end_path = []
		for element in example_paths:
			end_path.extend(element[1:-1])
		return end_path

	def evaluate(self):
		global best_yet
		total_path = memory_trick(self.q0s, self.q1s)
		total_path = total_path[:len(self.q0s)]
		end_path = np.array(self.merge_paths(total_path))
		value = self.judge_path(end_path, self.end)
		print("Value: ", value)
		print("Q0s: ", self.q0s)
		print("Q1s: ", self.q1s)
		print("Best yet: ", best_yet)
		if(value >= best_yet):
			if(best_yet == -np.inf):
				best_yet = -1e20
			else:
				best_yet = value
		return value

	def add_intermediate(self, choice):
		# q0s - sources
		# q1s - destinations
		# A -> C -> B
		# q0 = [A]
		# q1 = [B]

		# q0 = [A, C]
		# q1 = [C, B]

		# A -> C -> D -> B
		# q0 = [A, C, D]
		# q1 = [C, D, B]

		temp = self.q1s[-1] # B
		self.q1s = self.q1s[:-1] # []
		self.q0s = np.concatenate((self.q0s, np.array([choice])), axis=0)
		self.q1s = np.concatenate((np.array([choice]), self.q1s, np.array([temp])), axis=0)

def child_finder(node, montecarlo):
	for move in node.state.give_options():
		child = Node(deepcopy(node.state)) #or however you want to construct the child's state
		child.state.add_intermediate(move) #or however your library works
		node.add_child(child)

def node_evaluator(node, montecarlo):
	value = node.state.evaluate()
	return value

#--------------------------------------------------

print("Making inital path...")
inital_path = PointSet(np.array([[-73.8767,40.8513,8024]]), np.array([[-73.8571,40.7721,50]]))
print("Loading...")
inital_path.evaluate()

print("Setting up Monte Carlo...")
montecarlo = MonteCarlo(Node(inital_path))

montecarlo.child_finder = child_finder
montecarlo.node_evaluator = node_evaluator
montecarlo.simulate(50)