import numpy as np
import math
from enum import Enum

# Ethan Pascuales
# Matthew Byrnes

class MoveType(Enum):
      Continue = 0
      Seek = 6
      Flee = 7
      Arrive = 8
      Follow = 1

# Global Movement Constants
time = 0
maxAcceleration = 100.0
maxSpeed = 1.0 # maximum speed for character
targetRadius = 5.0 # arrival radius
#slowRadius = 1.0 # slowing-down radius
timeToTarget = 0.1 

class Path(object):
      def __init__(self):
            self.x = np.array([])
            self.y = np.array([])
            self.params = np.array([])
            self.distance = np.array([])
            self.total_distance = 0
            self.segments = 0

      def pathAssemble(self,ID, X,Y): 
            for item in X:
                  self.x = np.append(self.x, item) # check if this works for numpy arrays
                  
            for item in Y:
                  self.y = np.append(self.y,item)

            self.segments = len(X) - 1 # Generates total num of segments
            
            # Create distance for each lines segment and store it in segments array
            """
            for i in range(self.segments - 1):
                  # x_1, x_2, y_1, y_2 for distance formula
                  a = np.array([self.x[i], self.x[i + 1]])
                  b = np.array([self.y[i], self.y[i + 1]])
                  self.distance = np.append(self.distance,distanceBetweenPoints(a, b))
            """ 
            self.distance = np.append(self.distance, 0.00)
            for i in range(self.segments): #+ 1
                  a = np.array([self.x[i], self.x[i + 1]])
                  b = np.array([self.y[i], self.y[i + 1]])
                  distance_covered = self.distance[-1] + distanceBetweenPoints(a, b) #
                  self.distance = np.append(self.distance, distance_covered)
            #for distance in self.distance:
            #      self.total_distance += distance
            self.total_distance += self.distance[-1] # Final distance = total?

            for i in range(self.segments + 1): # Old vers: - 1
                  self.params = np.append(self.params,self.distance[i]/self.total_distance)

      def getParam(self, position):
            # position is a nd array like [x,y]
            # a is passed in as 

            # creates an array of arrays, may need to be changed later?
            points = np.array([])
            distances = np.array([])
            points_list = []
            
            # Find lower vertex and upper for all vertexes, then find closest points to our current position
            for vertex in range(self.segments): # + 1 because we have 1 more vertex than segments
                  segment_vertex1 = np.array([self.x[vertex], self.y[vertex]])
                  segment_vertex2 = np.array([self.x[vertex+1], self.y[vertex+1]])
                  potential_closest_point = closestPointSegment(position,segment_vertex1,segment_vertex2)
                  
                  points_list.append(potential_closest_point)
                  
                  points = np.append(points,np.array(potential_closest_point))
                  distances = np.append(distances,np.linalg.norm(position - potential_closest_point))
                  #np.linalg.norm(segment_vertex1,segment_vertex2)

            closest_distance = min(distances)
            points = np.array(points_list,dtype=object)
            # This should return an array with 1 value I think, and the index of the closest point
            
            closest_point_index = np.where(distances == closest_distance)[0]
            closest_point = points[closest_point_index]
            current_segment = closest_point_index

            next_segment = current_segment + 1
            # Calculate path parameter of closest point
            # Check if this is a correct array declaration later
            first_vertex = np.array([])
            second_vertex = np.array([])
            first_vertex = np.append(first_vertex,[self.x[current_segment], self.y[current_segment]]) # removing []
            second_vertex = np.append(second_vertex,[self.x[next_segment], self.y[next_segment]]) # removing []
            #second_vertex = np.array([self.x[next_segment], self.y[next_segment]])
            first_vertex_param = self.params[current_segment]
            second_vertex_param = self.params[current_segment+1]

            # Clarifying??
            segment_vector = second_vertex - first_vertex
            point_vector = closest_point[0] - first_vertex

            
            t = np.linalg.norm(closest_point[0] - first_vertex) / np.linalg.norm(second_vertex - first_vertex)
            #t = np.linalg.norm(point_vector - segment_vector) / np.linalg.norm(segment_vector - segment_vector)
            #t = np.dot(point_vector, segment_vector) / np.dot(segment_vector, segment_vector)
            closest_point_param = first_vertex_param + (t * (second_vertex_param - first_vertex_param))
            return closest_point_param


            # Create a vector between each point and our current position, then normalize that
            # Store the distances in an array, then find the smallest distance. This corresponds to the closest point

      def getPosition(self,param):
            index_arr = np.argwhere(self.params < param)[-1]
            
            
            print(index_arr)
            #index_arr = np.where(self.params == param) # In this guy we need to find the index of the vertex param directly below
                                                       # param in value
            index = index_arr[0]
            first_vertex = np.array([self.x[index], self.y[index]])
            second_vertex = np.array([self.x[index+1], self.y[index+1]])
            #for index in range (self):
            #      first_vert = np.array([self.x[index]],[self.y[index]])
            #      sec_vert = np.array([self.x[index+1]],[self.y[index+1]])
            t = (param - self.params[index]) / (self.params[index+1]-self.params[index])
            position = first_vertex + (t *(second_vertex-first_vertex))
            return position

class SteeringOutput(object):
      def _init_(self):
            self.linear = np.array([0.0,0.0])
            self.angular = 0.0

class Character(object):
    def __init__(self):
            self.id = 0
            self.steer = 2
            #self.behavior = behavior
            self.position = np.array([0.0,0.0])
            self.velocity = np.array([0.0,0.0])
            self.linear = np.array([0.0,0.0])
            self.orientation = 0.0
            self.rotation = 0.0
            self.angular_acceleration = 0.0
            self.max_velocity = 0.0
            self.max_acceleration = 0.0
            self.target = self
            self.arrival_radius = 0.0
            self.slowing_radius = 0.0
            self.time_to_target = 0.0
            self.colcollide = False
            self.path_to_follow = 0
            self.path_offset = 0.04

def logRecord(character, time_step):
      path = "data.txt"
      f = open(path,"a")
      
      f.write(str(time_step) + ',')
      f.write(str(character.id) + ',')
      f.write(str(character.position[0]) + ',' + str(character.position[1]) + ',') # pos x & z
      f.write(str(character.velocity[0]) + ',' + str(character.velocity[1]) + ',') # vel x & z
      f.write(str(character.linear[0]) + ',' + str(character.linear[1]) + ',') # linear x & z
      f.write(str(character.orientation)+ ',')
      f.write(str(character.steer) + ',')
      f.write(str(character.colcollide))
      f.write('\n')


def length(vector):
      pass
def normalize(vector):
      norm = np.linalg.norm(vector)
      if norm == 0:
            return vector
      return vector / norm

def distanceBetweenPoints(a, b):
      return np.linalg.norm(b - a) # mebbe 
      #return math.sqrt((a[1] - a[0])^2 + (b[1] - b[0])^2)
      

def closestPointSegment(q, a, b):
      # Makes sure all these operations work with numpy arrays
      t = (np.dot((q - a),(b - a))) / np.dot((b - a) , (b - a))
      #return (a + (t*(b-a)))
      
      if t <= 0:
            return a
      elif t >= 1:
            return b
      else:
            return a + t*(b - a)

def getSteeringSeek(Character, Target):
      # Create output structure
      result = SteeringOutput()
      # Get the direction to the target
      result.linear = Target - Character.position
      # Accelerate at maximum rate
      result.linear = normalize(result.linear)
      result.linear *= Character.max_acceleration
      # Output steering
      result.angular = 0
      return result
      result = SteeringOutput()

      result.linear = Target.position - Character.position


def testSteering(moveType):
      char1 = Character()
      np.put(char1.position,[0,1],[0.0,0.0])
      #print(char1.position)

      tar1 = Character()
      np.put(tar1.position,[0,1],[2.0,5.0])
      #print(tar1.position)
      #tar1.position = [-8.0,2.0]
      #tar1.position.shape

      match moveType:
            case MoveType.Continue.value:
                  print(getSteeringContinue(char1, tar1).linear)
            case MoveType.Seek.value:
                  print(getSteeringSeek(char1, tar1).linear)
            case MoveType.Flee.value:
                  print(getSteeringFlee(char1, tar1).linear)
            case MoveType.Arrive.value:
                  print(getSteeringArrive(char1, tar1).linear)

#testSteering(MoveType.Arrive.value)   # Test if your steering function works. 
                              # Pass in an enum for which function you want to test

def movementUpdate(steering: SteeringOutput, time: float, character: Character ):
      # x - > change in position
      # y - > change in position
      # x - > change in velocity
      # y - > change in velocity

      # Make calculations numpy style

      # Update the position and orientation
      character.position += character.velocity * time
      character.orientation += character.rotation * time
      
      #np.multiply(steering.linear, time)
      # Update the velocity and rotation
      character.velocity += steering.linear * time
      character.rotation += steering.angular * time
      # Check for speed above max and clip
      if np.linalg.norm(character.velocity) > character.max_velocity:
            character.velocity = normalize(character.velocity)
            character.velocity *= character.max_velocity
      return

def main():
      numOfCharacters = 1
      characters = []
      for i in range(numOfCharacters):
            characters.append(Character())

      # Initialize Character 2 FOLLOW
      characters[0].id = 2701
      characters[0].steer = 1
      np.put(characters[0].position,[0,1],[20.0, 95])
      np.put(characters[0].velocity,[0,1],[0,0])
      characters[0].max_velocity = 4
      characters[0].max_acceleration = 2
      characters[0].path_to_follow = 1
      characters[0].path_offset = 0.04
      
      X = (0,-20,20,-40,40,-60,60,0)
      Y = (90,65,40,15,-10,-35,-60,-85)

      path = Path()
      path.pathAssemble(characters[0].id, X,Y)

      time_step_length = 250
      for time_step in range(time_step_length):
            
            for character in characters:
                  logRecord(character, time_step_length)
                  output = SteeringOutput
                  current_param = path.getParam(character.position)

                  target_param = current_param + character.path_offset
                  target_position = path.getPosition(target_param)
                  output = getSteeringSeek(character, target_position)
      
                  movementUpdate(steering=output, time=0.5, character=character)
      
main()
