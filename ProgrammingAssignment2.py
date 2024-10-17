import numpy as np
import math
from enum import Enum

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
      def _init_(self):
            self.x = np.array([])
            self.y = np.array([])
            self.params = np.array([])
            self.distance = np.array([])
            self.segments = 0
      def pathAssemble(self,ID, X,Y):

      def getParam(self, position):

      def getPosition(self,param):

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
            self.pat_offset = 0.0

def logRecord(characters, time_step):
      path = "data.txt"
      f = open(path,"a")
      for character in characters:
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

def getSteeringSeek(Character, Target):
      # Create output structure
      result = SteeringOutput()
      # Get the direction to the target
      result.linear = Target.position - Character.position
      # Accelerate at maximum rate
      result.linear = normalize(result.linear)
      result.linear *= Character.max_acceleration
      # Output steering
      result.angular = 0
      return result
      result = SteeringOutput()

      result.linear = Target.position - Character.position

def getSteeringFlee(Character, Target):
      # Create output structur
      result = SteeringOutput()
      #Get the direction to the target
      result.linear = Character.position - Target.position
      #result.linear = np.subtract(Character.position, Target.position)
      # Accelerate at maximum rate
      result.linear = normalize(result.linear)
      result.linear *= Character.max_acceleration
      # Output steering
      result.angular = 0
      return result
      #result.angular = 0
      return result
      
      

def getSteeringArrive(Character, Target):

      #if Character.position[0] == Target.position[0] and Character.position[1] == Target.position[1]:
      #      Character.colcollide = True
      
      #if Character.colcollide:
      #      return -Character.velocity

      result = SteeringOutput()
                  # Get the direction and distance to the target
      direction = np.subtract(Target.position, Character.position) ## Check if these need to be switched later
      distance = np.linalg.norm(direction) # Magnitude
                  # Test for arrival
      if distance < Character.arrival_radius:
            #result.linear = 0
            #result.angular
            return result
                   # Outside slowing-down (outer) radius, move at max speed
      if distance > Character.slowing_radius: ### Not sure if this needs to be target or character slow radius
            targetSpeed = Character.max_velocity
      # Between radii, scale speed to slow down
      else:
            targetSpeed = Character.max_velocity * distance / Character.slowing_radius
                  # Target velocity combines speed and direction
      targetVelocity = direction
      targetVelocity = normalize(targetVelocity)
      targetVelocity *= targetSpeed
                  # Accelerate to target velocity
      result.linear = np.subtract(targetVelocity, Character.velocity)
      result.linear /= Character.time_to_target
                  # Test for too fast acceleratio
      if np.linalg.norm(result.linear) > Character.max_acceleration:
            result.linear = normalize(result.linear)
            result.linear *= Character.max_acceleration

      # Change might be wrong so commenting this out
      # if result.linear.length() > maxAcceleration:
      #      result.linear = normalize(result.linear)
      #      result.linear *= maxAcceleration

      # Output steering
      #result.angular = 0
      return result

"""
def getSteeringArrive(Character, Target):
      result = SteeringOutput()
                  # Get the direction and distance to the target
      direction = np.subtract(Target.position, Character.position) ## Check if these need to be switched later
      distance = np.linalg.norm(direction) # Magnitude
                  # Test for arrival
      if distance < targetRadius:
            return None
                   # Outside slowing-down (outer) radius, move at max speed
      if distance > Target.slowing_radius: ### Not sure if this needs to be target or character slow radius
            targetSpeed = maxSpeed
      # Between radii, scale speed to slow down
      else:
            targetSpeed = maxSpeed * distance / Target.slowing_radius
                  # Target velocity combines speed and direction
      targetVelocity = direction
      targetVelocity = normalize(targetVelocity)
      targetVelocity *= targetSpeed
                  # Accelerate to target velocity
      result.linear = np.subtract(targetVelocity, Character.velocity)
      result.linear /= timeToTarget
                  # Test for too fast acceleratio
      if np.linalg.norm(result.linear) > maxAcceleration:
            result.linear = normalize(result.linear)
            result.linear *= maxAcceleration

      # Change might be wrong so commenting this out
      # if result.linear.length() > maxAcceleration:
      #      result.linear = normalize(result.linear)
      #      result.linear *= maxAcceleration

      # Output steering
      #result.angular = 0
      return result
"""

def getSteeringContinue(Character, Target):
      #r code
      #result < - list(linear=mover$linear, angular=mover$angular)
      #return(result)
      result = SteeringOutput
      result.linear = Character.linear
      result.angular = Character.orientation ### This may be angular acceleration, but for now we'll say orientation
      
      return result

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
      numOfCharacters = 2
      characters = []
      for i in range(numOfCharacters):
            characters.append(Character())

      # Initialize Character 1 CONTINUE
      characters[0].id = 2601
      characters[0].steer = 1

      # Initialize Character 2 FOLLOW
      characters[1].id = 2701
      characters[1].steer = 1
      np.put(characters[1].position,[0,1],[20.0, 95])
      np.put(characters[1].velocity,[0,1],[0,0])
      characters[1].max_velocity = 4
      characters[1].max_acceleration = 2
      characters[1].path_to_follow = 1
      characters[1].path_offset = 0.04
      
      X = (0,-20,20,-40,40,-60,60,0)
      Y = (90,65,40,15,-10,-35,-60,-85)

      time_step_length = 50
      for time_step in range(time_step_length):
            logRecord(characters, time_step_length)
            for character in characters:
                  output = SteeringOutput
                  
                  match character.steer:
                        case MoveType.Continue.value:
                              output = getSteeringContinue(character, character.target)
                        case MoveType.Seek.value:
                              output = getSteeringSeek(character, character.target)
                        case MoveType.Flee.value:
                              output = getSteeringFlee(character, character.target)
                        case MoveType.Arrive.value:
                              output = getSteeringArrive(character, character.target)
                  
                  movementUpdate(steering=output, time=0.5, character=character)
            
            
            

      
main()
