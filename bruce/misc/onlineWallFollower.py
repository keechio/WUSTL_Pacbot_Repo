#https:#github.com/Rajneesh04/Wall-follower/blob/master/Wallfollower/pidwallfollower3.0/pidwallfollower3.0.ino
SENSOR_MAX 
DIST_PROPORTIONAL_= 0.034/2 #DEPENDS ON ENVIRONMENT AND SPEED OF SOUND
INPUT_DISTANCE = 15 #in cm
ERROR_DIST = 5  #in cm
SPEED = 95
MAX_SENSOR_VALUE = 10000
MAX_OF_SENSOR = 3000
MAX_DIFFERNCE = 10
MAX_TURN_SPEED = 10
CALIBRATION = 3
MAX_ALLIGN_ANGLE = 5
COLLISION_DISTANCE = 10

#sensor 1 pins
trigPin1 = 10
echoPin1 = 11

#sensor 2 pins
trigPin2 = 12
echoPin2 = 13

#sensor 3 pins : front sensor
trigPin3 = 6
echoPin3 = 2

#right motor pins
in1R = 7
in2R = 8
enR = 9

#left motor pins
in1L = 4
in2L = 5
enL = 3

#pid
kp_a = 1
kd_a = 0
#tuning parameters
kp_d = 1
kd_d = 0
ki_d = 0
#distance
errorD
previousErrorD = 0
dt = 0.1


distance1, distance2, currentDistance, integral, derivative, outputD, outputA, angle, previousAngle=0.0, allign_angle=0.0
region, pidDist, speedL, speedR, distance

#reading sensor
def sensor_output( trigPin,  echoPin):
  digitalWrite(trigPin, LOW)
  delayMicroseconds(2)

  digitalWrite(trigPin, HIGH)
  delayMicroseconds(10)
  digitalWrite(trigPin, LOW)

  float duration = pulseIn(echoPin, HIGH)
  print("duration:")
  println(duration)
  if(duration==0.0||duration>=MAX_SENSOR_VALUE):
    duration = MAX_OF_SENSOR
    
    

  float distance = duration*DIST_PROPORTIONAL_CONST
   
  return distance 


#calculates distance from wall
def current_distance( read1,  read2):
  float distance = (read1 + read2)/2  #taking average of two values

  return distance
  

#which direction to turn
def check_region( read1,  read2):
  distance = current_distance(read1, read2)
  if(abs(distance-INPUT_DISTANCE)>ERROR_DIST):
    if(distance > INPUT_DISTANCE):
        return -1 #for left movement
    else:
        return 1 #for right movement
  else:
    return 0 #inside region
      
  

#steers the robot to reach at the specified distance
def reach_distance():
  distance1 = sensor_output(trigPin1, echoPin1)
  distance2 = sensor_output(trigPin2, echoPin2)
  currentDistance = current_distance(distance1, distance2)
  errorD = currentDistance - INPUT_DISTANCE
  derivative = errorD - previousErrorD
  integral += errorD
  outputD = kp_d*errorD + ki_d*integral*dt + kd_d*derivative
  previousErrorD = errorD
  speedL = SPEED - (int)outputD
  speedR = SPEED + (int)outputD
  #right turn 
  if((speedL-speedR)>MAX_DIFFERNCE){
    speedL = SPEED + MAX_DIFFERNCE
    speedR = SPEED - MAX_DIFFERNCE
  }else if((speedL-speedR)<(-1)*MAX_DIFFERNCE){
    speedL = SPEED - MAX_DIFFERNCE
    speedR = SPEED + MAX_DIFFERNCE
    }
   analogWrite(enL, speedL)
   analogWrite(enR, speedR)
   digitalWrite(in1L, HIGH)
    digitalWrite(in2L, LOW)
    digitalWrite(in1R, HIGH)
    digitalWrite(in2R, LOW)
  
  

#keep the robot alingned to the wall and gives a stable path

def follow_wall():
  distance1 = sensor_output(trigPin1, echoPin1)
  distance2 = sensor_output(trigPin2, echoPin2)
  angle = distance2-distance1#if +ve turn left else turn right
  derivativeA = angle - previousAngle
  outputA = kp_a*angle + kd_a*derivativeA
  previousAngle = angle
  speedL = SPEED + CALIBRATION - outputA
  speedR = SPEED + outputA
  if((speedL-speedR)>MAX_TURN_SPEED){
    speedL = SPEED + CALIBRATION + MAX_TURN_SPEED
    speedR = SPEED - MAX_TURN_SPEED
    }else if((speedL-speedR)>MAX_TURN_SPEED){
      speedL = SPEED + CALIBRATION - MAX_TURN_SPEED
      speedR = SPEED + MAX_TURN_SPEED
      }
  analogWrite(enL, speedL)
  analogWrite(enR, speedR)
  digitalWrite(in1L, HIGH)
  digitalWrite(in2L, LOW)
    digitalWrite(in1R, HIGH)
    digitalWrite(in2R, LOW)
  
  

def check_collision():
  distance3 = sensor_output(trigPin3, echoPin3)
  if(distance3<COLLISION_DISTANCE):
    return true
  else:
    return false
  

def setup():
  begin(9600)
  pinMode(in1L,OUTPUT)
  pinMode(in2L,OUTPUT)
  pinMode(in1R,OUTPUT)
  pinMode(in2R,OUTPUT)
  pinMode(enL,OUTPUT)
  pinMode(enR,OUTPUT)                          
  
  pinMode(trigPin1,OUTPUT)
  pinMode(echoPin1,INPUT)
  pinMode(trigPin2,OUTPUT)
  pinMode(echoPin2,INPUT)

  digitalWrite(enL, LOW)
  digitalWrite(enR, LOW)


def loop():
  distance1 = sensor_output(trigPin1, echoPin1)
  distance2 = sensor_output(trigPin2, echoPin2)
  region = check_region(distance1, distance2)
  allign_angle = abs(distance2 - distance1)
  print("distance1:")
  print(distance1)
  print("distance2:")
  print(distance2)
  print("region:")
  print(region)
  if(check_collision()){
    #turn right
    speedL = speedL + MAX_DIFFERNCE
    speedR = speedR - MAX_DIFFERNCE
    analogWrite(enL, speedL)
    analogWrite(enR, speedR)
    digitalWrite(in1L, HIGH)
    digitalWrite(in2L, LOW)
    digitalWrite(in1R, HIGH)
    digitalWrite(in2R, LOW)
  }else{
    if(region==0){
    follow_wall()
    }
   else{
    if(allign_angle>MAX_ALLIGN_ANGLE){
      follow_wall()
      }else{
        reach_distance()
      }
    
    }

  }
  
