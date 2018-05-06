#!/usr/bin/env python
import rospy
from robotiq_msgs.msg import CModelCommand
from time import sleep

def genCommand(char, command):
  """Update the command according to the character entered by the user."""    
  if char == 'o':
    command = CModelCommand();
    command.rACT = 0
    command.rGTO = 1
    command.rATR = 1
    command.rPR  = 4095
    command.rSP  = 400
    command.rFR  = 200

  if char == 'r':
    command = CModelCommand();
    command.rACT = 0
    command.rGTO = 1
    command.rATR = 1
    command.rPR  = 2000
    command.rSP  = 400
    command.rFR  = 200

  if char == 'c':
    command.rACT = 0
    command.rGTO = 1
    command.rATR = 1
    command.rPR  = 0
    command.rSP  = 400
    command.rFR  = 200

  #If the command entered is a int, assign this value to rPRA
  try: 
    command.rPR = int(char)
    if command.rPR > 4095:
      command.rPR = 4095
    if command.rPR < 0:
      command.rPR = 0
  except ValueError:
    pass

  if char == 'f':
    command.rSP += 25
    if command.rSP > 1000:
      command.rSP = 1000

  if char == 'l':
    command.rSP -= 25
    if command.rSP < 0:
      command.rSP = 0

  if char == 'i':
    command.rFR += 25
    if command.rFR > 1000:
      command.rFR = 1000

  if char == 'd':
    command.rFR -= 25
    if command.rFR < 0:
      command.rFR = 0
  return command

def askForCommand(command):
  """
  Ask the user for a command to send to the gripper.
  """
  currentCommand  = 'Simple C-Model Controller\n-----\nCurrent command:'
  currentCommand += '  SAFETY_MODE = '  + str(command.rACT)
  currentCommand += ', MEASURE = '  + str(command.rGTO)
  currentCommand += ', MEASURE_INPUT = '  + str(command.rATR)
  currentCommand += ', POSITION = '   + str(command.rPR )
  currentCommand += ', VELOCITY = '   + str(command.rSP )
  currentCommand += ', FORCE = '   + str(command.rFR )
  print currentCommand
  strAskForCommand  = '-----\nAvailable commands\n\n'
  strAskForCommand += 'r: Reset\n'
  strAskForCommand += 'c: Close\n'
  strAskForCommand += 'o: Open\n'
  strAskForCommand += '(0-4095): Go to that position\n'
  strAskForCommand += 'f: Faster\n'
  strAskForCommand += 'l: Slower\n'
  strAskForCommand += 'i: Increase force\n'
  strAskForCommand += 'd: Decrease force\n'
  strAskForCommand += '-->'
  return raw_input(strAskForCommand)

def publisher():
  """
  Main loop which requests new commands and publish them on the CModelRobotOutput topic.
  """
  rospy.init_node('robotiq_simple_controller')
  pub = rospy.Publisher('command', CModelCommand, queue_size=3)
  command = CModelCommand()
  while not rospy.is_shutdown():
    command = genCommand(askForCommand(command), command)
    pub.publish(command)
    rospy.sleep(0.1)

if __name__ == '__main__':
  publisher()
