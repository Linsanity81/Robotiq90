#!/usr/bin/env python
from math import ceil
import time
import threading
from robotiq_msgs.msg import CModelStatus
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadInputRegistersResponse


class ComModbusTcp:
  def __init__(self):
    self.client = None
    self.lock = threading.Lock()
  
  def connectToDevice(self, address):
    """
    Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument.
    """
    self.client = ModbusTcpClient(address)

  def disconnectFromDevice(self):
    """Close connection"""
    self.client.close()

  def sendCommand(self, data):
    """
    Send a command to the Gripper - the method takes a list of int as an argument. 
    The meaning of each variable depends on the Gripper model 
    """
    if (len(data) == 6):
      message = []
      for i in range (0, len(data)):
        message.append( data[i] )

      with self.lock:
        self.client.write_register(52, message[0])
        self.client.write_register(53, message[1])
        self.client.write_register(54, message[2])
        self.client.write_register(3006, message[3])
        self.client.write_register(3007, message[4])
        self.client.write_register(3008, message[5])

  def getStatus(self, numRegs):
    """
    Sends a request to read, wait for the response and returns the Gripper status. 
    The method gets the number of bytes to read as an argument
    """
    # TODO: Implement try/except 
    # Get status from the device
    with self.lock:
      response1 = self.client.read_input_registers(51, numRegs)
      response2 = self.client.read_input_registers(3012, numRegs)
    # Instantiate output1 output2 as an empty list
    output1 = []
    output2 = []
    # Fill the output with the bytes in the appropriate order
    for i in range(0, numRegs):
      output1.append( response1.getRegister(i))
      output2.append( response2.getRegister(i))

    for i in range(0, numRegs):
      output1.append( output2[i] )
    # Output the result
    return output1


class RobotiqCModel:
  def __init__(self):
    #Initiate output message as an empty list
    self.message = []

  def verifyCommand(self, command):
    # Verify that each variable is in its correct range
    command.rACT = max(0, command.rACT)
    command.rACT = min(1, command.rACT)
    
    command.rGTO = max(0, command.rGTO)
    command.rGTO = min(1, command.rGTO)

    command.rATR = max(0, command.rATR)
    command.rATR = min(1, command.rATR)
    
    command.rPR  = max(0,   command.rPR)
    command.rPR  = min(4095, command.rPR)
    
    command.rSP  = max(0,   command.rSP)
    command.rSP  = min(1000, command.rSP)
    
    command.rFR  = max(0,   command.rFR)
    command.rFR  = min(1000, command.rFR)
    
    # Return the cropped command
    return command
    
  def refreshCommand(self, command):
    # Limit the value of each variable
    command = self.verifyCommand(command)
    # Initiate command as an empty list
    self.message = []
    # Build the command with each output variable
    # TODO: add verification that all variables are in their authorized range
    self.message.append(command.rACT)
    self.message.append(command.rGTO)
    self.message.append(command.rATR)
    self.message.append(command.rPR)
    self.message.append(command.rSP)
    self.message.append(command.rFR)

  def sendCommand(self):
    self.client.sendCommand(self.message)

  def getStatus(self):
    """
    Request the status from the gripper and return it in the CModelStatus msg type.
    """
    #Acquire status from the Gripper
    status = self.client.getStatus(4);
    # Message to report the gripper status
    message = CModelStatus()
    #Assign the values to their respective variables
    message.gACT =  status[0]
    message.gGTO =  status[1]
    message.gSTA =  status[2]
    message.gOBJ =  status[3]
    message.gFLT =  status[4]
    message.gPR  =  status[5]
    message.gPO  =  status[6]
    message.gCU  =  status[7]
    return message
