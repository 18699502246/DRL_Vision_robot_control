import time
# from zmqRemoteApi import RemoteAPIClient
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def myFunc(input1, input2):
    print('Hello', input1, input2)
    return 21

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')


# Run a simulation in asynchronous mode:
# sim.startSimulation()

