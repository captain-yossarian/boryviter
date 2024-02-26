import time
from connection import Connection

print("connection to RPI")
connection = Connection("simulator")
time.sleep(1)
connection.position()
time.sleep(1)

print("Learning model loaded")
