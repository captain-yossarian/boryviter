import time
from connection import Connection

print("connection to RPI")
connection = Connection("simulator")

print("turn")
connection.turn_from(-1)


print("Learning model loaded")
