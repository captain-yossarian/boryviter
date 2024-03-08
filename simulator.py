import time
from connection import Connection

print("connection to RPI")
connection = Connection("simulator")


connection.fly_forward()


print("Learning model loaded")
