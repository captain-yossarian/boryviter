import time
from connection import Connection


def first_arm():
    connection.arm()
    time.sleep(5)
    connection.takeoff(10)
    time.sleep(2)


connection = Connection("simulator")
# first_arm()
# connection.turn_from(1)
connection.fly_forward(10)
# time.sleep(2)
