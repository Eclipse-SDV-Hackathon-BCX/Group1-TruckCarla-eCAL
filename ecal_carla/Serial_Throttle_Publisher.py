import sys
import time

import serial

import ecal.core.core as ecal_core
from ecal.core.publisher import StringPublisher

Serial_Port = 'COM3'

if __name__ == "__main__":
    # initialize eCAL API. The name of our Process will be "Python Hello World Publisher"
    ecal_core.initialize(sys.argv, "Python Serial Throttle Publisher")

    # Create a String Publisher that publishes on the topic "hello_world_python_topic"
    pub = StringPublisher("SerialThrottleString")

    # Open Serial Port to receive Arduino Throttle Data (Running Average over 5)
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = Serial_Port
    ser.timeout = 2
    if ser.open():
        print("Opened COM Port")
    time.sleep(6)
    ser.flushInput()

    # Infinite loop (using ecal_core.ok() will enable us to gracefully shutdown
    # the process from another application)
    while ecal_core.ok():
        # Create a message with a counter an publish it to the topic
        ser_bytes = ser.readline()
        decoded_bytes = float(ser_bytes[0:len(ser_bytes) - 2].decode("utf-8"))
        current_message = str(decoded_bytes)
        print("Sending: {}".format(current_message))
        pub.send(current_message)

    # finalize eCAL API
    ecal_core.finalize()