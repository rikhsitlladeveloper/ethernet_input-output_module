#!/usr/bin/env python3
import rospy
from ethernet_remote_io_module.msg import WriteCoil, WriteCoils, ReadDigitalInputs

def callback(data):
    rospy.loginfo(data.din_1)
    if (data.din_1 == True):
        pub = rospy.Publisher('write_coil', WriteCoil, queue_size=10)
        address =0
        value= True
        pub.publish(address,value)
        print("True")
    else:
        pub = rospy.Publisher('write_coil', WriteCoil, queue_size=10)
        address =0
        value= False
        pub.publish(address,value)
        print("False")

def listener():
    rospy.init_node('listen_konsevik', anonymous=True) 
    rospy.Subscriber("/read_digital_inputs", ReadDigitalInputs, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()
