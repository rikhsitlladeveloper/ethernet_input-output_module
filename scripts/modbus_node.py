#!/usr/bin/env python3
import re
from typing import List
import rospy
from pymodbus.client.sync import ModbusTcpClient
from ethernet_remote_io_module.msg import WriteCoil, WriteCoils, ReadDigitalInputs
from rospy import loginfo, logerr, logwarn, loginfo_once

def validate_ip(ip: str) -> bool:
    """[Validating IP address]
    Args:
        ip (str): [IP address]
    Returns:
        bool: [True if validation succeeded, False otherwise]
    """
    regex = r"^((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])$"
    return bool(re.search(regex, ip))


def shutdown_hook() -> None:
    """[Executes on shutdown, sets all DOs to False and closes modbus connection]
    """
    if CONNECT_FLAG:
        flag = client.write_coils(address=0, values=[False]*8, unit=0x01)
        if flag:
            client.close()
    else:
        pass

class RemoteIO:
    """
        ROS wrapper for modbus client of remote IO module

        Note: wrapper reads/writes only digital inputs/outputs
    """

    def __init__(self,tcp_client: ModbusTcpClient) -> None:
        """[Initialize a wrapper instance]

        Args:
            tcp_client (ModbusTcpClient): [instance of a modbus tcp client]

        Note: other parameters retrieves from rosparam server
        """
        self._client = tcp_client
        self.freq = rospy.get_param('~rate', default=10)
        self.__rate = rospy.Rate(self.freq)
        self.__pub_topic = rospy.get_param('~inputs_topic_name', default='read_digital_inputs')
        self.__single_sub_topic = rospy.get_param('~single_coil_sub_topic_name', default='write_coil')
        self.__multi_sub_topic = rospy.get_param('~milti_coils_sub_topic_name', default='write_coils')
        self.__inputs_publisher = rospy.Publisher(self.__pub_topic, ReadDigitalInputs, queue_size=10, latch=True)
        rospy.Subscriber(self.__single_sub_topic, WriteCoil, callback=self.__write_coil_clbk)
        rospy.Subscriber(self.__multi_sub_topic, WriteCoils, callback=self.__write_coils_clbk)
        self.__start_publish()

    def set_coil(self, address: int, value: bool, unit=0x01) -> bool:
        """
        Args:
            address (int): [The address to write to]
            value (bool): [The value to write to the specified address]
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            bool: [A deferred response handle ]
        """
        res = self._client.write_coil(address=address, value=value, unit=unit)
        if not res.isError():
            loginfo(f'Successfully set to DO{address+1} Value = {value}')
            return True
        else:
            logerr(f'Unable to set to DO{address+1}')
            return False

    def set_coils(self, start_address: int, value: list, unit=1) -> bool:
        print(value)
        """
        Args:
            start_address (int): [The starting address to write to ]
            end_address (int): [The last address to write to]
            value (bool): [The value to write to the specified address]
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            bool: [A deferred response handle ]
        """
        res = self._client.write_coils(address=start_address, values=value, unit=unit)
        if not res.isError():
            loginfo(f'Successfully set to DO{start_address+1}- DO 8 Values = {value}')
            return True
        else:
            logerr(f'Unable to set to DO{start_address}- DO 8')
            return False

    def __write_coil_clbk(self, msg: WriteCoil) -> None:
        """[Callback function for Subscriber]
        Args:
            msg (WriteCoil): [data]
        """
        address = 0
        value = msg.value
        self.set_coil(address=address, value=value)

    def __write_coils_clbk(self, msg: WriteCoils) -> None:
        """[Callback function for Subscriber]
        Args:
            msg (WriteCoils): [data]
        """
        
        
        value = msg.value
        self.set_coils(0, value=value)

    def get_inputs(self, address: int=0, count: int=8, unit=0x01) -> List[bool]:
        """
        Args:
            address (int, optional): [The starting address to read from]. Defaults to 0.
            count (int, optional): [The number of discretes to read]. Defaults to 8.
            unit (hexadecimal, optional): [The slave unit this request is targeting]. Defaults to 0x01.

        Returns:
            List[bool]: [If response is ok returns states of DINs otherwise empty list]
        """
        res = self._client.read_discrete_inputs(address=address, count=count, unit=unit)
        if not res.isError():
            loginfo_once(f'Reading inputs DIN{address+1} - DIN{count}')
            if all(isinstance(val, bool) for val in res.bits[address:count]):
                return res.bits[address:count]
            else:
                return []
        else:
            logerr(f'Unable to read inputs DIN{address+1} - DIN{count}')
            return []

    def __start_publish(self) -> None:
        """[Wraps DINs states to ros msg and publishes to topic]
        """
        while not rospy.is_shutdown():
            din_states = self.get_inputs()
            pub_msg = ReadDigitalInputs()
            if din_states:
                pub_msg.all_inputs = din_states
                self.__inputs_publisher.publish(pub_msg)
                self.__rate.sleep()

            else:
                logerr('Unable to read inputs')

if __name__ == '__main__':
    rospy.init_node('remote_io_node', anonymous=True, disable_signals=True)
    rospy.on_shutdown(shutdown_hook)
    CONNECT_FLAG = False
    # get private parameters
    if not rospy.has_param('~ip_address'):
        logwarn('IP address of modbus server not specified')
        logwarn('Try to connect to default address')
    if not rospy.has_param('~port'):
        logwarn('Modbus server port not specified')
        logwarn('Using default port')
    ip_address = rospy.get_param('~ip_address', default='192.168.1.110')
    port = rospy.get_param('~port', default=502)
    # validating IP address
    if validate_ip(ip_address):
        loginfo(f'Modbus Server IP= {ip_address} port= {port}')
        loginfo('Connecting...')
        client = ModbusTcpClient(host=ip_address, port=port)
        CONNECT_FLAG = client.connect()
        # check connection
        if CONNECT_FLAG:
            loginfo('Connected successfully')
            remote_io = RemoteIO(client)
        else:
            logerr(f"Can't connect to {ip_address}")
            rospy.signal_shutdown('Wrong Modbus Server IP address or port')
    else:
        logerr(f'IP address validation error. Wrong IP {ip_address}')
        rospy.signal_shutdown('IP address validation error')
    