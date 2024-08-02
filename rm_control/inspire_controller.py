from robotic_arm_package.robotic_arm import *
import time
import json
import socket

class realman_json:
    def __init__(self, ip, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))

    def get_current_arm_state(self):
        get_state = '{"command":"get_current_arm_state"}\r\n'
        self.client.send(get_state.encode('utf-8'))
        data = self.client.recv(1024).decode()
        time.sleep(0.5)
        return data
    
    def close_modbus_mode(self):
        close_modbus = '{"command":"close_modbus_mode","port":0}\r\n'
        self.client.send(close_modbus.encode('utf-8'))
        data = self.client.recv(1024).decode()
        print(data)
        return data
    
    def set_modbus_rtu(self, port, baudrate, timeout):
        # port = 1
        # baudrate = 115200
        # timeout = 5
        set_modbus_rtu = '{"command":"set_modbus_mode", "port":' + str(port) + ', "baudrate":' + str(baudrate) + ', "timeout":' + str(timeout) + '}\r\n'
        self.client.send(set_modbus_rtu.encode('utf-8'))
        data = self.client.recv(1024).decode()
        time.sleep(0.5)
        return data
    
    def set_hand_posture(self, posture_id):
        set_hand_posture = '{"command":"set_hand_posture","posture_num": '+ str(posture_id) + '}\r\n'
        self.client.send(set_hand_posture.encode('utf-8'))
        data = self.client.recv(1024).decode()
        print(data)
        return data
    
    def set_finger_position(self, position):
        set_finger_position = '{"command":"set_hand_angle","hand_angle":'+ str(position) +'}\r\n'
        print(set_finger_position)
        self.client.send(set_finger_position.encode('utf-8'))
        data = self.client.recv(1024).decode()
        print(data)
        return data
    
    def read_finger(self):
        read_position = '{"command":"read_multiple_holding_registers","port":1,"address":1546,"num":6,"device":1} \r\n'
        self.client.send(read_position.encode('utf-8'))
        data = self.client.recv(1024).decode()
        print(data)
        return data
    
    def read_finger_degrees(self):
        data = self.read_finger()
        data = json.loads(data)
        print(data)
        # if data['read_state'] == False:
        #     print('Read failed')
        #     return None
        degree = data['data']

        angle_values = []
        
        for i in range(0, len(degree), 2):
            # 从高位字节和低位字节创建16位数值
            high_byte = degree[i]
            low_byte = degree[i + 1]
            combined_value = (high_byte << 8) | low_byte
            
            # 将16位数值添加到角度值列表
            angle_values.append(combined_value)
        
        print(angle_values)
        return angle_values
    
if __name__ == "__main__":

    ip = '192.168.31.106'
    port = 8080
    realman = realman_json(ip, port)
    # print(realman.set_modbus_rtu(1, 115200, 5))
    # realman.close_modbus_mode()
    realman.set_hand_posture(1)
    # time.sleep(0.5)

    # position = [500,500,500,500,500,-1]
    position = [900,900,900,900,900,900]
    # realman.set_finger_position(position)
    # time.sleep(0.5)
    
    realman.read_finger()