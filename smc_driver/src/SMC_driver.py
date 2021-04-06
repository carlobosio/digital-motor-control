#!/usr/bin/env python
# coding: utf-8

import numpy as np
import socket

class SMC_driver:

    def __init__(self,IP_config = ("192.168.0.10",9999,"192.168.0.100",11111)):
    
        self.micro_IP = IP_config[0]
        self.micro_port = IP_config[1]
        self.personal_IP = IP_config[2]
        self.personal_port = IP_config[3]
        
        # user specified attributes
        
        self.send_buttons = np.array([0,0,0,0], np.uint8)
        self.send_axis = np.single(0.0)
        self.send_control_mode_button = np.uint8(0)
        self.send_position = np.single(0.0)
        
        # echo attributes
        
        self.receive_buttons = np.array([0,0,0,0], np.uint8)
        self.receive_axis = np.single(0.0)
        self.receive_control_mode = np.uint8(0)
        self.receive_position = np.single(0.0)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.personal_IP, self.personal_port))
        
        
    def __str__(self):
        return 'smc_driver(' + str(self.micro_IP) + ',' + str(self.micro_port) + ')\n'
        
        
    def __repr__(self):
        print('smc_driver(' + str(self.micro_IP) + ',' + str(self.micro_port) + ')\n')
        
        
    def print_attributes(self):
        print('Desired attributes:\n')
        print('Buttons:' + str(self.send_buttons[0])+' '+str(self.send_buttons[1])+' '+ str(self.send_buttons[2])+' '+str(self.send_buttons[3])+' '+str(self.send_control_mode_button)+'\n')
        print('Axes:' + str(self.send_axis) + '\n')
        print('Position:'+ str(self.send_position) + '\n')
        
        print('\n')
        
        print('Current attributes:\n')
        print('Buttons:' + str(self.receive_buttons[0])+' '+str(self.receive_buttons[1])+' '+ str(self.receive_buttons[2])+' '+str(self.receive_buttons[3])+'\n')
        print('Axis:' + str(self.receive_axis) + '\n')
        print('Control mode:'+ str(self.receive_control_mode) + '\n')
        print('Position:'+ str(self.receive_position) + '\n')
        
        print('\n')
        
        
    def set_desired_position(self,dp):
        self.send_position = np.single(dp)
        
        
    def set_axis(self,x):
        self.send_axis = np.single(x)
        
        
    def set_buttons(self,b):
        self.send_buttons[0] = np.uint8(b[0])
        self.send_buttons[1] = np.uint8(b[1])
        self.send_buttons[2] = np.uint8(b[2])
        self.send_buttons[3] = np.uint8(b[3])
        self.send_control_mode_button = np.uint8(b[4])
        
        
    def get_current_position(self):
        return self.receive_position
        
        
    def get_control_mode(self):
        return self.receive_control_mode
        
        
    def send_to_driver(self):        
        buttons_bytes = self.send_buttons.tobytes()
        axes_bytes = self.send_axis.tobytes()
        control_mode_button_bytes = self.send_control_mode_button.tobytes()
        position_bytes = self.send_position.tobytes()
        
        output_bytes = buttons_bytes + axes_bytes + control_mode_button_bytes + position_bytes

        self.sock.sendto(output_bytes, (self.micro_IP, self.micro_port))
        
        
    def receive_from_driver(self):
        
        data, _ = self.sock.recvfrom(13)
        self.receive_buttons = np.frombuffer(data, dtype=np.uint8, count=4)
        self.receive_axis = np.frombuffer(data, dtype=np.single, count = 1, offset = 4)
        self.receive_control_mode = np.frombuffer(data, dtype=np.uint8, count = 1, offset = 8)
        self.receive_position = np.frombuffer(data, dtype=np.single, count = 1, offset = 9)
        
        
    def driver_echo(self):
        
        print('Current attributes:\n')
        print('Buttons:' + str(self.receive_buttons[0])+' '+str(self.receive_buttons[1])+' '+ str(self.receive_buttons[2])+' '+str(self.receive_buttons[3])+' '+str(self.send_control_mode_button)+'\n')
        print('Axis:' + str(self.receive_axis) + '\n')
        print('Control mode:'+ str(self.receive_control_mode) + '\n')
        print('Position:'+ str(self.receive_position) + '\n')
        print('\n')
        
