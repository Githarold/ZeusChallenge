#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import * 


from socket import *
import _socket
class tcp_server():
    
    def __init__(self, host="192.168.0.23", port=9000):
        
        # 192.168.0.23 : robot controller ip address
        # you must set port number larger than 1024
        self.listen_socket = socket(AF_INET, SOCK_STREAM)
        
        # exception이 나면 1초 기다리고 바로 포트를 할당할 수 있게 함
        self.listen_socket.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        self.listen_socket.bind((host, port))
        self.listen_socket.listen(1)
        
    
    def connect_client(self):
        
        self.client_socket, self.client_address = self.listen_socket.accept()
        
        if self.client_socket:
            return self.client_socket
        
        return None
        
    def close_client_connection(self):
        self.client_socket.close()
        
    def close_server(self):
        self.listen_socket.close()
        
    def get_data(self):
        data = self.client_socket.recv(1024)
        data = eval(data)
        print data
        get_flag = 1
        return data , get_flag
    
    def send_data(self, curr_pos):
        a = curr_pos
        print "send_data"
        # print curr_pos
        data = str(a).encode()
        self.client_socket.send(data)

class move_zeus():
    
    def __init__(self):
        ## 2. 초기 설정② ####################################
        # ZERO 로봇 생성자
        # offset_4 = 272.0
        offset_4 = 260.0
        # offset_5 = 370.0
        # offset_3 = 260.0 - 60.0
        self.rb = i611Robot()
        
        # 좌표계의 정의
        _BASE = Base()
        # 로봇과 연결 시작 초기화
        self.rb.open()
        # I/O 입출력 기능의 초기화 
        IOinit( self.rb )
        # self.rb.settool(id = 3, offz=offset_3)
        self.rb.settool(id = 4, offz=offset_4)
        # self.rb.settool(id = 5, offz=offset_5)

        self.rb.changetool(4)  
        # self.robot_pose = [0,0,0,0,0,0]
        # self.m = MotionParam(jnt_speed=50, lin_speed=300, overlap=30, acctime=1, dacctime=1)
        self.m = MotionParam(jnt_speed=20, lin_speed=30, overlap=30, acctime=1, dacctime=1)
        self.rb.motionparam( self.m )
        
        
    def get_curr_pose(self):
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        return curr_pos_list[:6]
    
    def move_to_custom(self, pose_array):
        
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        print pose_array[0]
            
        # print "move_by_position"
        x = pose_array[0]
        y = pose_array[1]
        # z = curr_pos_list[2]
        z = pose_array[2]
        
        
        rz = pose_array[3]
        ry = pose_array[4]
        rx = pose_array[5]
        pose = curr_pos_list[7]
        position = Position( x, y, z, rz, ry, rx, posture = pose )

        self.rb.move(position)
    
    def move_only_translations(self, pose_array):
        
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        print pose_array[0]
            
        # print "move_by_position"
        x = pose_array[0]
        y = pose_array[1]
        z = curr_pos_list[2]
        
        
        
        rz = curr_pos_list[3]
        ry = curr_pos_list[4]
        rx = curr_pos_list[5]
        pose = curr_pos_list[7]
        position = Position( x, y, z, rz, ry, rx, posture = pose )

        self.rb.move(position)
    
    def move_to_object(self, pose_array):
        
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        
            
        print "move_by_position"
        x = pose_array[0]
        y = pose_array[1]
        # z = curr_pos_list[2]
        z = pose_array[2] +60.0
        
        
        rz = pose_array[3]
        
        ry = pose_array[4]
        rx = curr_pos_list[5]
        rx = pose_array[5]
        # ry = curr_pos_list[4]
        # pose = curr_pos_list[7]
        position = Position( x, y, z, rz, ry, rx, posture = pose)

        self.rb.optline(position)
       
    
    def move_place_pos(self):
        # self.rb.settool(id = 4, offz=offset)
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        # curr_pos = self.rb.getpos()
        # curr_pos_list = curr_pos.pos2list()
        place_position = Position( x= -100.45,y = 444.26,z = 332.80,rz = -90.0,ry = -0.0,rx = -180.0, posture = 6)
        self.rb.move(place_position)
        self.rb.sleep(0.5)
        
    def move_pose_estimate_pos(self):
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        # curr_pos = self.rb.getpos()
        # curr_pos_list = curr_pos.pos2list()
        place_position = Position( x = 190.69 + 2.5,y = 474.69,z = 330.74,rz = -90.0,ry = -0.0,rx = -180.0, posture = 6)
        print "packaging!!"
        self.rb.move(place_position)
        self.rb.sleep(0.5)
        
    def move_depth_pos(self):
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        # curr_pos = self.rb.getpos()
        # curr_pos_list = curr_pos.pos2list()
        place_position = Position( x = -70.87,y = 333.76,z = 354.93,rz = -90.0,ry = -0.0,rx = -180.0, posture = 6)
        print "packaging!!"
        self.rb.move(place_position)
        self.rb.sleep(0.5)
    
    def move_pick_pos(self):
        # self.rb.settool(id = 4, offz=offset)
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        # curr_pos = self.rb.getpos()
        # curr_pos_list = curr_pos.pos2list()
        # pick_position = Position( x = 446.80,y = 269.60,z = 371.60,rz = -90.0, ry = -0.00,rx = -180.0, posture = curr_pos_list[7])
        pick_position = Position( x = 497.37,y = 242.28,z = 313.53,rz = -90.0,ry = -0.0,rx = -180.0, posture = 6)
        self.rb.move(pick_position)
        self.rb.sleep(0.5)
        
    
    def pick(self, z_pos):
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        dout(48, '000')
        print "pick"
        # pick_position = Position( curr_pos_list[0], curr_pos_list[1], z_pos -50, curr_pos_list[3], curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        pick_position = Position( curr_pos_list[0], curr_pos_list[1], curr_pos_list[2] + z_pos, curr_pos_list[3], curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        
        print "pick pos"
        print curr_pos_list[2] + z_pos
        
        self.rb.move(pick_position)
        self.rb.sleep(0.1)
        dout(48, '001')
        # self.rb.sleep(0.5)
        self.rb.sleep(2)
        
        # pick_position2 = Position( curr_pos_list[0], curr_pos_list[1], 371.60, curr_pos_list[3], curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        pick_position2 = Position( curr_pos_list[0], curr_pos_list[1], 150.60, -90.0, curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        self.rb.move(pick_position2)
        pick_position2 = Position( x = 497.37,y = 242.28,z = 313.53,rz = -90.0, ry = -0.00,rx = -180.0, posture = curr_pos_list[7])
        self.rb.move(pick_position2)
        
        self.rb.sleep(0.5)
        
    def place(self, z_pos):
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        curr_pos = self.rb.getpos()
        curr_pos_list = curr_pos.pos2list()
        dout(48, '000')
        print "place"
        pick_position = Position( curr_pos_list[0], curr_pos_list[1], z_pos, curr_pos_list[3], curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        
        self.rb.move(pick_position)
        self.rb.sleep(0.1)
        dout(48, '100')
        # self.rb.sleep(0.5)
        self.rb.sleep(2)
        
        pick_position2 = Position( curr_pos_list[0], curr_pos_list[1], 371.60, curr_pos_list[3], curr_pos_list[4], curr_pos_list[5], posture = curr_pos_list[7])
        self.rb.move(pick_position2)
        
        self.rb.sleep(0.5)

    def gripper_open(self):
        dout(48, '000')
        self.rb.sleep(0.1)
        dout(48, '100')
        self.rb.sleep(0.1)

    def gripper_close(self):
        dout(48, '000')
        self.rb.sleep(0.1)
        dout(48, '001')
        self.rb.sleep(0.1)

    def orientation_offet(self, orientation):
        rz = orientation[0]
        ry = orientation[1]
        rx = orientation[2]
        self.rb.changetool(4) 
        self.rb.sleep(0.1)
        self.rb.relline(drz = rz, dry = ry, drx = rx)
        self.rb.sleep(0.5)

    
    def close_zeus(self):
        self.rb.close()
        
if __name__ == "__main__":
    server = tcp_server()
    robot = move_zeus()
    get_flag = 0
    
    try:
        while True:
            
            # connection client
            # robot.move_pick_pos()
            # curr_pos = robot.get_curr_pose()
                
            # print curr_pos 
            client_socket = server.connect_client()
            
            if client_socket:
                while True:
                
                    print "move zeus:"
                
                
                    data, get_flag = server.get_data()
                    
                    if data == -1 :
                        print "finish" 
                        server.close_client_connection()
                        break
                    
                    print "data 00"
                    print data[0][0]

                    if get_flag:
                        if data[0][0] == 0:
                            pose = data[1]
                            print "move custom position"
                            robot.move_to_custom(pose)
                            server.send_data(1)
                        
                        elif data[0][0] == 1:
                            print "send current position"
                            curr_pos = robot.get_curr_pose()
                            server.send_data(curr_pos)
                        
                        elif data[0][0] == 2:
                            print "move pick position"
                            robot.move_pick_pos()
                            server.send_data(1)
                            
                        elif data[0][0] == 3:
                            print "move place position"
                            robot.move_place_pos()
                            server.send_data(1)
                            
                        elif data[0][0] == 4:
                            print "move pose_estimation position"
                            robot.move_pose_estimate_pos()
                            server.send_data(1)
                        
                        elif data[0][0] == 5:
                            print "move depth position"
                            robot.move_depth_pos()
                            server.send_data(1)
                        
                        elif data[0][0] == 6:
                            print "move object position"
                            pose = data[1]
                            robot.move_to_object(pose)
                            server.send_data(1)
                            
                        elif data[0][0] == 7:
                            print "pick"
                            robot.pick(data[1][0])
                            server.send_data(1)
                            
                        elif data[0][0] == 8:
                            print "place"
                            robot.place(data[1][0])
                            server.send_data(1)
                            
                        elif data[0][0] == 9:
                            print "move only translations"
                            pose = data[1]
                            robot.move_only_translations(pose)
                            server.send_data(1)
                        
                        elif data[0][0] == 10:
                            print "gripper open"
                            
                            robot.gripper_open()
                            server.send_data(1)
                        
                        elif data[0][0] == 11:
                            print "gripper close"
                            
                            robot.gripper_close()
                            server.send_data(1)

                        elif data[0][0] == 12:
                            print "orientation offset"
                            
                            robot.orientation_offet(data[1])
                            server.send_data(1)
                        else :
                            print 'error no detect index'
                            server.send_data(-1)
                        # server.close_client_connection()
                    get_flag = 0
    except KeyboardInterrupt:
        print "close_server"
        # server.close_client_connection()
        # close server
        server.close_server()
        
        # close zeus robot
        robot.close_zeus()
    
    # except Exception:
    #     print "exception"
        
        
    #     server.close_client_connection()
        
    #     # close server
    #     server.close_server()
        
    #     # close zeus robot
    #     robot.close_zeus()
        