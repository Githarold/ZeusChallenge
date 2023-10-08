from socket import *  # tcp socket
import time


class JeusController():

    def __init__(self):
        self.client = socket(AF_INET, SOCK_STREAM)
        print('connecting...')
        self.client.connect(('192.168.0.23', 9000))
        self.client.setblocking(False)
        print('connected!')

    def get_current_pos(self):
        '''
        Get current pose of the robot.
        '''
        list_data = [[1], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        send_flag = str(list_data).encode()
        self.client.send(send_flag)
        close_flag = 0
        
        while True:
            try:
                recived_data = self.client.recv(1024)
                curr_pos = eval(recived_data)
                if recived_data:
                    print('received current position')
                    print(curr_pos)
                    break

            except KeyboardInterrupt:
                # ctrl-c
                close_flag = 1
                print("socket closed")
                self.client.close()
                break

            except BlockingIOError:
                # when data hasn't arrived yet
                pass

            if close_flag == 1:
                break

        return curr_pos

    def send_data(self, data, index):
        '''
        Send data and wait for completion signal.
        '''
        close_flag = 0
        send_flag = str(data).encode()
        self.client.send(send_flag)
        flag = "complete " + index

        while True:
            try:
                recived_data = self.client.recv(1024)
                error_flag = eval(recived_data)

                if error_flag == 1:
                    print(flag)
                    break
                elif error_flag == -1:
                    print('invalid index')
                    break

            except KeyboardInterrupt:
                # ctrl-c
                close_flag = 1
                print("socket closed")
                self.client.close()
                break

            except BlockingIOError:
                # when data hasn't arrived yet
                pass

            if close_flag == 1:
                break

    def move(self, pose_list):
        '''
        Custom move command.
        pose_list = [x,y,z,rx,ry,rz]
        '''
        x = pose_list[0]
        y = pose_list[1]
        self.z_pick = pose_list[2]
        rz = pose_list[3]
        ry = pose_list[4]
        rx = pose_list[5]

        list_data = [[0], [x, y, self.z_pick, rz, ry, rx]]

        self.send_data(list_data, "custom move")

    def move_pick(self):
        '''
        Move robot to pick object position.
        '''
        list_data = [[2], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to pick position")

    def move_place(self):
        '''
        Move robot to place object position.
        '''
        list_data = [[3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to place position")

    def move_pose_estimation(self):
        '''
        Move robot to position for pose estimation.
        '''
        list_data = [[4], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to pose estimation position")

    def move_depth(self):
        '''
        Move robot to position for depth map creation.
        '''
        list_data = [[5], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to depth position")

    def move_object(self, pose_list):
        '''
        Move robot to XY axis position of the object.
        pose = [x, y, z, rz, ry, rx]        
        '''
        x = pose_list[0]
        y = pose_list[1]
        z = pose_list[2]
        rz = pose_list[3]
        ry = pose_list[4]
        rx = pose_list[5]
        list_data = [[6], [x, y, z, rz, ry, rx]]

        self.send_data(list_data, "moving object")
                
    def move_only_position(self, x, y):
        '''
        Move Zeus only position
        '''
        list_data = [[9], [x, y]]
        
        self.send_data(list_data, "moving only translations")

    def pick(self):
        '''
        Move robot in Z-axis direction to pick up the object.
        '''
        offset = 50.0
        list_data = [[7], [self.z_pick - offset, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "pick")

    def place(self, z_pos):
        '''
        Move robot in Z-axis direction to place the object.
        z_pos : position in Z-axis direction of the object
        '''
        list_data = [[8], [z_pos , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
        self.send_data(list_data, "place")
    
    def close(self):
        
        print("close client")
        self.client.close()