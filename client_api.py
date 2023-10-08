from socket import *  # tcp socket

class JeusController():
    
    def __init__(self):
        '''
        Connect to JEUS.
        '''
        self.client = socket(AF_INET, SOCK_STREAM)
        print('connecting...')
        
        self.client.connect(('192.168.0.23', 9000))
        self.client.setblocking(False)
        print('complete_connecting!')       
        
    def move(self, pose_list):
        '''
        Move JEUS to a specified pose.
        pose_list = [x,y,z,rx,ry,rz]
        '''
        list_data = [[0], pose_list],
        
        send_flag = str(list_data).encode()
        self.client.send(send_flag)
        print('complete_send_move')        
        
    def get_current_pos(self):
        '''
        Get the current pose of the robot.
        '''
        list_data = [[1], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        
        send_flag = str(list_data).encode()
        self.client.send(send_flag)
        
        while True:
            try:
                received_data = self.client.recv(1024)
                curr_pos = eval(received_data)
                if received_data:
                    print('receive_curr_pos')
                    break                                   
                
            except KeyboardInterrupt:
                # If ctrl-c is pressed.
                close_flag = 1
                print("close socket")
                self.client.close()
                break
                    
            except BlockingIOError:
                # If data has not arrived yet.
                pass
            
            if close_flag == 1:
                break
        
        return curr_pos
                
    def send_data(self, data, index):
        '''
        Send data and receive a completion signal.
        '''
        send_flag = str(data).encode()
        self.client.send(send_flag)
        
        flag = "complete " + index
        
        while True:
            try:
                received_data = self.client.recv(1024)
                if received_data:
                    print(flag)
                    break                    
                
            except KeyboardInterrupt:
                # If ctrl-c is pressed.
                close_flag = 1
                print("close socket")
                self.client.close()
                break
                    
            except BlockingIOError:
                # If data has not arrived yet.
                pass
            
            if close_flag == 1:
                break
        
    def move_pick(self):
        '''
        Move to the position to pick the object.
        '''
        list_data = [[2], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.send_data(list_data, "move pick position")
            
    def move_place(self):
        '''
        Move to the position to place the object.
        '''
        list_data = [[3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.send_data(list_data, "move place position")
    
    def move_pose_estimation(self):
        '''
        Move to the position for object pose estimation.
        '''
        list_data = [[4], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.send_data(list_data, "move pose_estimation position")
        
    def pick(self):
        '''
        Move along the Z-axis to pick up the object.
        '''
        list_data = [[5], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.send_data(list_data, "pick")
        
    def place(self):
        '''
        Move along the Z-axis to release the object.
        '''
        list_data = [[6], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.send_data(list_data, "place")
    
    def close(self):
        '''
        Close the connection.
        '''
        self.client.close()