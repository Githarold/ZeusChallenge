from socket import * # tcp socket
import time

class client_api():
    
    def __init__(self):
        self.client = socket(AF_INET, SOCK_STREAM)
        print('connecting...')
        
        self.client.connect(('192.168.0.23', 9000))
        self.client.setblocking(False)
        print('complete_connecting!')       

    
        
    
        
        
    def get_current_pos(self):
        '''
        현재 로봇의 pose 값을 가지고 옴
        '''
        list_data = [[1],[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        send_flag = str(list_data).encode()
        self.client.send(send_flag)
        close_flag = 0
        while True:
            try:
                recived_data = self.client.recv(1024)
                curr_pos = eval(recived_data)
                if recived_data:
                    print('recive_curr_pos')
                    print(curr_pos)
                    break
                    
                
                
            except KeyboardInterrupt:
                
                # ctrl-c
                close_flag = 1
                print("close socket")
                self.client.close()
                break
                    
            except BlockingIOError:
                # 데이터가 아직 도착하지 않은 경우
                pass
            
            if close_flag == 1:
                
                break
        
        
        return curr_pos
        
        
    def send_data(self, data, index):
        '''
        데이터 전송하고 도착 완료 신호를 받아오기
        '''
        close_flag = 0
        send_flag = str(data).encode()
        self.client.send(send_flag)
        # self.client.flush()  # 추가
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
                print("close socket")
                self.client.close()
                break
                    
            except BlockingIOError:
                # 데이터가 아직 도착하지 않은 경우
                pass
            
            if close_flag == 1:
                
                break
    
    def move(self, pose_list):
        '''
        move jeus
        pose_list = [x,y,z,rx,ry,rz]
        '''
        x = pose_list[0]
        y = pose_list[1]
        z = pose_list[2]
        rz = pose_list[3]
        ry = pose_list[4]
        rx = pose_list[5]
        
        list_data = [[0],[x,y,z,rz,ry,rx]]
        
        
        self.send_data(list_data, "custom move")    
    
    
    def move_pick(self):
        
        '''
        물체를 pick 하는 위치로 이동
        '''
        list_data = [[2],[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        self.send_data(list_data, "move pick position")
        
    
    def move_place(self):
        
        '''
        물체를 place 하는 위치로 이동
        '''
        list_data = [[3],[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        
        
        self.send_data(list_data, "move place position")
    
    def move_pose_estimation(self):
        
        '''
        물체의 pose를 인식하는 곳으로 이동
        '''
        list_data = [[4],[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        
        
        self.send_data(list_data, "move pose_estimation position")
        
    def move_depth(self):
        
        '''
        depth맵을 만드는 곳으로 이동
        '''
        list_data = [[5],[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        
        
        self.send_data(list_data, "move pose_depth position")
        
    def move_object(self, pose_list):
        
        '''
        물체의 xy축 위치로 로봇 이동
        pose = [x, y, z, rz, ry, rx]
        '''
        x = pose_list[0]
        y = pose_list[1]
        z = pose_list[2]
        rz = pose_list[3]
        ry = pose_list[4]
        rx = pose_list[5]
        list_data = [[6],[x,y,z,rz,ry,rx]]
        
        self.send_data(list_data, "move move object")
        
    def pick(self, z_pos):
        
        '''
        물체의 잡기위해 z축으로 이동
        z_pos : 물체의 z축 방향의 위치
        '''
        list_data = [[7],[z_pos , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        
    
        
        self.send_data(list_data, "pick")
        
    def place(self, z_pos):
        
        '''
        물체의 잡기 놓기 위해 z축으로 이동
        z_pos : 물체의 z축 방향의 위치
        '''
        list_data = [[8],[z_pos , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]]
        
        
        
        
        self.send_data(list_data, "place")
    
    def close(self):
        print("close client")
        self.client.close()