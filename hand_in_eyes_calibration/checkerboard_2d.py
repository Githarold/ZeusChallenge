import numpy as np
import cv2
import glob




cv2.namedWindow('img', cv2.WINDOW_NORMAL)


checkerboard = (5, 3)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 43, 0.001) #30, 40 ,45

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((checkerboard[0]*checkerboard[1],3), np.float32) #5*8 -> 최외곽 제외 꼭지점 수 (아래도 수정 모두 )
objp[:,:2] = np.mgrid[0:checkerboard[0],0:checkerboard[1]].T.reshape(-1,2)
# index = "_pos5_4"
index = '_1'
rgb_path = "./data/cali_rgb"+ index +".png"
depth_path =  "./data/xyz_image"+ index +".npy"
print (rgb_path)
xyz_points = np.empty((0, 3))
for img_name in range(1):
    image = glob.glob(rgb_path)
    xyz_image = np.load(depth_path)

    print(xyz_image.shape)
    for fname in image:
        
        img = cv2.imread(fname)
     
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, checkerboard, None)
        


        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Zivid는 검출된 corner픽셀의 index만 추출하면 x,y,z값을 구할 수 있음..(Raw)
            items = np.empty((0,3))
            for ii in corners2:
                pixel_x = round(ii[0][0])
                pixel_y = round(ii[0][1])

                # print(pixel_x)
                # print(pixel_y)
                print(xyz_image.shape)
                xyz_point = xyz_image[pixel_y, pixel_x] # or xyz_image[pixel_y, pixel_x]
                
                xyz_points = np.vstack((xyz_points, xyz_point))
                

            num=0
            # print(xyz_points[num])
            cv2.circle(img, (int(corners2[num][0][0]),int(corners2[num][0][1])), 10, (0, 0, 0))
            cv2.putText(img, str(xyz_points[num]), (int(corners2[num][0][0]),int(corners2[num][0][1])), 1, 1, (0,0,0), 1, cv2.LINE_AA)
                # items = np.vstack((items, item[0]))
            print(xyz_points)
            cv2.drawChessboardCorners(img, checkerboard, corners, ret)

            formatted_data = []
            # for row in xyz_points:
            #     formatted_row = "[ {:.7f} , {:.7f} , {:.7f} ],\n".format(row[0], row[1], row[2])
            #     formatted_data.append(formatted_row)
            # formatted_data.append("#####################################\n")
            # # 기존 파일에 추가로 저장합니다.
            # with open("./data/1. camera_pose.txt", "a") as file:
            #     file.writelines(formatted_data)
            # np.save("./data/cam2target.npy", xyz_points)

            
            # formatted_row = "[ {:.7f} , {:.7f} , {:.7f} ],\n".format(xyz_points[7][0], xyz_points[7][1], xyz_points[7][2])
            # formatted_data.append(formatted_row)
            # formatted_data.append("#####################################\n")
            # # 기존 파일에 추가로 저장합니다.
            # with open("./data/1. camera_pose.txt", "a") as file:
            #     file.writelines(formatted_data)
            # np.save("./data/cam2target.npy", xyz_points)

            cv2.imshow('img',img)
            

            # board의 점 순서와 로봇으로 직접 찍는 위치의 순서는 동일해야함. -> 시각화가 중요
            
            

        key = cv2.waitKey(0)
        
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
    # cv2.destroyAllWindows()