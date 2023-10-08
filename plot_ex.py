import numpy as np
import matplotlib.pyplot as plt

class Palletizing:
    
    def __init__(self):
        self.before_depth_map = None
        self.after_depth_map = None
        
    def visualization(self):
        fig, axs = plt.subplots(2, 1, figsize=(6,10))

        if self.before_depth_map is not None:
            axs[0].imshow(self.before_depth_map, cmap='jet')
            axs[0].set_title('Before Depth Map')
            
        if self.after_depth_map is not None:
            axs[1].imshow(self.after_depth_map, cmap='jet')
            axs[1].set_title('After Depth Map')

        plt.tight_layout()
        plt.show()

# 예제 실행
if __name__ == "__main__":
    palletizing = Palletizing()
    
    # 동일한 크기의 랜덤 행렬 생성
    before_depth_map = np.random.rand(100, 100)
    after_depth_map = np.random.rand(100, 100)
    
    palletizing.before_depth_map = before_depth_map
    palletizing.after_depth_map = after_depth_map

    palletizing.visualization()
