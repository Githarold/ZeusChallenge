import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

class Palletizing:
    
    def __init__(self):
        self.before_depth_map = None
        self.after_depth_map = None
        
    def visualization(self):
        fig = plt.figure(figsize=(10, 12))

        # Gridspec 사용하여 layout 설정
        gs = gridspec.GridSpec(2, 3, width_ratios=[1, 0.05, 1]) 

        # Before Depth Map Visualization
        ax1 = fig.add_subplot(gs[0, 0])
        if self.before_depth_map is not None:
            img1 = ax1.imshow(self.before_depth_map, cmap='jet')
            ax1.set_title('Before Depth Map')

        # Before Depth Map의 3D 플롯
        ax2 = fig.add_subplot(gs[0, 2], projection='3d')
        if self.before_depth_map is not None:
            x = np.linspace(0, self.before_depth_map.shape[1]-1, self.before_depth_map.shape[1])
            y = np.linspace(0, self.before_depth_map.shape[0]-1, self.before_depth_map.shape[0])
            X, Y = np.meshgrid(x, y)
            ax2.plot_surface(X, Y, self.before_depth_map, cmap='viridis')
            ax2.set_title('Before Depth 3D Plot')
        
        # After Depth Map Visualization
        ax3 = fig.add_subplot(gs[1, 0])
        if self.after_depth_map is not None:
            img2 = ax3.imshow(self.after_depth_map, cmap='jet')
            ax3.set_title('After Depth Map')

        # After Depth Map의 3D 플롯
        ax4 = fig.add_subplot(gs[1, 2], projection='3d')
        if self.after_depth_map is not None:
            x = np.linspace(0, self.after_depth_map.shape[1]-1, self.after_depth_map.shape[1])
            y = np.linspace(0, self.after_depth_map.shape[0]-1, self.after_depth_map.shape[0])
            X, Y = np.meshgrid(x, y)
            ax4.plot_surface(X, Y, self.after_depth_map, cmap='viridis')
            ax4.set_title('After Depth 3D Plot')

        # 중앙에 colorbar 추가
        cax = fig.add_subplot(gs[:, 1])
        fig.colorbar(img1, cax=cax, orientation='vertical')

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
