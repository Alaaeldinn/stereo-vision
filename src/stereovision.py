import numpy as np
import cv2
import matplotlib.pyplot as plt

class Depthmap:
    def __init__(self):
        self.left_img = None
        self.right_img = None

    def load_images(self, left_path, right_path):
        self.left_img = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        self.right_img = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    def display_image_pair(self):
        if self.left_img is None or self.right_img is None:
            print("Images not loaded. Please load images first.")
            return

        fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(12, 8))

        axes[0].imshow(self.left_img, cmap='gray')
        axes[1].imshow(self.right_img, cmap='gray')

        axes[0].axis('off')
        axes[1].axis('off')

        plt.show()

    def compute_depth_map_bm(self):
        if self.left_img is None or self.right_img is None:
            print("Images not loaded. Please load images first.")
            return

        nfactor = 12
        stereo = cv2.StereoBM_create(numDisparities=16*nfactor, blockSize=21)
        disparity = stereo.compute(self.left_img, self.right_img)
        plt.imshow(disparity, 'gray')
        plt.show()

    def compute_depth_map_sgbm(self):
        if self.left_img is None or self.right_img is None:
            print("Images not loaded. Please load images first.")
            return

        window_size = 2
        min_disp = -100
        nfactor = 4
        num_disp = 16*nfactor-min_disp

        stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=8*3*window_size**2,
            P2=32*3*window_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        disparity = stereo.compute(self.left_img, self.right_img)
        plt.imshow(disparity, 'gray')
        plt.colorbar()
        plt.show()
