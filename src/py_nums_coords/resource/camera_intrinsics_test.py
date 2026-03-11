import cv2
import numpy as np

# 1. 加载标定参数 (请替换为你自己的标定结果)
# 假设你保存为了 npz 文件，或者直接填入数值
# K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist_coeffs = np.array([0.03, -0.072958, -0.001949469, -0.000311134, 0]) # 示例畸变系数
camera_matrix = np.array([[1378.2163254, 0, 658.93450119], 
                          [0, 1388.42507377, 356.2443771669], 
                          [0, 0, 1]], dtype=np.float32)

# 2. 配置实验参数
DISTANCE_TO_OBJECT = 745.0  # 物体到相机的实际垂直距离 (单位: mm)
points = []

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(img_undistorted, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Calibration Verification", img_undistorted)
        
        if len(points) == 2:
            calculate_distance()

def calculate_distance():
    p1, p2 = points[0], points[1]
    
    # 计算像素距离 (欧几里得距离)
    pixel_dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    # 获取焦距 (通常取 fx 和 fy 的平均值，或者根据测量方向选择)
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    f_avg = (fx + fy) / 2
    
    # 核心公式: 物理尺寸 = (像素距离 * 距离) / 焦距
    real_size = (pixel_dist * DISTANCE_TO_OBJECT) / f_avg
    
    print(f"--- 验证结果 ---")
    print(f"像素距离: {pixel_dist:.2f} px")
    print(f"计算出的物理尺寸: {real_size:.2f} mm")
    print(f"请对照物体的实际尺寸进行验证。")

# 3. 读取并去畸变
image = cv2.imread('11.jpg') # 替换为你的测试图片
h, w = image.shape[:2]
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))
img_undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)

# 4. 进入交互界面
print("操作指南: 在图中点击物体的两个端点来测量长度。")
cv2.imshow("Calibration Verification", img_undistorted)
cv2.setMouseCallback("Calibration Verification", click_event)

cv2.waitKey(0)
cv2.destroyAllWindows()