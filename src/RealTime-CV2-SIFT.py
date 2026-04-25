
import cv2
import numpy as np

import matplotlib.pyplot as plt
import numpy as np

# --- Configuração da câmera ---
cap = cv2.VideoCapture(0)  # use 0 para webcam, ou substitua por um vídeo/câmera IP

sift = cv2.SIFT_create()

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

# --- Matriz da câmera (supondo parâmetros aproximados) ---
K = np.array([[736.42621 ,  0,         269.59851],
              [  0,         725.07717, 239.85903],
              [  0,         0,         1]],         dtype=np.float32)

prev_gray = None
prev_kp = None
prev_des = None

# Pose acumulada (R, t)
R_f = np.eye(3)
t_f = np.zeros((3, 1))

trajectory = []

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # --- Extração de features ---
    kp, des = sift.detectAndCompute(gray, None)
    
    if prev_gray is not None:
        # Matching
        matches = flann.knnMatch(prev_des, des, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        
        if len(good) > 8:
            pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
            pts2 = np.float32([kp[m.trainIdx].pt for m in good])
            
            # Estima matriz Essencial
            E, mask = cv2.findEssentialMat(
                pts2, pts1, K, cv2.RANSAC, 0.999, 1.0)
            
            if E is not None:
                _, R, t, mask_pose = cv2.recoverPose(E, pts2, pts1, K)
                
                # Acumular pose
                t_f = t_f + R_f @ t
                R_f = R @ R_f
                
                trajectory.append(t_f.copy())

                # Mostrar resultado
                cv2.putText(frame, f"t: {t_f.ravel()}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2)
        
        # Desenhar features
        frame = cv2.drawKeypoints(frame, kp, None, color=(0,255,0))

    cv2.imshow("Visual Odometry", frame)
    
    prev_gray = gray
    prev_kp = kp
    prev_des = des
    
    if cv2.waitKey(1) & 0xFF == 27:  # Pressione ESC para sair
        break

traj = np.array(trajectory).squeeze()  # vira N x 3

plt.plot(traj[:,0], traj[:,2])  # x vs z (coordenadas horizontais no KITTI)
plt.xlabel("X")
plt.ylabel("Z")
plt.title("Trajetória estimada")
plt.axis('equal')
plt.show()

cap.release()
cv2.destroyAllWindows()
