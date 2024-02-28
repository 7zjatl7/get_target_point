from sklearn.cluster import KMeans
import transforms3d
import numpy as np



def transform_angle(init_pose, cur_pose, obj_pose):

    init_pose = np.array(init_pose)
    cur_pose = np.array(cur_pose)
    obj_pose = np.array(obj_pose)

    # 현재 방향 벡터와 목표 방향 벡터 계산
    current_direction_vector = cur_pose - init_pose
    target_direction_vector = obj_pose - cur_pose

    # 두 벡터 사이의 각도 계산
    cos_theta = np.dot(current_direction_vector, target_direction_vector) / (np.linalg.norm(current_direction_vector) * np.linalg.norm(target_direction_vector))
    theta_radians = np.arccos(cos_theta)
    theta_degrees = np.degrees(theta_radians)

    print("current -> target degree :", theta_degrees)

    # 지상 로봇이므로 z축기준으로 회전 
    roll = np.radians(0) 
    pitch = np.radians(0) 
    yaw = np.radians(theta_degrees) 

    quaternion = transforms3d.euler.euler2quat(roll, pitch, yaw)
    print("trasnform from euler to quaternion :{0} -> {1}".format(yaw, quaternion))
    
    return [f"{x:.3f}" for x in quaternion]



# Z-점수 계산 함수 (2차원 데이터용)
def calculate_z_scores_2d(data):
    means = np.mean(data, axis=0)
    stds = np.std(data, axis=0)
    z_scores = (data - means) / stds
    return z_scores

# 이상치 제거 함수 (2차원 데이터용)
def remove_outliers_2d(data, threshold=2):
    z_scores = calculate_z_scores_2d(data)
    return np.array([data[i] for i in range(len(data)) if np.all(np.abs(z_scores[i]) <= threshold)])

def cluster_kmeans(data):
    # 이상치 제거 실행 (2차원 데이터용)
    print('Data Length',len(data))
    filtered_data = remove_outliers_2d(data)
    print('filtered data Length',len(filtered_data))
    # K-평균 클러스터링 실행
    kmeans = KMeans(n_clusters=1, n_init=10)  # 클러스터의 수를 3으로 설정
    kmeans.fit(filtered_data)
    labels = kmeans.labels_

    # 클러스터 중심점 표시
    centers = kmeans.cluster_centers_

    return centers

def write_goal_txt(file_path, data):

    fp = file_path
    with open(fp, 'a', encoding='utf-8') as file:
        file.write("{0}\n".format(data))