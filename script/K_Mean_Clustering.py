import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# 주어진 데이터
data = [
    [-1018.993197631836, -837.5276565551758], [1764.6167006492615, -669.2886500358582],
    [-1025.532130432129, -850.7581314086915], [1767.2729349136353, -664.0365365982057],
    [-1023.9440780639649, -848.0464843750001], [1766.6335898399352, -666.0622782707214],
    [1764.9190562248232, -662.5833056449891], [1762.9594633102417, -663.4886018753052],
    [-1014.8852844238281, -848.4338539123536], [1768.3092930793762, -671.0941494941711],
    [-1014.4371002197266, -848.5017807006836], [1767.2270526885986, -671.5812168121338],
    [-1013.5607421875, -847.3581153869629], [1767.5519500732423, -670.4835090637207]
]

# NumPy 배열로 변환
X = np.array(data)

# X 좌표를 조정하여 데이터 간격을 줄임 (X 좌표의 범위를 축소)
X[:, 0] = X[:, 0] / 100  # X 좌표를 100으로 나눠서 간격을 줄임

# 데이터에 약간의 노이즈 추가
np.random.seed(42)  # 재현 가능성을 위해 랜덤 시드 설정
noise = np.random.normal(0, 10, X.shape)  # 평균 0, 표준편차 10의 노이즈 추가
X_noisy = X + noise

# KMeans 클러스터링 수행
num_clusters = 3  # 클러스터 개수 3개로 설정
kmeans = KMeans(n_clusters=num_clusters)
kmeans.fit(X_noisy)

# 클러스터 레이블과 중심점(centroids) 가져오기
labels = kmeans.labels_
centroids = kmeans.cluster_centers_

# 클러스터링 결과 시각화
plt.figure(figsize=(8, 6))
colors = ['r', 'g', 'b']  # 클러스터 색상

for i in range(num_clusters):
    cluster_points = X_noisy[labels == i]
    plt.scatter(cluster_points[:, 0], cluster_points[:, 1], c=colors[i], s=50, label=f'Cluster {i+1}')

# 중심점을 별 모양으로 표시
plt.scatter(centroids[:, 0], centroids[:, 1], c='black', marker='*', s=200, label='Centroids')

plt.title('K-Means Clustering with Adjusted X Coordinates')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.grid(True)
plt.show()


