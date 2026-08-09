# SLAM Handbook 中文术语表

本表适用于第 3--18 章的翻译和审查。优先采用机器人学、SLAM、估计理论及计算机视觉文献中的通行译法；未列出的专有方法名、库名、数据集名、算法缩写和无法可靠定译的术语保留英文。

| English | 中文规范译法 | 说明 |
| --- | --- | --- |
| simultaneous localization and mapping (SLAM) | 同步定位与建图（SLAM） | 后文可直接用 SLAM。 |
| front-end / back-end | 前端 / 后端 | 不译为“前端部”“后端部”。 |
| factor graph | 因子图 | |
| maximum a posteriori (MAP) | 最大后验（MAP） | |
| least squares / nonlinear least squares | 最小二乘 / 非线性最小二乘 | |
| state variable | 状态变量 | |
| manifold | 流形 | |
| Lie group / Lie algebra | 李群 / 李代数 | |
| Jacobian matrix | 雅可比矩阵 | |
| covariance / information matrix | 协方差矩阵 / 信息矩阵 | |
| residual | 残差 | |
| measurement model | 测量模型 | |
| data association | 数据关联 | |
| outlier / inlier | 离群值 / 内点 | |
| robust estimation / robust loss | 鲁棒估计 / 鲁棒损失函数 | |
| outlier process | 离群值过程 | Black--Rangarajan 对偶性中的辅助惩罚函数。 |
| perceptual aliasing | 感知混叠 | 首次出现保留英文。 |
| loop closure | 回环 / 回环约束 | 按上下文使用。 |
| pose-graph optimization (PGO) | 位姿图优化（PGO） | |
| bundle adjustment (BA) | 束调整（BA） | |
| visual odometry (VO) | 视觉里程计（VO） | |
| point cloud | 点云 | |
| dense map representation | 稠密地图表示 | |
| range sensor / range sensing | 范围传感器 / 范围感知 | |
| occupancy map | 占据地图 | |
| occupancy grid | 占据栅格 | |
| implicit surface | 隐式表面 | |
| signed distance field (SDF) | 有符号距离场（SDF） | |
| Euclidean Signed Distance Field (ESDF) | 欧氏有符号距离场（ESDF） | |
| Truncated Signed Distance Function (TSDF) | 截断有符号距离函数（TSDF） | |
| surfel | surfel | 作为技术名保留英文。 |
| voxel | 体素 | |
| mesh | 网格 | |
| Gaussian Process Occupancy Map (GPOM) | 高斯过程占据地图（GPOM） | |
| Gaussian process implicit surface (GPIS) | 高斯过程隐式表面（GPIS） | |
| Hilbert map | Hilbert map | 作为方法名保留英文。 |
| simultaneous localization | 同步定位 | 不作“同时定位”。 |
| differentiable optimization | 可微优化 | |
| automatic differentiation | 自动微分 | |
| implicit differentiation | 隐式微分 | |
| unrolled differentiation | 展开微分 | |
| Gaussian process | 高斯过程 | |
| certifiably optimal | 可认证最优 | 指可验证最优性证书；首次出现可说明。 |
| semidefinite program (SDP) | 半正定规划（SDP） | |
| Fisher information matrix (FIM) | Fisher 信息矩阵（FIM） | |
| Cramer--Rao lower bound (CRLB) | Cramer--Rao 下界（CRLB） | |
| photogrammetry | 摄影测量学 | |
| structure from motion (SfM) | 运动恢复（SfM） | |
| visual SLAM (VSLAM) | 视觉 SLAM（VSLAM） | |
| LiDAR | LiDAR | 保留缩写。 |
| inertial measurement unit (IMU) | 惯性测量单元（IMU） | |
| neural radiance field (NeRF) | neural radiance field（NeRF） | 专有方法名保留英文。 |
| foundation model | 基础模型 | |
| spatial AI | 空间智能（Spatial AI） | |
| Shor's relaxation | Shor 松弛 | 半正定松弛的基础工具。 |
| Riemannian Staircase | 黎曼阶梯 | SE-Sync 等方法使用的流形优化求解器。 |
| pose-graph optimization (PGO) | 位姿图优化（PGO） | 与第 1 章术语一致。 |
| certifiably correct | 可认证正确 | 指可给出全局最优性或次优性证书。 |
| convex relaxation | 凸松弛 | |
| rank constraint | 秩约束 | |
| quadratically constrained quadratic program (QCQP) | 二次约束二次规划（QCQP） | |
| polynomial optimization problem (POP) | 多项式优化问题（POP） | |
| moment relaxation / Lasserre relaxation | 矩松弛 / Lasserre 松弛 | POP 的半正定松弛层级。 |
| range-aided SLAM | 距离辅助 SLAM | 使用范围测量的 SLAM。 |
| anisotropic noise | 各向异性噪声 | |
| Mahalanobis norm | Mahalanobis 范数 | |
| Fisher information matrix (FIM) | Fisher 信息矩阵（FIM） | |
| graph Laplacian | 图拉普拉斯矩阵 | |
| algebraic connectivity | 代数连通度 | 图拉普拉斯矩阵第二小特征值相关术语。 |
| D-optimality / A-optimality / E-optimality | D 最优性 / A 最优性 / E 最优性 | 最优实验设计准则。 |
