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
| visual odometry (VO) | 视觉里程计（VO） | 估计相机连续帧间相对运动。 |
| visual place recognition | 视觉地点识别 | 从已登记图像中检索同一地点。 |
| relocalization | 重定位 | 跟踪失败或回环后重新估计位姿。 |
| reprojection error | 重投影误差 | 观测像素与模型投影像素之差。 |
| photometric error / photometric consistency | 光度误差 / 光度一致性 | 基于像素强度或辐照度的残差约束。 |
| keypoint / feature point | 关键点 / 特征点 | 图像中可重复检测和匹配的显著点。 |
| keypoint detector / descriptor | 关键点检测器 / 描述子 | 分别用于检测关键点和编码局部外观。 |
| camera intrinsic parameters | 相机内参 | 投影模型中的焦距、主点和畸变参数等。 |
| pinhole camera model | 针孔相机模型 | 直线透视投影模型。 |
| rolling shutter / global shutter | 卷帘快门 / 全局快门 | 分行采集 / 同时采集整幅图像的快门。 |
| field of view (FOV) | 视场（FOV） | 镜头能够成像的角度范围。 |
| radial-tangential distortion | 径向--切向畸变 | 常用镜头畸变参数化模型。 |
| fisheye camera | 鱼眼相机 | 采用超广视场镜头的相机。 |
| Double Sphere (DS) model | Double Sphere（DS）模型 | 鱼眼和广角相机投影模型，方法名保留英文。 |
| covisibility graph | 共视图 | 以共同观测点连接关键帧的图。 |
| essential graph | 本质图 | 用于回环校正的稀疏关键帧图。 |
| pose-only bundle adjustment | 仅位姿束调整 | 固定地图点、仅优化相机位姿的 BA。 |
| Schur complement | Schur 补 | 通过消去变量构造降维线性系统。 |
| variable projection | 变量投影 | 将部分变量解析消去以改善优化初始化。 |
| photometric calibration | 光度标定 | 将辐照度映射为像素值的相机标定。 |
| RGB-D camera | RGB-D 相机 | 同时提供彩色和深度图像的相机。 |
| truncated signed distance function (TSDF) | 截断有符号距离函数（TSDF） | 对深度测量进行融合的距离场表示。 |
| voxel hashing / octree | 体素哈希 / 八叉树 | 稠密地图的自适应存储结构。 |
| visual-inertial SLAM | 视觉惯性 SLAM | 融合相机与 IMU 的 SLAM 系统。 |
| tightly coupled / loosely coupled fusion | 紧耦合 / 松耦合融合 | 联合优化原始测量 / 先独立估计再融合。 |
| delayed marginalization | 延迟边缘化 | 延后边缘化以改善运动可观测性建模。 |
| LiDAR odometry | LiDAR 里程计 | 由连续 LiDAR 扫描估计传感器增量运动。 |
| scan registration / scan matching | 扫描配准 / 扫描匹配 | 对齐两次扫描并估计相对变换。 |
| time-of-flight (TOF) | 飞行时间（TOF） | 根据光脉冲往返时间测量距离。 |
| amplitude modulated continuous wave (AMCW) | 调幅连续波（AMCW） | LiDAR 的连续波测距技术。 |
| frequency modulated continuous wave (FMCW) | 调频连续波（FMCW） | 通过频移测量距离和相对速度的技术。 |
| signal-to-noise ratio (SNR) | 信噪比（SNR） | 信号强度与噪声强度之比。 |
| mechanical LiDAR / solid-state LiDAR / flash LiDAR | 机械式 LiDAR / 固态 LiDAR / 闪光 LiDAR | 按扫描和发射机构分类的 LiDAR 类型。 |
| Risley prism | Risley 棱镜 | 用于宏观扫描光束偏转的棱镜组件。 |
| scan-to-scan / scan-to-map | 扫描到扫描 / 扫描到地图 | 分别相对上一扫描 / 局部地图进行配准。 |
| motion distortion / motion undistortion | 运动畸变 / 运动去畸变 | 扫描期间传感器运动造成的点云畸变及其校正。 |
| Iterative Closest Point (ICP) | 迭代最近点（ICP） | 通过交替更新对应关系和变换进行点云配准。 |
| normal distributions transform (NDT) | 正态分布变换（NDT） | 将体素点集拟合分布并进行分布匹配。 |
| point-to-point / point-to-line / point-to-plane | 点到点 / 点到线 / 点到平面 | 扫描配准中的几何残差类型。 |
| scan-to-map odometry | 扫描到地图里程计 | 将输入扫描配准到持续更新局部地图的里程计。 |
| structural perceptual aliasing | 结构感知混叠 | 结构相似导致不同地点在识别中难以区分。 |
| place recognition invariance / awareness | 地点识别不变性 / 感知能力 | 分别指对位姿变化保持识别 / 还可估计重访位姿变化。 |
| bird's-eye view (BEV) | 鸟瞰图（BEV） | 点云的俯视粗粒度投影表示。 |
| range image | 范围图像 | 按扫描方向组织的距离测量图像。 |
| global registration | 全局配准 | 不依赖精确初始猜测的点云相对变换估计。 |
| anchor node | 锚节点 | 连接不同会话地图坐标系的辅助位姿变量。 |
| multi-session SLAM / multi-robot SLAM | 多会话 SLAM / 多机器人 SLAM | 跨建图会话 / 跨机器人融合地图的 SLAM。 |
| centralized / decentralized SLAM | 集中式 / 分散式 SLAM | 在基站联合计算 / 各机器人独立建图再合并。 |
| Size, Weight, and Power (SWaP) | 尺寸、重量和功耗（SWaP） | 机载系统部署的传统资源约束。 |
| event camera / event-based vision | 事件相机 / 基于事件的视觉 | 按像素异步输出亮度变化事件，而非固定帧率图像。 |
| event / polarity / event packet | 事件 / 极性 / 事件数据包 | 极性表示亮度增加或减小；数据包为一组事件。 |
| time surface / voxel grid | 时间表面 / 体素网格 | 事件数据的常用时空表示。 |
| image of warped events (IWE) | 事件扭曲图像（IWE） | 依据候选运动将事件投影并进行运动补偿后的图像。 |
| contrast maximization (CMax) | 对比度最大化（CMax） | 通过最大化事件对齐图像的对比度估计运动。 |
| inertial odometry / inertial navigation system (INS) | 惯性里程计 / 惯性导航系统（INS） | 由 IMU 测量推断平台运动和状态。 |
| IMU preintegration | IMU 预积分 | 将高频 IMU 测量汇总为关键帧间相对运动因子。 |
| aided inertial navigation system (AINS) | 辅助惯性导航系统（AINS） | 将 IMU 与 GPS、相机或 LiDAR 等外感传感器融合。 |
| observability / unobservable direction | 可观测性 / 不可观测方向 | 由测量能否唯一确定状态方向的性质。 |
| bias random walk / bias factor | 偏置随机游走 / 偏置因子 | IMU 偏置的随机游走模型及其因子图约束。 |
| leg odometry | 腿部里程计 | 根据足式机器人关节与接触感知估计机体运动。 |
| stance phase / swing phase | 支撑阶段 / 摆动阶段 | 足端与地面无滑动接触 / 暂时离开地面的步态阶段。 |
| forward kinematics / differential kinematics | 正向运动学 / 微分运动学 | 由关节状态求足端位姿 / 速度的映射。 |
| ground reaction force (GRF) | 地面反作用力（GRF） | 地面对接触足端施加的力。 |
| force/torque sensor (F/T sensor) | 力/力矩传感器（F/T 传感器） | 测量接触力和力矩的传感器。 |
| friction cone / center of pressure (CoP) | 摩擦锥 / 压力中心（CoP） | 判断接触无滑移和足端支撑范围的几何约束。 |
| contact preintegration / velocity preintegration | 接触预积分 / 速度预积分 | 在因子图中汇总支撑接触或足端速度测量。 |
| contact frame / touchdown | 接触坐标系 / 触地 | 足端建立地面接触时定义的临时惯性坐标系 / 事件。 |
| leg deformation / slippage | 腿部变形 / 滑移 | 刚体腿弯曲或接触点相对地面发生运动。 |
