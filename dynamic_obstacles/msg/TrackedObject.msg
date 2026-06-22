# TrackedObject.msg

# ============================================================
# 坐标系与时间规范
#
# 1. 本消息自身不包含 Header，所有目标统一使用
#    TrackedObjectArray.header 中的时间戳和参考坐标系。
#
# 2. 本项目规定：
#    TrackedObjectArray.header.frame_id 必须为 "map"或"odom"。
#
# 3. pose、twist 和 acceleration 均对应
#    TrackedObjectArray.header.stamp 所表示的统一状态时刻。
#
# 4. 以下物理量均在 指定参考的 坐标系上表达：
#    - pose.position 底面中心点
#    - pose.orientation 表示目标局部坐标系到 指定参考的 坐标系的旋转
#    - twist.linear
#    - twist.angular
#    - acceleration.linear
#    - acceleration.angular
#
# 5. twist 和 acceleration 表示经过机器人自身运动补偿后的、
#    目标相对于 指定参考的 坐标系的估计运动状态，
#    不是相对机器人或相对传感器的运动状态。
#
# 6. 目标局部坐标系基础约定：
#   - 原点位于目标底面几何中心，右手坐标系
#   - X 轴沿目标长度方向，但其具体指向受以下 orientation_availability 约束
#   - Y 轴沿目标宽度方向
#   - Z 轴沿目标高度方向，指向上方
#
# 7. dimensions 和 footprint 在目标局部坐标系中定义。
#    footprint 不是 指定参考的坐标系下 的坐标；应结合 pose 转换到 指定参考的坐标系下。
#
# 8. 单位遵循 ROS 约定：
#    - 位置、尺寸：m
#    - 线速度：m/s
#    - 角速度：rad/s
#    - 线加速度：m/s^2
#    - 角加速度：rad/s^2
# ============================================================


# 1. 基础属性
uuid_msgs/UniqueID object_id           # 全局唯一的追踪 ID，轨迹删除后，不应立即复用相同 ID

ObjectClassification classification    # 当前目标置信度最高的最终语义类别

# ============================================================
# 2. 跟踪状态
# ============================================================

# TRACK_UNKNOWN：未知
# TENTATIVE：单帧新增障碍物/持续多帧尚未满足稳定确认条件，防止目标漏避
# CONFIRMED：稳定轨迹，当前有观测更新
# COASTED：当前无观测，由运动模型预测外推

uint8 TRACK_UNKNOWN=0
uint8 TRACK_TENTATIVE=1
# 当前版本的感知发布端仅输出 TRACK_CONFIRMED。
uint8 TRACK_CONFIRMED=2
uint8 TRACK_COASTED=3

uint8 track_state

# 自轨迹创建以来的持续时间，单位 s
float32 tracking_time

# 距最后一次真实观测更新的时间，单位 s
float32 time_since_update

# ==========================================
# 3. 运动学状态 (Kinematics)
# 包含了位姿、速度、加速度，以及它们对应的 6x6 协方差矩阵(表示不确定性)
# ==========================================

# 位姿 (x, y, z 及 四元数姿态) + 协方差
geometry_msgs/PoseWithCovariance pose
bool has_pose_covariance

# 速度 (线速度 v_x, v_y, v_z 及 角速度) + 协方差
geometry_msgs/TwistWithCovariance twist
bool has_linear_velocity 
bool has_angular_velocity 
bool has_linear_covariance
bool has_angular_covariance

# 加速度 (线加速度 a_x, a_y, a_z 及 角加速度) + 协方差
geometry_msgs/AccelWithCovariance acceleration
bool has_acceleration
bool has_acceleration_covariance

# 运动状态

uint8 MOTION_UNKNOWN=0
uint8 MOTION_MOVING=1
uint8 MOTION_STATIONARY=2

uint8 motion_state

# ============================================================
# 4. 朝向有效性
# ============================================================

# 无法获得可靠的几何方向或语义朝向。
# pose.orientation 必须是合法且归一化的四元数；
# 没有任何方向估计时推荐填单位四元数。
#
# 当 shape_type=SHAPE_BOUNDING_BOX 时，应输出相对于
# header.frame_id 的轴对齐保守包围盒，或改用不依赖朝向的
# SHAPE_CYLINDER。
uint8 ORIENTATION_UNAVAILABLE=0

# 只能确定目标长轴，正反方向存在 pi 弧度二义性。
# 局部 X 轴平行于目标长轴，但正方向可能与真实前方相反。
uint8 ORIENTATION_SIGN_UNKNOWN=1

# 能够完整区分目标正面和背面，局部 +X 指向前方，+Y 指向左侧，+Z 指向上方。
uint8 ORIENTATION_AVAILABLE=2

uint8 orientation_availability



# ==========================================
# 5. 几何形状 (Shape)
# 局部 x 轴为 length 方向，局部 y 轴为 width 方向，局部 z 轴为 height 方向
# polygon 点序为逆时针，首尾点闭合
# ==========================================
# 形状未知或当前无法提供可靠几何形状 
uint8 SHAPE_UNKNOWN=0
uint8 SHAPE_BOUNDING_BOX=1
uint8 SHAPE_CYLINDER=2
uint8 SHAPE_POLYGON=3

uint8 shape_type                       # 形状类型

#BOUNDING_BOX：x=length, y=width, z=height，且 footprint 应为空
#CYLINDER：x=diameter, y=diameter, z=height，且 footprint 应为空
#POLYGON：x=0, y=0, z=height，形状由 footprint 决定
geometry_msgs/Vector3 dimensions

# 仅在 shape_type=SHAPE_POLYGON 时使用；
# 点位于目标局部坐标系中
geometry_msgs/Polygon footprint
