# TrackedObjectArray.msg
# Version history:
#
# v0.0.3 20260618
# - 目标pose是底面中心。
# - 感知仅输出track_state=TRACK_CONFIRMED，没检出使用普通障碍物解决。
# - 速度标志位区分线速度与角速度及其协方差。
# - 静止目标运动状态感知输出完全是 0 m/s。
#
# v0.0.2 20260618
# - 重新划分类别编号区间并扩充各类别族的预留空间。
# - 将 classifications 数组修改为单个 Top-1 classification。
# - 删除无法可靠提供的 existence_probability 字段。
# - 完善坐标系、时间戳、跟踪状态和运动状态的定义。
# - 完善目标局部坐标系、朝向有效性及几何形状约定。
# - 明确 dimensions 和 footprint 使用目标局部坐标系表达。
# - 本版本与 v0.0.1 不兼容，相关 ROS 节点需要重新编译。
#
# v0.0.1 20260616
# - 首次发布。
# - 所有追踪目标统一使用数组级 Header。
# - 支持目标唯一标识、语义分类、运动学状态、跟踪状态、运动状态和几何形状。
#

std_msgs/Header header         # 包含 timestamp 和 frame_id (通常是 odom 或 map)
TrackedObject[] objects        # 追踪目标数组
