# 机械臂校准流程

适用于四杆(平行四边形)传动机械臂:大臂/小臂 120 mm,elbow 舵机动作杆 40 mm,
曲柄相对小臂固定偏置 20°,连杆宽 10 mm。elbow 舵机接地,经平行四边形控制的是
小臂的对地绝对俯仰角,而不是肘部相对角。

## 固件保护行为(校准时依赖)

- 上电不 attach 任何舵机;每个关节在收到它的第一条校准命令时才单独 attach。
  第一条命令直接就位(无缓动),之后的命令以 60°/s 缓动逼近。
- shoulder(joint 1)与 elbow(joint 2)之间有耦合护栏:折叠角
  fold = (90° + shoulder偏移) − forearm绝对角,校准态允许 [-5°, 138°]。
  越界的命令收到 Nack(ValidationFailed);缓动中越界则原地停住,并在遥测
  status 的 warning 位报告 `WarningArmTargetLimited`(bit 4)。
- 运行态(非校准 profile)折叠角限制收紧为 [5°, 135°],对应腕点到肩轴距离
  d = 240·cos(fold/2) ∈ [约 92, 约 240] mm 的圆环;所有任务空间目标在写舵机
  之前都会被钳制到该圆环与 reach/height 盒的交集内。
- 舵机写入不做静默夹紧:映射结果超出该关节 lower/upper(±1.5° 容差)时整帧
  拒绝,指令模型停在硬件实际所处的位姿上。

## 关节语义(新约定,必须按此重标)

| 关节 | offset 定义 | center(offset=0)的物理含义 |
| --- | --- | --- |
| 0 base | yaw − 90° | 正前方(preset:左 0°/前 90°/右 180°) |
| 1 shoulder | 大臂相对铅垂线的前倾角 | 大臂竖直 |
| 2 elbow | 小臂对地绝对俯仰角 | 小臂水平(此时动作杆与水平面成 20°) |
| 3 gripper | 开口角 − 80° | 张开(80 = 开,25 = 合) |

注意:旧固件把 IK 的肘部相对角直接当 joint 2 的 offset,旧标定值(包括曾经
的 `ElbowZeroDegrees = 10`)在新语义下全部作废,joint 2 四个标定字段必须重测。

## 锚定位形

大臂铅垂 + 动作杆与水平面 20°(小臂水平)+ 抓手张开。该位形一次性确定
joint 1 与 joint 2 的 center,且方便用水平尺/直角尺验证。

## 步骤

1. 烧录 `arm_calibration`(USB 直连,该 profile 不能 ARM,校准命令要求
   DISARMED,协议 0x11:joint, degrees)。
2. joint 1(shoulder):首条命令 90 上电就位;微调至大臂铅垂,记下原始角
   `S0`。再小幅增大命令观察大臂是否向前(reach 增大方向)倾——是则
   direction=+1,否则 −1。centerOffset = S0 − 90(direction=−1 时同式,
   固件按 direction 换算)。
3. joint 2(elbow):首条命令给 90;缓动到小臂目测水平,验证动作杆与水平面
   约 20°,记原始角 `E0`。增大命令观察小臂是否上仰(绝对角增大)——是则
   direction=+1。centerOffset = E0 − 90。
4. 行程限位:分别向两端缓动,在(a)舵机机械端点、(b)结构干涉点、(c)护栏
   Nack/停止点三者最先出现处记录 lower/upper。四杆耦合干涉不需要手动留裕
   量——运行态由折叠角带实时保证,lower/upper 只需覆盖绝对行程(抓手碰地、
   碰底盘、线缆张力等)。
5. joint 0(base):首条命令 90 = 正前;两端各转到底记录 lower/upper。要求
   总行程 ≥ 180°(机械旋转范围下限 180°):若舵机满行程即 0–180,则
   lower=0、upper=180;确认增大命令的旋转方向与 preset 左(0°)/右(180°)
   语义一致,否则 direction=−1。
6. joint 3(gripper):标出张开=逻辑 80、闭合=逻辑 25 对应的原始角,换算
   centerOffset/direction,lower/upper 留在略超出开合两点处。
7. 通过参数组 1(Servo)逐关节写入 lower/upper/centerOffset/direction,
   参数组 10(ArmGeometry)按需修正几何,快照回读核对 revision。
8. 把确认值固化进 `BuildConfig.h`(`BaseZeroDegrees` 等即各关节 center 原始
   角)与 `RuntimeConfig::defaults()` 所引用的常量,然后在生产 env 加
   `-DROBOT_ARM_CALIBRATED=1` 启用机械臂。

## 启用后的上电/ARM 行为

生产固件上电仍不 attach 舵机;第一次进入 ARMED 时 attach 并输出收纳位
(stow)。ARM 之前请把机械臂手动摆到收纳位附近,避免首帧跳变。DISARM 不
detach(保持位保持力矩),故障保持(hold)后需 ClearFault 再 ARM 恢复。

## 理论依据(数值)

- 拉杆与大臂平行间距 g = 40·sin(fold + 20°);10 mm 杆宽裸接触在
  fold ≈ 145.5°,含 M3 螺母余量取 140°,再留传动刚度余量取运行上限 135°。
- 收纳位 fold ≈ 129°、经由货物净空高度的中间路径点 fold ≈ 133°,都在带内
  但接近上限——修改 stow/clearance 参数后应重跑
  `pio test -e native`(`test_arm_joint_safety` 会验证预设与路径点)。
