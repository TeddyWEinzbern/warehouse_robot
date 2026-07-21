# 机械臂校准手册

本手册面向第一次接触本项目的使用者:每一步都给出可直接复制的命令和要改的
文件位置,不需要读固件源码。唯一前提:已按 [README](../README.md) 装好
PlatformIO 与 Python 环境(`python3 -m pip install -e '.[test]'`),并且
机械臂舵机已接线、舵机电源可用。

## 0. 三句话原理(可跳过)

elbow 舵机装在底座上,通过平行四边形连杆控制的是小臂**相对地面**的角度,
不是肘关节的相对角度;所以"肘部还能转多少"取决于肩的位置。固件已内置
防撞护栏(肩肘耦合折叠角限制),校准时把关节推过头会被固件拒绝或停住,
不会撞坏连杆。你要做的只有:给 4 个关节找出零位、两端、方向,共 16 个数,
填进一个文件,重新烧录。

## 1. 烧录校准固件

在仓库根目录执行(若 Uno 的 D0/D1 接着电机驱动板,先拔掉再上传):

```sh
pio run -t upload -e arm_calibration
```

## 2. 找到串口并启动校准会话

```sh
warehouse-robot list-ports
# macOS 形如 /dev/cu.usbmodem14101,Windows 形如 COM5
warehouse-robot calibrate --port /dev/cu.usbmodem14101
```

注意:

- 打开 USB 串口会让 Uno 复位,程序自动等 2 秒,这是正常现象。
  也因此**不要**用一次性的 `calibrate-joint` 做微调——它每条命令都重开
  串口、复位板子、舵机泄力。`calibrate` 会话全程只打开一次串口。
- 上电后所有舵机都是松的(固件不会自己动),机械臂会在重力下垂,请用手
  扶住或让它靠在支撑上。
- 每个关节的**第一条**命令会让该舵机直接跳到目标角(之后的命令都是
  60°/s 缓动)。所以每个关节的第一条命令永远发 `90`,并扶稳臂。

会话内命令(输入 `help` 随时可见):

| 命令 | 作用 |
| --- | --- |
| `j2 95` | 2 号关节转到 95°(关节号:0 底座,1 肩,2 肘,3 夹爪) |
| `j2 +2` / `j2 -1` | 在上一条命令基础上微调 |
| `mark j2 center` | 把当前角度记为该关节的 center(可用:lower/upper/center;夹爪用 open/closed) |
| `dir j1 -1` | 记录方向符号 |
| `s` | 查看各关节命令角、固件回报角、已记录的值、折叠角估计 |
| `export` | 打印可直接粘贴进 BuildConfig.h 的代码块 |
| `q` | 退出(舵机保持当前位置) |

## 3. 逐关节校准

按 **肩 → 肘 → 底座 → 夹爪** 的顺序做(肘的护栏需要肩先有位置)。

### 3.1 肩(j1)

1. `j1 90`,扶稳,舵机就位。
2. 用 `j1 +1` / `j1 -1` 微调,直到大臂**竖直**(靠直角尺或目测铅垂)。
3. `mark j1 center`
4. 方向:发 `j1 +5`,看大臂是否向机器人**前方**倾斜。是→`dir j1 +1`;
   反了→`dir j1 -1`。然后 `j1 -5` 回到竖直。
5. 两端:一路 `j1 +5` 直到出现下列任一情况就 `j1 -2` 退回并
   `mark j1 upper`:(a) 结构顶死/舵机异响,(b) 命令被固件拒绝,
   (c) 会话提示折叠角接近 138°。反方向同理 `mark j1 lower`。

### 3.2 肘(j2)

1. 先让 j1 回 center(`j1 90` 附近你 mark 的值)。
2. `j2 90`,扶稳。
3. 微调到小臂**水平**。检查:此时舵机的白色动作杆应与水平面约成 20°。
4. `mark j2 center`
5. 方向:发 `j2 +5`,小臂**上仰**则 `dir j2 +1`,反之 `-1`。
6. 两端同肩;`s` 会显示折叠角估计(护栏在 138° 硬性挡住,被挡是正常的,
   说明该端行程已到——退 2° 记录即可)。

### 3.3 底座(j0)

1. `j0 90`,微调到机械臂指向**正前方**,`mark j0 center`。
2. 方向:`j0 +10`,臂向机器人**右侧**转则 `dir j0 +1`(预设语义:
   0°=左,90°=前,180°=右),反之 `-1`。
3. 两端到底记录 lower/upper。机械设计要求总行程 **≥ 180°**;若
   `export` 警告不足 180°,检查是不是被线缆缠绕挡住了。

### 3.4 夹爪(j3)

1. `j3 80`,微调到**完全张开但不顶死**,`mark j3 open`。
2. 缓慢闭合(每次 `-2`)到两指刚好接触,`mark j3 closed`。
3. 方向:若"从 open 往**更小**的角度走"是闭合方向,`dir j3 +1`;
   若要往更大角度走才闭合,`dir j3 -1`。

## 4. 导出并写回固件

在会话里输入 `export`,会打印类似:

```cpp
// Paste over the matching lines in src/app/BuildConfig.h
constexpr uint8_t BaseZeroDegrees = 92;
constexpr uint8_t ShoulderZeroDegrees = 88;
constexpr uint8_t ElbowZeroDegrees = 95;
constexpr uint8_t GripperOpenDegrees = 78;
constexpr uint8_t GripperClosedDegrees = 24;

constexpr uint8_t ServoLowerDegrees[4] = {2, 15, 20, 19};
constexpr uint8_t ServoUpperDegrees[4] = {178, 165, 170, 83};
constexpr int8_t ServoDirectionSign[4] = {1, -1, 1, 1};
```

打开 `src/app/BuildConfig.h`,搜索
`Per-joint servo calibration`,把上面这些**同名常量逐行替换**(五个
`...Degrees` 单值常量在注释块上方不远处,三个数组紧跟在注释块下方)。
不需要改其他任何文件——`RuntimeConfig` 的默认值直接引用这些常量。

`export` 末尾若有 `// WARNINGS:`,说明有值没 mark(用了默认值)或底座
行程不足,回到会话补测,不要带着警告进入下一步。

## 5. 验证并启用机械臂

```sh
pio test -e native        # test_arm_joint_safety 会检查几何一致性
pio run -t upload -e arm_calibration   # 重新烧录,复查各 center 是否还准
```

确认无误后,打开 `platformio.ini`,找到 `[env:uart_closed_loop_robot]`,
在其 `build_flags` 列表末尾**加一行**:

```ini
[env:uart_closed_loop_robot]
extends = uno
build_flags =
    ${uno.build_flags}
    -DROBOT_BACKEND_UART=1
    -DROBOT_UART_CLOSED_LOOP=1
    -DROBOT_DRIVE_QUALIFICATION=0
    -DROBOT_DRIVE_CALIBRATION_QUALIFIED=0
    -DROBOT_ARM_CALIBRATION=0
    -DROBOT_ARM_CALIBRATED=1
```

重新构建烧录该环境。生产固件上电**不会**动臂;第一次 ARM 时舵机才通电
并走到收纳位——ARM 之前请把机械臂手动摆到收纳位附近(臂收拢、抬高),
避免跳变。DISARM 不会断舵机(臂不会掉),故障保持后先 ClearFault 再 ARM。

## 6. 故障排查

| 现象 | 原因与处理 |
| --- | --- |
| 命令回报 `blocked by the shoulder/elbow coupling guard` | 撞到肩肘耦合护栏(折叠角出 [-5°, 138°]),这是保护不是故障;往回退,或先动另一个关节 |
| 命令回报 `invalid state` | 烧的不是 `arm_calibration` 固件,或固件不在 DISARMED(会话启动时会自动发 DISARM,重启会话即可) |
| 缓动中途臂停住不到目标 | 同护栏;`s` 查看折叠角估计 |
| 每条命令后臂先泄力再猛跳 | 你在用 `calibrate-joint` 单发命令;改用 `calibrate` 会话 |
| `serial device ... is not available` | 端口名写错;重跑 `warehouse-robot list-ports`,macOS 必须带 `/dev/` 前缀 |
| 首条命令关节猛跳 | 第一条命令没有缓动;先扶住臂,并让第一条命令接近关节当前实际位置 |
| `s` 里 telemetry 一直是 `-` | 遥测约 1 秒一帧,稍等;一直没有则检查固件是否为 `arm_calibration` |

## 附:理论背景(供好奇者)

拉杆与大臂的平行间距 g = 40·sin(折叠角 + 20°),10 mm 杆宽在 145.5° 裸
接触,含螺母余量取 140°,运行态再留传动刚度余量收紧到 135°(校准态放宽
到 138°)。折叠角只由腕点到肩轴的距离决定(d = 240·cos(折叠角/2)),
所以运行固件把它实现为一个环形可达域并对所有目标自动投影——这就是
"校准时不需要为耦合手动留余量"的原因。
