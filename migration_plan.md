# 版本升级迁移计划：当前 → prepare

> **目标**：将 `prepare/` 中的最新代码合并到当前项目，覆盖 .c/.h/CMakeLists.txt 文件。
> **范围**：仅修改 .c、.h、CMakeLists.txt 文件，不动其他文件（main/ 目录结构、.cpp、配置文件等需另行确认）。

---

## Phase 0：备份与验证

| 步骤 | 操作 | 说明 |
|------|------|------|
| 0.1 | `git add -A && git commit -m "backup: before prepare migration"` | 在当前分支提交全部修改，确保可回滚 |
| 0.2 | 确认 `prepare/` 目录存在且为最终版本 | 本计划的唯一来源 |

---

## Phase 1：Middleware 层（Kinematics + Script）

**原因**：这两个模块是 main.c 的核心依赖，Kinematics 接口变了（新增 config 参数），必须先升级。

### 1.1 Kinematics 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Middleware/Kinematics/Kinematics.h` | **完全替换**为 prepare 版本（新增 `CLAMP` 宏、`gripper_length` 字段、`KinematicConfig` 枚举、`Kinematics_Inverse` 新增第3参数、新增 `Kinematics_Inverse_Iterative` 声明） |
| `components/Middleware/Kinematics/Kinematics.c` | **完全替换**为 prepare 版本（正运动学加夹爪长度；逆运动学从简单几何法→枚举搜索+评分配置；新增迭代雅可比IK；`CheckReachable` 加夹爪长度） |

**兼容性影响**：
- `Kinematics_Inverse()` 签名从 `(pos, angles)` 变为 `(pos, angles, config)` → **所有调用方必须更新**
- 新增 `Kinematics_Init()` 替代隐式初始化 → main.c 中需调用

### 1.2 Script 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Middleware/Script/Script.h` | **完全替换**为 prepare 版本（从单一 `PickAndPlace()` → 完整脚本API：`KinematicScript_MoveObject`、`Wave_Animate`、`Arm_Reset`、`Servo_SetAngle_Single`、`Gripper_*`） |
| `components/Middleware/Script/Script.c` | **完全替换**为 prepare 版本（阻塞式Servo_SetAngle → 非阻塞队列发送；新增完整运动学脚本） |

### 1.3 Middleware CMakeLists.txt

| 文件 | 变更内容 |
|------|----------|
| `components/Middleware/CMakeLists.txt` | 编译选项从 `-ffast-math -O3` → `-O2` |

---

## Phase 2：Hardware 层（I2C + PCA9685 + MPU6050 + Servo）

### 2.1 I2C 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Hardware/I2C/I2C.h` | **完全替换**（移除 `I2C_Deinit()` 声明） |
| `components/Hardware/I2C/I2C.c` | **完全替换**（移除 `I2C_Deinit()` 实现） |

### 2.2 PCA9685 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Hardware/PCA9685/PCA9685.h` | **完全替换**（移除 `PCA9685_SetDuty()` / `PCA9685_SetAllDuty()` 声明） |
| `components/Hardware/PCA9685/PCA9685.c` | **完全替换**（移除两个占空比函数实现，保留核心PWM功能） |

### 2.3 MPU6050 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Hardware/MPU6050/MPU6050.c` | **补丁**（仅修改2行：`3.1415926535f` → `M_PI`） |
| `components/Hardware/MPU6050/MPU6050.h` | **不变**（prepare 版本无变化） |

### 2.4 Servo 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Hardware/Servo/Servo.c` | **补丁**（初始化循环从 `channel < 16` → `channel < 5`，加注释） |
| `components/Hardware/Servo/Servo.h` | **不变**（prepare 版本无变化） |

### 2.5 Hardware CMakeLists.txt

| 文件 | 变更内容 |
|------|----------|
| `components/Hardware/CMakeLists.txt` | **完全替换**（移除LED源文件，编译选项 `-ffast-math -O3` → `-O2`） |

---

## Phase 3：Communication 层（WebServer + WiFi）

### 3.1 WebServer 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Communication/WebServer/WebServer.h` | **完全替换**（`servo_status_t` 新增 `gripper_angle`、`pid_pitch_error`、`pid_pitch_output`、`target_pitch`；`WebServer_Deinit()` 移除） |
| `components/Communication/WebServer/WebServer.c` | **完全替换**（WebSocket心跳检测、HTML大幅重写含gripper/PID/kinematic UI、HTTP回退命令解析增强） |

### 3.2 WiFi 模块

| 文件 | 变更内容 |
|------|----------|
| `components/Communication/WiFi/WiFi.c` | **不变**（prepare 版本无差异） |
| `components/Communication/WiFi/WiFi.h` | **不变**（prepare 版本无差异） |

### 3.3 Communication CMakeLists.txt

| 文件 | 变更内容 |
|------|----------|
| `components/Communication/CMakeLists.txt` | **补丁**（移除 `esp_wifi_remote` 依赖，编译选项 `-ffast-math -O3` → `-O2`） |

---

## Phase 4：主程序（main.c 重写）

### 4.1 文件位置变更

| 当前 | prepare |
|------|---------|
| `main/main.c` (545行) | `prepare/main.c` (763行，项目根目录) |

**关键决策**：prepare 将 main.c 放在项目根目录（ESP-IDF 单文件入口模式），而当前版本使用 `main/main.c`（ESP-IDF 组件模式）。

**方案 A（推荐）**：将 prepare/main.c 复制到 `main/main.c`，保持现有目录结构不变。
- 优点：不破坏现有 CMake 结构
- 缺点：需要修改 `main/CMakeLists.txt` 的 REQUIRES

**方案 B**：将 prepare/main.c 放到项目根目录，修改顶层 CMakeLists.txt。
- 需要改动 CMake 结构，风险较大

### 4.2 执行步骤（采用方案 A）

| 步骤 | 操作 |
|------|------|
| 4.2.1 | 将 `prepare/main.c` 的内容写入 `main/main.c`（完全替换） |
| 4.2.2 | 修改 `main/CMakeLists.txt`：添加 `Kinematics` 和 `Script` 到 REQUIRES |

**main.c 核心变更**：
- 新增 PID 控制器（结构体 + Init/Compute/Reset）
- 新增 `map_IMU_To_Servos()` 角度映射（含翻转防抖）
- `IMU_Task` → `IMU_Feedback_Task`（100Hz 末端MPU6050反馈）
- `Control_Task` 重写（50Hz PID闭环控制，含雅可比伪逆分配）
- `Servo_Task` 重写（互斥锁+非阻塞队列）
- 新增 `WebStatus_Task`（20Hz 状态发布含PID数据）
- `web_command_handler` 大幅扩展（reset/wave/mode/phone/servo/gripper/kinematic_move/kmove）
- `app_main` 重写（创建 Kinematics、队列、任务）
- WiFi SSID 从硬编码 `ServoArm_AP` → `Arm_AP`（支持 sdkconfig 覆盖）

### 4.3 main/CMakeLists.txt 变更

当前：
```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES Middleware Communication bt nvs_flash esp_wifi_remote)
```

需要添加 `Kinematics` 和 `Script` 到 REQUIRES（因为 main.c 新包含 `Kinematics.h` 和 `Script.h`）。

---

## Phase 5：编译依赖验证

| 检查项 | 说明 |
|--------|------|
| `main/CMakeLists.txt` REQUIRES | 必须包含 `Kinematics` 和 `Script` |
| `sdkconfig` | 确认 WiFi SSID 配置兼容 |
| 移除 `bt` 依赖 | prepare 的 main.c 不再需要 BLE |
| 移除 `esp_wifi_remote` | prepare 的 Communication/CMakeLists.txt 已移除 |

---

## 变更影响矩阵

| 模块 | 新增 | 修改 | 删除 | 风险等级 |
|------|------|------|------|----------|
| Kinematics | `Kinematics_Forward`、`Kinematics_Inverse_Iterative`、`Kinematics_Init`、`ScoreConfig`、`ComputeJacobian` | `Kinematics_Inverse` 签名变更 | `g_arm_params` 隐式初始化 | 🔴 高（API签名变更） |
| Script | 6个新函数 | 整体重写 | `PickAndPlace` | 🟡 中（调用方式变更） |
| WebServer | WebSocket心跳、gripper/PID UI | HTML重写、命令解析增强 | `WebServer_Deinit` | 🟢 低 |
| I2C | 无 | 无 | `I2C_Deinit` | 🟢 低 |
| PCA9685 | 无 | 无 | `SetDuty`/`SetAllDuty` | 🟢 低 |
| MPU6050 | 无 | 2行常量替换 | 无 | 🟢 低 |
| Servo | 无 | 初始化通道数 | 无 | 🟢 低 |
| main | PID控制器、新任务 | 整体重写 | `IMU_Task`旧版 | 🔴 高（核心逻辑变更） |

---

## 执行顺序（依赖图）

```
Phase 1 (Kinematics + Script)
    ↓
Phase 2 (Hardware 底层)
    ↓
Phase 3 (WebServer)
    ↓
Phase 4 (main.c — 依赖以上所有模块)
    ↓
Phase 5 (CMakeLists 验证)
```

---

## 注意事项

1. **`Kinematics_Inverse` 签名变更**是最关键的破坏性变更 — 任何调用旧签名的代码都会编译失败
2. **Servo 调用方式变更** — 从阻塞 `Servo_SetAngle(ch, angle)` 改为非阻塞 `Servo_SetAngle_Single(queue, ch, angle)` 或队列发送 `ServoCommandGroup`
3. **`gripper_angle` 范围** — 从 0-180° 变为 0-90°（prepare 的 Web UI 和 Servo_Init 都是 5 通道 0-90°）
4. **PID 控制器** — 新增的 PID 仅在 mode=1 时生效，mode=0 时自动重置
5. **WiFi SSID** 从 `ServoArm_AP` 改为 `Arm_AP`，如需要保持一致请手动改回
