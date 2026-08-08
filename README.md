项目文档 — 蓝牙 PID 调参与持久化（适用于 STM32F103C8T6）

概述
- 本项目通过蓝牙（连接到 USART3 / huart3）接收文本命令，实现运行时调整 PID 参数、读取当前参数，以及远程控制小车运动。
- 命令为单行 ASCII 文本，通常以 CR+LF 结束，由 HAL_UARTEx_ReceiveToIdle_IT 的空闲回调触发处理。

命令集合与允许范围
- 设置参数（KEY=VALUE）
  - V_KP=float    -- 直立环 Kp，范围 0.0 .. 1000.0
  - V_KD=float    -- 直立环 Kd，范围 0.0 .. 10.0
  - S_KP=float    -- 速度环 Kp，范围 0.0 .. 5.0
  - S_KI=float    -- 速度环 Ki，范围 0.0 .. 1.0
  - ST_KP=float   -- 转向环 Kp，范围 0.0 .. 20.0
  - ST_KD=float   -- 转向环 Kd，范围 0.0 .. 5.0
  - T_SPEED=int   -- 目标速度，范围 -2000 .. 2000
  - T_ANGLE=int   -- 目标角度，范围 -30 .. 30

- 查询参数
  - GET ALL       -- 返回所有当前参数值（单行字符串）

- 运动控制（CMD）
  - CMD=FORWARD[:pwm]   -- 前进，pwm 为可选整数（默认1000）
  - CMD=BACKWARD[:pwm]  -- 后退
  - CMD=LEFT[:pwm]      -- 向左原地转
  - CMD=RIGHT[:pwm]     -- 向右原地转
  - CMD=STOP            -- 停止电机

- 持久化（Flash）
  - SAVE           -- 把当前 PID 参数写入 Flash（手动触发以避免频繁擦写）
  - LOAD           -- 从 Flash 读取并应用已保存的参数

响应示例
- 设置成功：OK V_KP=210.500\r\n
- 查询返回（示例）：V_KP=200.000 V_KD=2.040 S_KP=0.600 S_KI=0.003 ST_KP=1.000 ST_KD=0.100 T_SPEED=0 T_ANGLE=0\r\n
- 保存成功：OK SAVE\r\n
- 加载成功：OK LOAD\r\n
- 错误示例：ERR no_equal\r\n 或 ERR unknown_key\r\n

Flash 存储实现细节
- 设备：本仓库为 STM32F103C8T6（64KB Flash）提供了默认保存页：0x0800FC00（即最后 1KB 页）。
  - 如果你的板子不是 STM32F103C8T6，请确认 Flash 大小与页面起始地址，并在 App/Inc/flash_storage.h 中修改 FLASH_SAVE_ADDR。
- 存储格式：结构体 (magic 'PIDF', 参数字段..., checksum)，checksum 为结构除最后字段外的 32-bit 累加和。
- 写入流程：擦除目标页（1 page）、按 32-bit 单元编程，并锁定 Flash。
- 读取流程：校验 magic 与 checksum，成功则把值原子写入运行时全局变量。
- 注意：Flash 擦写寿命有限（通常数万次），请避免频繁写入。推荐手动 SAVE 或实现节流/磨损均衡策略。

构建注意（Release/体积）
- 为了减少固件体积：
  - 当前默认使用 Release 构建（-Os），并将浮点 printf 支持移除以节省空间；因此响应中浮点以定点/整数分解方式格式化。
  - 若需要恢复 %f 支持（更直观的浮点输出），可在 CMakeLists.txt 中恢复链接选项 -u _printf_float，但会显著增加二进制体积。
- 构建命令示例（已在本机验证成功）：
  - cmake --build build/Release --config Release
  - 或在 IDE（CubeIDE / VSCode）选择 Release 配置并构建

串口/蓝牙测试指南（精确设置）
- 串口参数：9600, 8, N, 1, Flow control None
- 行结束：请选择 CR+LF（"Both NL & CR" / "CR+LF"）以保证兼容

快速验证序列（建议在台架或确保安全的环境）
1. 连接蓝牙串口（手机或 PC 串口助手）并设置 CR+LF 结尾
2. 发送：GET ALL\r\n --> 检查返回的当前参数
3. 发送：V_KP=210.5\r\n --> 应返回 OK V_KP=210.500\r\n
4. 发送：SAVE\r\n --> 应返回 OK SAVE\r\n
5. 断电并上电（或复位 MCU）
6. 发送：GET ALL\r\n --> 检查 V_KP 是否为 210.5（若是，持久化生效）

故障排查
- 无响应：确认蓝牙与 MCU 的物理连线（BT TX → MCU RX, BT RX → MCU TX）、波特率、以及 bluetooth_init() 是否在 main 中调用。
- SAVE 失败或 LOAD 失败：检查 FLASH_SAVE_ADDR 是否与 MCU Flash 布局匹配，以及程序是否有擦写权限（HAL_FLASH APIs 可用）。
- 链接错误（strtof/printf 等）：若遇到链接错误，请把完整链接错误输出贴上来，我可以改为更轻量的解析或恢复浮点 printf 支持。

关键源码位置
- 蓝牙接收与命令分发： App/Src/bluetooth.c
- 命令解析与 PID 变量： App/Src/pid.c 以及 App/Inc/pid.h
- Flash 存储： App/Src/flash_storage.c 以及 App/Inc/flash_storage.h
- 电机控制底层： Lib/Src/motor.c 以及 Lib/Inc/motor.h

后续建议
- 若需自动化或远程保存策略（节流/轮换页写入），我可以实现：
  - 写入节流：合并多次修改后延迟写入（例如修改后 60s 内不再写）
  - 简单磨损均衡：轮换多页写入并保存最新页索引（需要管理索引与回收策略）

需要我现在帮你：
- 把 README.md 提交为仓库的一次 commit（包括 Co-authored-by trailer）？（我可以直接提交）
- 或者生成一份更短的快速使用说明打印到串口启动时？