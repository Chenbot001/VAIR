# 血管内导丝遥操作系统

该项目提供了一个基于键盘方向键控制的血管内导丝操作装置控制系统。它允许通过键盘输入控制两个伺服电机。

## 功能介绍

`operate_wire.py` 脚本通过串行端口与导丝操作设备建立连接，并提供键盘界面来控制两个电机：

- **上/下方向键**：控制第一个电机的角度，负责推/拉导丝
- **左/右方向键**：控制第二个电机的角度，负责逆时针/顺时针旋转导丝，控制导丝头的朝向
- **Q 键**：退出程序

每次按键会将相应的电机向指定方向调整1度。系统使用Dynamixel SDK与电机通信，并实现安全控制以防止损坏。

## 系统要求

- Python 3.7或更高版本
- Windows、macOS或Linux操作系统，且有可用的COM端口
- 通过串行端口连接的Dynamixel电机控制器

## 安装说明

### 1. 准备环境

```bash
# 创建并激活虚拟环境（可选但推荐）
python -m venv env
# Windows系统
.\env\Scripts\activate
# macOS/Linux系统
source env/bin/activate
```

### 2. 安装依赖

```bash
# 安装必需的软件包
pip install -r requirements.txt
```

### 3. 硬件连接

1. 将手术设备控制器通过USB连接到您的计算机
2. 确认其使用的COM端口（默认为COM3）
   - Windows系统：在设备管理器中查看端口（COM和LPT）
   - macOS系统：使用终端命令 `ls /dev/tty.*`
   - Linux系统：使用终端命令 `ls /dev/ttyUSB*` 或 `ls /dev/ttyACM*`

### 4. 更新端口设置（如有必要）

如果您的设备未连接到COM3，请修改 `operate_wire.py` 中的第5行：
```python
hand = SSRSurgery(port="您的COM端口")  # 替换为您的实际端口
```

### 5. 运行程序

```bash
python operate_wire.py
```

使用方向键控制电机。按"q"退出程序。

## 故障排除

- **串行端口问题**：如果您收到"未找到COM端口"或类似错误：
  - 确认设备已正确连接
  - 检查您指定的端口是否正确
  - 确保您有访问该端口的适当权限

- **权限错误**：在Linux/macOS上，您可能需要运行：
  ```bash
  sudo chmod 666 /dev/ttyUSB0  # 替换为您的端口
  ```

- **键盘模块问题**：如果在远程环境或作为服务运行，键盘输入可能无法按预期工作。
