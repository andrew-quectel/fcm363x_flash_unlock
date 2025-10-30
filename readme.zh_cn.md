# FCM363X Flash 解锁工具

## 工程概述

本工程是专为FCM363X模块设计的XMC Flash解锁工具，用于解决XMC XM25QH64D Flash芯片的各种保护锁定问题。

## 编译说明

1. **开发环境**: MCUXpresso for VS Code
2. **编译配置**: 选择 `ram_debug` 或 `ram_release` 配置

## 使用方法

### 方法1: JLink命令行烧录

1. 编译完成后，在 `armgcc/ram_debug/` 或者 `armgcc/ram_release/` 目录下找到 `freertos_hello.bin`
2. 使用JLink连接目标板
3. 执行以下JLink命令：

```bash
# 加载固件到SRAM
loadbin freertos_hello.bin 0x20000000

# 设置程序计数器到Reset_Handler（地址可能因编译而异）
setpc 0x200012fc

# 开始执行
go
```

### 方法2: MCUXpresso IDE调试

1. 导入工程到MCUXpresso IDE
2. 选择ram_debug配置
3. 连接调试器并启动调试会话
4. 运行程序

## 输出示例

```
Starting Flash JEDEC ID and Status Register reading...
Flash JEDEC ID: 0x204017

--- Flash Information ---
Vendor ID: 0x20 (XMC, Supported)
Memory Type: 0x40
Capacity ID: 0x17 (8 MB)
-------------------------

--- Status Registers ---
Status Register 1 (0x05): 0x40 [SEC=1]
Status Register 2 (0x35): 0x43 [SRP1=1-HW_PROTECTED] [CMP=1]
...

Flash protection detected. Attempting to unlock...
*** CRITICAL: Hardware Write Protection Detected! ***
Attempting volatile unlock...
Attempting non-volatile unlock...
Hardware unlock sequences completed.
Flash unlock successful!
```
