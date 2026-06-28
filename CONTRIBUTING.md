# 贡献指南

感谢你对本项目的关注！

## 开发环境

- **ESP-IDF**: v5.5.4（必需）
- **目标平台**: ESP32-P4（主控）+ ESP32-C5（协处理器）
- **操作系统**: Windows（批处理脚本），WSL/Linux 支持手动 `idf.py` 命令

## 快速开始

```bash
git clone <你的仓库地址>
cd Robot_Arm
idf.py reconfigure
```

## 如何贡献

1. **Fork** 本仓库
2. **创建功能分支**: `git checkout -b feature/your-feature`
3. **在真机上编译测试**后再提交
4. **提交**时使用清晰的提交信息：`feat: 添加 X`、`fix: 修复 Y`
5. **推送**并创建 **Pull Request**

## 提交 Issue

- 使用 GitHub Issues
- 请包含：ESP-IDF 版本、硬件版本、串口日志、复现步骤
- WiFi/连接问题请说明 C5 固件版本

## 代码规范

- 注释使用中文或中英双语
- C 代码：4 空格缩进，K&R 大括号风格
- 遵循 `main/main.c` 和 `components/` 中已有的代码风格
