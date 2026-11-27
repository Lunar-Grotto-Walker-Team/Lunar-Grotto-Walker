# Lunar-Grotto-Walker
月窟行者

## 技术栈

- **嵌入式**: C/C++
- **AI/仿真**: Python
- **CI/CD**: GitHub Actions
- **项目管理**: GitHub Projects (V2)

## 开发流程

请参考 [开发流程指南](docs/project/Git-Flow-Guide.md)

## 依赖与构建

第三方驱动与基础库统一收编在
[Lunar-Grotto-Walker-Vendor](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor)，
版本由根目录 [`deps.repos`](deps.repos) 固定。

```bash
vcs import src < deps.repos     # 拉取第三方包
colcon build --symlink-install  # 构建
```

完整清单、系统依赖、`/usr/local` SDK 安装步骤见
[依赖说明](docs/dependencies.md)。
