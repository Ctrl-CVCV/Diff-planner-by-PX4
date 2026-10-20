# gcs_ws 工作区版本管理

此仓库保存整个 ROS 工作区的源码、配置、launch、脚本和文档。
工作区级远程为 `https://github.com/Ctrl-CVCV/Diff-planner-by-PX4.git`。

## 收录和排除范围

- 收录 `src/` 下的 ROS 软件包，包括本机修改过的 PX4Ctrl、Diff-Planner、Livox 驱动和飞行脚本。
- 各依赖源码按普通文件收录，不使用 gitlink，因此不依赖递归克隆来获得本次快照的源码。
- 原有嵌套仓库的 `.git` 元数据、历史和 remote 保留在本机，不上传，也不删除。
- 排除 build、devel、install、log、Python 缓存、编译产物、rosbag、生成的 PCD 点云、编辑器设置、本机密钥和临时备份。
- 保留文档图片和示意 GIF；它们不是编译产物。
- 保留 `CATKIN_IGNORE` 等构建控制文件，避免克隆后意外启用原本停用的软件包。
- 不修改原有依赖内的 `.gitignore`；初次暂存显式收录未被工作区规则排除的文件，避免遗漏被依赖自身规则忽略的 ROS `package.xml` 等必要文件。

## 常用操作

始终从工作区根目录操作总仓库，避免误操作到依赖自己的仓库：

```bash
cd ~/gcs_ws
git remote -v
git status --short
git diff --cached --stat
```

源码修改后：

```bash
git -C ~/gcs_ws add -- src
git -C ~/gcs_ws diff --cached
```

现有文件已作为普通文件纳入总仓库。以后添加一个全新的嵌套仓库时，不要直接暂存其整个目录：
Git 可能把它登记为 gitlink，而不是收录源码。新增文件还可能被依赖内部的 `.gitignore` 忽略，
需要检查后再用明确的文件路径暂存，不要用 `git add -f .` 绕过所有排除规则。

本次初始化不自动创建提交或推送。正式提交前，请检查暂存内容、Git 作者身份，以及配置中是否含敏感信息；
推送前确认 GitHub 仓库现有分支和历史，避免覆盖远程内容，不使用强制推送。

## 在其他机器使用

安装对应的 ROS Noetic、硬件驱动和系统依赖后，克隆仓库，按各软件包文档重新编译。
`build/`、`devel/` 不随源码分发，不能直接复用本机编译结果。
`src/CMakeLists.txt` 是指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` 的 catkin 符号链接，
目标机器需要对应的 ROS 安装；其他 ROS 安装路径应在本机重新初始化 catkin 工作区。

本仓库保存当前配置，不表示这些参数已经完成飞行安全验证；编译或克隆完成后不要直接执行起飞脚本。
