# 本地开发检查

先按[安装](linkerbot_hand_ros/docs/zh/quick_start.md)或[更新](linkerbot_hand_ros/docs/zh/update.md)
准备 Transport、Hand Python 和 ROS 的最新 `main`，构建消息包及所有消费者。
依赖源码更新后重新构建，不从另一 Python 环境借用已生成的 ROSIDL 类型。

在独立测试终端、仓库根目录运行：

先设置 `export LINKERBOT_ROS_DISTRO=humble` 或 `export LINKERBOT_ROS_DISTRO=jazzy`。

```bash
source "/opt/ros/${LINKERBOT_ROS_DISTRO}/setup.bash"
source "$HOME/venvs/linkerbot-hand-${ROS_DISTRO}/bin/activate"
source "$HOME/linkerbot_${ROS_DISTRO}_ws/install/setup.bash"
cd "$HOME/linkerbot_${ROS_DISTRO}_ws/src/linkerbot-hand-ros"
python -m pip install -r requirements/ci-lint.txt -r requirements/ci-test.txt
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q -p pytest_timeout -p pytest_cov.plugin \
  linkerbot_hand_ros/tests linkerbot_hand_gui/test linkerbot_hand_msgs/test \
  --timeout=60 --cov=linkerbot_hand_ros --cov=linkerbot_hand_gui \
  --cov-report=xml:reports/coverage.xml --junitxml=reports/pytest.xml
ruff check .
ruff format --check .
```

驱动 pytest 默认使用 Domain 87 和本机发现，保留显式设置的环境变量。启动时探测
5 秒，为启动期间的发现公告重发留出时间；发现其他 ROS 节点就停止并提示更换域号，
87 只是默认值，不保证长期空闲。
并行测试进程需指定不同的空闲域；pytest-xdist worker 使用基础域号加 worker 序号，
例如基础域 87 对应 `gw0=87`、`gw1=88`。建议在 Linux 上选用 1–101 范围内的域号，
并避免与其他测试任务重叠。探测仅检查启动窗口，不能阻止其他进程随后加入。

测试使用替身/内存协议验证软件行为；适配器通信、真实受压响应和机械动作需要对应硬件验证。
命令发送、完整触觉与 ROS 发布频率的测试方法见[频率测试](linkerbot_hand_ros/docs/zh/frequency_benchmark.md)。
变更记录通过 Git 历史及对应 PR 查看。

## Docker 验证 Humble 与 Jazzy

安装 Docker 并确保当前用户能访问 Docker daemon。先按安装指南准备 Transport 和
Hand Python 的最新 `main` 源码；无需在宿主机安装 ROS 或 Python 依赖。在本仓库根目录运行：

```bash
bash tests/docker/run.sh humble \
  "$HOME/linkerbot_deps/linkerbot-transport" "$HOME/linkerbot_deps/linkerbot-hand"
bash tests/docker/run.sh jazzy \
  "$HOME/linkerbot_deps/linkerbot-transport" "$HOME/linkerbot_deps/linkerbot-hand"
```

脚本分别使用官方 `ros:humble-ros-base`（Ubuntu 22.04）和 `ros:jazzy-ros-base`
（Ubuntu 24.04）构建测试镜像，通过 rosdep 安装三个包的系统依赖，在独立 Python
环境中安装指定的 SDK 源码、编译消息包/驱动/GUI，并运行三个包的测试。
源码以只读方式挂载，构建发生在容器内；容器退出后自动删除。脚本使用传入目录的
当前内容，不自动更新依赖，测试前应确认其分支及提交。
默认结果位于 `tests/docker/results/<发行版>/`，第四个参数可指定结果目录。
保存源码提交与修改状态、构建/测试日志、colcon 日志、JUnit 和覆盖率 XML；失败返回非零。
构建同时检查三个 ROS 包安装目录中的许可证文件。
测试容器使用本机发现，默认域号 87；可在运行前设置 `ROS_DOMAIN_ID` 更换基础域号，
同样会检查是否已有其他 ROS 节点。

若已按 [GUI 资源准备](linkerbot_hand_gui/README.md#准备模型资源)下载完整模型，
运行前设置 `export LINKERBOT_HAND_ASSETS_ROOT=/实际路径/linker-sim-assets`，脚本会将
该目录只读挂载并执行资源测试；未提供时会明确跳过 4 项外部资源测试。
Jazzy 测试容器将 Fast DDS 设置为 UDPv4。生命周期启动和域占用检查在独立
Python 进程中创建 ROS context，并继承选定的测试域，避免整套测试反复初始化后的
发现状态影响跨进程验证。检查保留实际服务、事件与域占用断言，不重发生命周期变更。

默认从 Docker Hub 获取基础镜像；需要使用镜像源时可通过 `ROS_TEST_BASE_IMAGE`
指定同一 ROS 发行版的基础镜像。构建会读取标准 `HTTP_PROXY`、`HTTPS_PROXY` 和
`NO_PROXY` 环境变量；Linux 上若代理只监听宿主 localhost，可设置
`ROS_TEST_BUILD_NETWORK=host`，仅让镜像构建使用宿主网络，测试容器仍使用独立网络。
测试不连接硬件或打开 GUI 图形窗口，不能代替实机与模型预览验证。

## GitHub CI

PR、`main` 更新、每周及手动运行执行 Ruff lint/format、ShellCheck、actionlint，
并分别在 Humble 和 Jazzy Docker 环境中构建三个包、运行离线测试。依赖使用 GitHub
上 Transport 和 Hand Python SDK 的 `main`，无需 Gitea 访问或设备凭据。
测试工具版本固定在 `requirements/ci-*.txt`，显式加载超时和覆盖率插件。
覆盖率先记录基线，不设置最低阈值；`reports/` 结果作为 Actions artifacts 保留 14 天。
默认未提供 GUI 外部模型资源时跳过对应测试，CI 不执行发布、实机动作或图形窗口验证。

## 在个人 fork 验证 CI

Actions 的手动运行入口支持 `dependency_owner` 和 `dependency_ref`：默认仍使用
`linker-bot` 依赖，以及原有的 main/配对提交规则。跨仓库改动可先将 Transport、
Hand/Arm Python SDK 和两个 ROS 仓库推送到个人 fork 的同名测试分支，再统一指定
个人账号与该分支，避免混用上游尚未更新的测试脚本。日志记录实际使用的源码提交。

新 fork 的工作流需要启用 Actions，且默认分支上需要包含工作流文件，才能手动运行；
可以仅将个人 fork 的默认分支设为测试分支。上游默认分支不受影响。

手动运行同时验证 Humble 与 Jazzy。
