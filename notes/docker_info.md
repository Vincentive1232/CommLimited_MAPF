# 研究容器使用说明

记录使用 Podman 构建和运行统一研究环境的方法。当前直接使用 Podman，不依赖 Docker Engine 或 Compose provider。

## 1. 构建镜像（宿主机执行）

在仓库根目录初始化源码依赖：

```bash
git -c url."https://github.com/".insteadOf=git@github.com: \
  submodule update --init --recursive
```

首次构建前下载官方 motion primitives。已有文件时先校验，无需重复下载：

```bash
mkdir -p data/dependencies
curl -fL --retry 3 \
  -o data/dependencies/db-lacam-primitives.zip \
  https://tubcloud.tu-berlin.de/s/wezMej9ieNjwjz6/download

echo 'f52afb14e391c8ad5d7b71e641917f6a2f65d627fbece76899246b4b62468190  data/dependencies/db-lacam-primitives.zip' \
  | sha256sum --check -
```

上述 SHA256 固定本次已验证下载包的内容，并非上游提供的真实性证明。压缩包位于被 Git 忽略的 `data/` 中，不能只克隆仓库就假定文件已存在。

构建镜像：

```bash
podman build \
  --format docker \
  --platform linux/amd64 \
  -f docker/Dockerfile.research \
  -t localhost/jli/commlimited-mapf:research-base \
  .
```

`--format docker` 保留 Dockerfile 的 `SHELL` 设置，执行工具仍是 Podman。多行命令中的反斜杠 `\` 必须位于行末，后面不要添加空格。

## 2. 直接创建和进入开发容器

以下命令无需配置 `.bashrc`。首次创建时，在宿主机仓库根目录执行：

```bash
podman run -d \
  --name jli-commlimited-research \
  --userns=keep-id \
  --device nvidia.com/gpu=all \
  -v "$PWD:/workspace" \
  -w /workspace \
  localhost/jli/commlimited-mapf:research-base \
  sleep infinity
```

容器名称已存在时，不要重复执行创建命令。若容器已停止，先启动：

```bash
podman start jli-commlimited-research
```

进入运行中的容器：

```bash
podman exec -it -w /workspace jli-commlimited-research bash
```

GPU 透传依赖宿主机 NVIDIA 驱动和 CDI 配置，当前机器已验证可用。`--userns=keep-id` 保持挂载目录中的文件使用当前用户身份。

## 3. Bash 快捷命令

在宿主机 `~/.bashrc` 中添加以下函数；如果已有同名函数，修改原定义，不要重复添加。根据实际仓库位置调整 `project_dir`：

```bash
cmapf-research() {
    local container_name="jli-commlimited-research"
    local project_dir="$HOME/workspace/CommLimited_MAPF"

    if podman container exists "$container_name"; then
        if [ "$(podman inspect --format '{{.State.Running}}' "$container_name")" != "true" ]; then
            podman start "$container_name" >/dev/null || return
        fi
    else
        podman run -d \
            --name "$container_name" \
            --userns=keep-id \
            --device nvidia.com/gpu=all \
            -v "$project_dir:/workspace" \
            -w /workspace \
            localhost/jli/commlimited-mapf:research-base \
            sleep infinity >/dev/null || return
    fi

    podman exec -it -w /workspace "$container_name" bash
}
```

保存后在宿主机执行：

```bash
source ~/.bashrc
cmapf-research
```

首次调用创建容器，之后启动或进入已有容器。旧的 `cmapf` 指向 `jli-commlimited-gpu`，保留用于旧环境对照。

## 4. 容器内开发与路径

| 内容 | 容器内路径 |
|---|---|
| 项目根目录 | `/workspace` |
| Python 虚拟环境 | `/opt/venv` |
| Python 可执行文件 | `/opt/venv/bin/python` |
| db-LaCAM 源码副本 | `/opt/db-lacam` |
| db-LaCAM 可执行文件 | `/opt/db-lacam/buildRelease/run_dblacam` |
| OMPL | `/opt/ompl` |
| robotpkg 依赖 | `/opt/openrobots` |
| 构建记录 | `/opt/build-metadata` |

宿主机仓库绑定挂载到 `/workspace`，两边编辑的是同一份项目文件。Python 代码修改无需重建镜像。`/opt/db-lacam` 是构建时复制的源码，宿主机 submodule 的修改不会自动更新它。

直接调用 `run_dblacam` 时，将工作目录设为 `/opt/db-lacam/buildRelease`；当前上游源码使用相对路径寻找模型和 primitives。项目命令仍在 `/workspace` 下运行。

容器内执行 `exit` 只退出当前交互 Shell，容器继续运行。需要停止时，在宿主机执行：

```bash
podman stop jli-commlimited-research
```

重建镜像不会更新已存在的容器。采用新镜像时需要重新创建开发容器；此前应先保存容器中未放在挂载目录里的文件，并把需要保留的依赖安装写入 Dockerfile。快捷函数不会自动删除或替换旧容器。

## 5. 当前已验证结果

- Python 3.12.13，项目使用 `/opt/venv`。
- PyTorch 2.7.1+cu128，实际 CUDA 张量运算通过。
- 实际 GPU 为 **NVIDIA RTX 4000 Ada Generation**，不是计划暂记的 RTX A4000。
- CasADi 3.6.7，IPOPT 简单优化求解通过。
- db-LaCAM 固定到 `91dbf49239179280fc85789509811eac04a3d1a7`，无需修改上游源码即可编译。
- `run_dblacam` 动态库检查无 `not found`。
- 官方 `circle2_integrator.yaml` 场景生成了结果文件；两次日志中的 planner 自报总耗时约为 175 ms 和 193 ms，不含容器启动时间，不能作为完整性能基准。
- 开发容器可访问 GPU，且 `/workspace/data`、`/workspace/outputs/checkpoints`、`/workspace/outputs/logs` 可写。

初期 CasADi 和 CUDA 运算检查是在逐步构建的镜像上完成的；最终镜像验收时应再次检查，以覆盖后续安装的 C++ 依赖。

## 6. 依赖兼容性记录

OMPL 使用固定版本 db-LaCAM CI 指定的 `e2994e5`，完整 SHA 保存在镜像的 `/opt/build-metadata/ompl.commit`。C++ 构建显式使用 `/usr/bin/cmake`，避免使用 pip 安装的 CMake 4.x。

Crocoddyl 3.2.1r1 使用的 `std::shared_ptr` 接口与当前 Dynoplan 的 `boost::shared_ptr` 调用不兼容。当前 Dockerfile 固定以下通过构建的组合：

| robotpkg 包 | 版本 |
|---|---|
| py310-crocoddyl | 2.1.0r1 |
| pinocchio / py310-pinocchio | 3.4.0 |
| py310-eigenpy | 3.10.3 |
| example-robot-data / py310-example-robot-data | 4.2.0 |
| coal / py310-coal | 3.0.1 |
| casadi / py310-casadi | 3.6.7 |
| qpoases+doc | 3.2.1r1 |

表内包名均省略了 `robotpkg-` 前缀。系统 Python 3.10 绑定不加入项目 Python 3.12 的搜索路径。robotpkg 当前使用上游 CI 的 HTTP 软件源地址和独立签名 keyring。

Primitives 的压缩包 checksum 写在 Dockerfile 中；实际 single-integrator 二进制的 checksum 保存在 `/opt/build-metadata/integrator1-primitives.sha256`。

## 7. 尚未完成的验收

官方求解 smoke 仅确认调用链可用，不代表 Milestone 1 已完成。仍需验证统一场景的参数、原始 controls 重放、连续碰撞检查、严格目标与三步 hold，以及失败分类。

两种 planner 的 PlannerFactory smoke 和现有 CasADi 回归测试仍需在研究环境中完成，并补齐最终运行元数据。

目前按开发偏好直接使用 Podman 和 Bash 快捷函数；计划中的 Compose `research-gpu` service 尚未接入，不应记为已完成。
