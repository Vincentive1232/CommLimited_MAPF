# 同步上游仓库速查

本仓库建议使用以下分支和远程结构：

- `origin`：自己的 Fork，`git@github.com:Vincentive1232/CommLimited_MAPF.git`
- `upstream`：原始项目仓库
- `main`：只用于同步上游，不放自己的研究代码
- `feature/*`：自己的功能与实验分支

## 首次配置 upstream

先确认原始仓库地址。根据当前 README，地址应为：

```bash
git remote add upstream git@github.com:jovaldivieso/csvil.git
git remote -v
```

如果原始仓库地址不同，请将上面的 URL 替换为实际地址。

## 开始自己的功能分支

不要直接在 `main` 上开发：

```bash
git switch main
git switch -c feature/dblacam-smoke-test
```

提交修改：

```bash
git status
git add <需要提交的文件>
git commit -m "feat: add db-LaCAM smoke test"
git push -u origin feature/dblacam-smoke-test
```

应精确指定 `git add` 的文件，避免意外提交 `.vscode`、数据集、模型或其他本地文件。

## 日常同步上游

同步前先检查工作区：

```bash
git status
```

建议先提交尚未完成的有效修改，确保工作区干净。然后：

```bash
git fetch upstream
git switch main
git merge --ff-only upstream/main
git push origin main
```

这会依次更新：

```text
upstream/main → 本地 main → origin/main
```

## 更新自己的个人分支（推荐 rebase）

个人独占的功能分支建议 rebase 到最新 `main`：

```bash
git switch feature/dblacam-smoke-test
git branch backup/dblacam-smoke-test-before-sync
git rebase main
```

如果该功能分支此前已经推送到自己的 Fork：

```bash
git push --force-with-lease origin feature/dblacam-smoke-test
```

使用 `--force-with-lease`，不要使用普通的 `--force`。

## 更新多人共享的分支（使用 merge）

如果其他人已经基于该分支工作，不要改写历史：

```bash
git switch feature/<branch-name>
git merge main
git push origin feature/<branch-name>
```

## 处理 rebase 冲突

查看冲突文件：

```bash
git status
```

人工编辑冲突文件，保留最终需要的代码，然后：

```bash
git add <已解决的文件>
git rebase --continue
```

如有更多冲突，重复以上步骤。

如果不确定如何解决，可以安全取消本次 rebase：

```bash
git rebase --abort
```

取消进行中的 merge：

```bash
git merge --abort
```

## 常用检查命令

```bash
# 当前状态与分支
git status
git branch --show-current

# 查看远程仓库
git remote -v

# 查看本地和远程分支
git branch -a

# 查看尚未合入当前分支的上游提交
git log --oneline HEAD..upstream/main

# 查看当前分支相对 main 的提交
git log --oneline main..HEAD

# 查看当前未提交修改
git diff
git diff --stat

# 查看暂存区修改
git diff --cached
```

## 推荐的完整同步流程

将 `<feature-branch>` 替换为自己的分支名：

```bash
git status
git fetch upstream
git switch main
git merge --ff-only upstream/main
git push origin main
git switch <feature-branch>
git branch backup/<feature-branch>-before-sync
git rebase main
git push --force-with-lease origin <feature-branch>
```

注意：如果 `<feature-branch>` 包含 `/`，对应的 backup 分支名也可以包含 `/`；但要确保该备份分支名此前不存在。

## 核心原则

1. `main` 只跟踪上游。
2. 自己的功能放在 `feature/*` 分支。
3. 同步或 rebase 前先运行 `git status`。
4. 个人分支优先 rebase，共享分支优先 merge。
5. 不要用 `git reset --hard` 处理普通同步问题。
6. 不要提交生成的数据集、大型 checkpoint 或本地 IDE 配置，除非项目明确需要。
