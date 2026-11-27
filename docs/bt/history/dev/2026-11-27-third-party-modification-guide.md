# 第三方仓库修改与入库通用指南

> 日期：2026-11-27
> 适用范围：项目中使用官方/第三方仓库（驱动、SDK、算法栈），并对其做了本地修改，
> 需要纳入团队仓库（`src/`、submodule、vendor 目录）的情况。
> 本文是通用流程；本项目的具体落地见 `2026-11-26-github-upload-plan.md`。
> 文中 `agx_arm_ros`、`BehaviorTree.ROS2` 仅作示例。

---

## 0. 术语

| 术语 | 含义 |
|---|---|
| 上游（upstream） | 第三方仓库的原始来源（如 `agilexrobotics/agx_arm_ros`） |
| 基线（baseline） | 你开始修改时的上游 commit；一切 diff 都以它为参照 |
| 消费方（consumer） | 引用该依赖的仓库（如团队 monorepo） |
| 钉死（pin） | 用 commit/tag 固定引用，不使用浮动分支 |

---

## 1. 五条核心原则

1. **单一来源**：同一份第三方代码只存在一个权威副本，其他位置都是引用，不复制。
2. **改动可追溯**：必须能回答"相对上游改了什么、改在哪个基线、为什么改"。
   没有基线 = 无法 diff，这正是本项目的教训。
3. **消费方钉死 commit**：引用方固定到具体提交或 tag；升级是一次显式操作，不是隐式拉取。
4. **许可与署名**：保留上游 `LICENSE`/`NOTICE`；分发时说明修改（Apache-2.0 要求标注被改文件）。
5. **最小 vendoring**：只引入实际使用、且确实改过的部分；未改的走上游引用或包管理器。

---

## 2. 决策矩阵：五种模式

| 模式 | 触发条件 | 交付形态 | 代价 |
|---|---|---|---|
| **A. 直接引用** | 未改动 | `deps.repos` / apt / pip / 上游 URL + commit | 无 |
| **B. 补丁文件** | 改动小（约 < 50 行）、稳定、偶尔 | `patches/*.patch` + 引导脚本 `git apply` | 上游变动时补丁可能失效 |
| **C. fork + submodule** | 有实质改动、需独立历史/review、长期跟上游 | 独立仓库 + 消费方 submodule 固定 commit | 多一个仓库；克隆需 `--recursive` |
| **D. 平铺 vendor** | 整栈照搬、不打算跟上游 | 代码直接进 `src/` | 体积；上游更新只能手工 diff |
| **E. 上游 PR** | 通用 bug / 缺失特性 | 合并进上游后用 release | 周期不可控，但一劳永逸（终态） |

决策顺序：

```
未改动？ ──是──> A 直接引用
   │否
改动是否通用、上游可能接受？ ──是──> 提 PR（E），合并后回到 A
   │否
改动 < ~50 行且稳定？ ──是──> B 补丁文件
   │否
需要跟上游、需要 review/CI？ ──是──> C fork + submodule
   │否
D 平铺 vendor（并在 UPSTREAM.md 记录基线）
```

> 同一项目里可以混用，但**必须在 `docs/` 里写明每个依赖用哪种模式**，否则后来者无法判断。

### 2.1 fork 与 vendor（subtree）的差异

两者都是业界通行做法，区别在"历史如何保留"和"消费方如何取用"：

| 维度 | fork | vendor（`git subtree`） |
|---|---|---|
| 上游历史 | 完整保留（同一 DAG，可直接 merge/rebase） | 压缩为 1 个 squash 提交，落在子目录 |
| 跟上游同步 | `git merge upstream/<branch>` | `git subtree pull --prefix=...` |
| GitHub 关联 | 有 `forked from`，可向上游提 PR | 无关联 |
| 消费方接入 | submodule / 直接 clone | `deps.repos` 固定 commit，一次 clone 全有 |
| 仓库数量 | 每个依赖一个仓库 | 集中在一个 vendor 仓库 |
| 适合场景 | 改动多、长期跟上游、要向社区回馈 | monorepo、构建可复现、团队统一管理 |
| 许可义务 | 完全相同 | 完全相同 |

**哪一种更"规范"**：没有绝对答案，取决于团队既有约定。
开源社区偏 fork（PR/issue 流程天然）；大型 monorepo 偏 vendor（一次 clone、版本组合可控）。
**同一团队内应二选一**，混用会让依赖来源、升级方式、review 路径都出现两套。

---

## 3. 模式 C：fork + submodule（最常用）

### 3.1 建立带历史的本地仓库

不是 `git init`，而是从上游 clone 后切到基线：

```bash
git clone <upstream-url> ~/dep_work
cd ~/dep_work
git checkout -b main <baseline-commit>
git submodule update --init --recursive     # 若上游带子模块
```

### 3.2 应用本地改动，按主题拆提交

```bash
# 从现有工作目录（无 .git 的那份）逐文件覆盖
cp <work-dir>/<file> <file>
git status --short          # 必须与预期清单完全一致，出现删除项立即停下
```

拆提交的经验规则：

- 一个提交 = 一个可独立回退的行为改动（如"park 位姿"、"topic 后端"、"TF 前缀"各一条）；
- 提交信息用团队既有风格（如 Conventional Commits），正文写**为什么**改；
- 不混入格式化噪声、`.bak`、临时脚本。

### 3.3 发布到组织/个人账号

**带 `forked from` 关联（推荐）**

1. 网页打开上游 → **Fork** → *Create a new fork*；
2. Owner 选目标账号/组织，仓库名保持一致；
3. `Copy the main branch only` 保持勾选（只复制默认分支）即可；
4. 本地把 fork 设为 `origin`、上游设为 `upstream`：

```bash
git remote rename origin upstream
git remote add origin <fork-url>
git fetch upstream --tags
git push -u origin main
git tag <your-tag> && git push origin <your-tag>
```

5. 仓库 Settings → Branches 把默认分支改为 `main`；上游同名分支（如 `ros2`）保持原样，只用于 merge。

**无关联的等价做法**：在目标账号下建空仓库，`git remote add origin <空仓库>` 后 push，
git 内容完全一致，只少了 GitHub 的 `forked from` 标识。

限制：public 仓库的 fork 也是 public；需要私有就只能走空仓库方式（并确认许可证允许）。

### 3.4 消费方接入（submodule）

```bash
cd <consumer-repo>
git checkout -b feature/<issue-id>-<desc> origin/develop
git submodule add -b main <fork-url> src/<dep-name>
git commit -m "feat(dep): 引入 <dep-name>（<改动摘要>）"
```

必须同时处理：

- 克隆/CI 使用 `--recursive`；`actions/checkout` 加 `with: submodules: recursive`；
- 若依赖自身带子模块，记录"嵌套子模块"这一事实（`git submodule status --recursive` 可查）。

### 3.5 日常同步上游

```bash
cd src/<dep-name>
git fetch upstream
git checkout main && git merge upstream/<upstream-branch>
# 冲突只会落在你改过的那几个文件
git push origin main
cd ../.. && git add src/<dep-name> && git commit -m "chore(dep): bump <dep-name>"
```

升级策略二选一，团队内统一：

- **merge**：保留合并历史，冲突解决一次；
- **rebase**：历史线性，但每个本地提交都可能冲突，适合改动少的情况。

---

## 4. 模式 B：补丁文件

适合小改动、希望源码树与上游逐字一致的情况。

```bash
# 生成（有 fork 或本地基线仓库时）
git format-patch <baseline-commit>..main --output-directory patches/

# 应用（消费方引导脚本）
git -C external/<dep> apply --check patches/*.patch && \
git -C external/<dep> apply patches/*.patch
```

要求：

- `patches/` 里每个补丁一个逻辑改动，文件名带序号与摘要；
- 记录适用基线：`patches/README.md`（上游仓库 + commit + 生成日期）；
- 上游更新后先 `--check`，失败即说明需要重做补丁。

---

## 5. 模式 D：平铺 vendor

```text
src/<dep-name>/
├── UPSTREAM.md        # 必填：上游 URL、基线 commit、修改清单、同步方式
├── LICENSE            # 上游许可证原件，不得删除
├── NOTICE             # 如上游有
└── ...上游文件（含你的修改）
```

纪律：

- 只拷贝用到的包/目录，不整仓照搬；
- `UPSTREAM.md` 的修改清单要写出**文件 + 改动摘要**，与补丁/提交对应；
- 大二进制（模型、meshes）不随仓库分发，写下载/导出脚本；
- 明确告知团队：此模式下上游同步是手工行为。

---

## 6. 许可速查

| 上游许可 | 可直接复制 | 修改后分发要求 | 注意 |
|---|---|---|---|
| MIT | 是 | 保留版权与许可声明 | 最常见 |
| Apache-2.0 | 是 | 保留 LICENSE/NOTICE，**标注被修改的文件** | 与本项目许可兼容 |
| BSD-2/3 | 是 | 保留声明 | 与 MIT 类似 |
| GPL-2/3 | 谨慎 | 衍生作品整体受 GPL | 不要并入宽松许可仓库 |
| AGPL-3.0 | 谨慎 | 网络服务也算分发 | **独立仓库/独立进程**，勿与 MIT/Apache 混放 |
| 无许可证 / Proprietary | 否 | 不得再分发 | 只能内部使用 |

通用动作：

1. 依赖目录保留上游 `LICENSE`；若平铺，放在依赖子目录而不是仓库根；
2. 仓库根 `NOTICE` 列出：上游名称、URL、基线 commit、本地修改摘要、许可；
3. README 增加"第三方组件"一节，指向 `NOTICE`。

---

## 7. 版本与依赖固定

| 引用方式 | 固定手段 |
|---|---|
| submodule | gitlink（直接写 commit；`-b` 只是分支提示） |
| 直接引用 | `deps.repos` 写 `version: <commit/tag>`；可选 `deps.lock` 记录解析结果 |
| apt/pip | 版本号 + 锁文件 |
| vendor | `UPSTREAM.md` 记录基线 |

禁止：在构建脚本里 `git clone` 浮动分支；CI 每次拉最新。

---

## 8. CI 适配清单

引入第三方依赖时，CI 常见失败点：

- [ ] `actions/checkout` 是否带 `submodules: recursive`
- [ ] 根目录 `cmake ..` 类构建是否假设了不存在的 `CMakeLists.txt`
- [ ] 是否依赖 ROS/系统库但 runner 上没有 → 用容器 job 或路径过滤
- [ ] 全局格式化/lint（如 `black --check .`）是否会因第三方代码全红 → 限定路径
- [ ] 依赖专属 job 与主 job 是否隔离（避免一方失败拖垮全部）

---

## 9. 评审与 PR 规范

- 一个 PR 一个主题：引入依赖 / 升级依赖 / 修复依赖，不混业务改动；
- PR 描述必须包含：上游 URL、基线 commit、改动清单、验证方式；
- 依赖仓库自身的改动走它自己的分支与 PR，消费方只更新 gitlink；
- 分支命名遵循团队规范（如 `feature/<issue-id>-<desc>`），不直接推 `main`/`develop`。

---

## 10. 常见陷阱

| 陷阱 | 后果 | 预防 |
|---|---|---|
| 直接改上游副本、删掉 `.git` | 无法 diff、无法 merge（本项目已经踩过） | 先建带基线的仓库再改 |
| 忘记 `--recursive` | 子模块目录为空，构建静默少包 | clone/CI 全部加 `--recursive` |
| 认为 fork 可以设私有 | public 上游的 fork 也是 public | 需私有则建空仓库推送 |
| 强推上游同名分支 | 破坏 fork 关联与 PR 能力 | 上游分支只读，改动放在自己的分支 |
| 提交 `build/ install/ log/ bags/` | 仓库爆炸 | `.gitignore` 先行 |
| 混入 `.bak` / 临时脚本 | review 噪声 | 提交前 `git status` 逐项确认 |
| AGPL 依赖并入宽松仓库 | 许可证传染 | 独立仓库/独立进程 |
| 修改未标注 | 违反 Apache-2.0 | `NOTICE` + 文件头说明 |
| 用浮动分支引用 | 构建不可复现 | 钉 commit/tag |
| 只 vendor 改动的一部分 | 构建缺包、行为不一致 | 按包边界完整引入 |

---

## 11. 模板

### 11.1 `NOTICE`

```text
本仓库包含以下第三方组件：

<dep-name>
  来源: <upstream-url>
  基线: <baseline-commit>（<date>）
  许可: <license>
  本地修改: <一句话摘要>，明细见 <path/to/UPSTREAM.md 或 patches/README.md>
```

### 11.2 `UPSTREAM.md`（vendor 模式必填）

```markdown
# <dep-name> 上游信息

- 上游: <url>
- 基线 commit: <sha> (<date>)
- 引入日期: <date>
- 同步方式: 手工 diff（本项目未保留 git 历史）/ fork+merge

## 本地修改

| 文件 | 改动 | 对应提交/补丁 |
|---|---|---|
| ... | ... | ... |

## 升级步骤

1. 拉取上游对应 commit 到临时目录；
2. `diff -ru` 对比本文档列出的文件；
3. 更新本文件与 `NOTICE`。
```

### 11.3 提交信息

```text
<type>(<scope>): <一句行为描述>

为什么这样改（现场现象 / 失败模式 / 上游行为）。
基线: <upstream>@<commit>
```

### 11.4 `.gitignore` 增补（ROS 项目）

```gitignore
build/ install/ log/
bags/ *.bag *.bag.zstd
.run/
frames_*.gv frames_*.pdf
__pycache__/ *.pyc
*.bak *.bak_* *.orig
```

---

## 12. 验收清单

- [ ] 依赖有明确基线 commit，`git diff <baseline>..main` 与预期改动一致
- [ ] 改动按主题拆成多个提交，信息含"为什么"
- [ ] `LICENSE`/`NOTICE` 保留并更新
- [ ] 消费方以 commit/tag 钉死，克隆/CI 带 `--recursive`
- [ ] 干净目录 `git clone --recursive` 后能构建
- [ ] 上游同步流程实际演练过一次（fetch/merge → 更新 gitlink）
- [ ] 文档记录：用了哪种模式、为什么、怎么升级
