# Wiki 仓库 Git 操作手册

> 适用仓库：`git@github.com:ManifoldTechLtd/wiki.git`
> 本地路径：`/home/hugo/git_wiki/wiki`
> GitHub Pages 部署：自动构建 `master` 分支的 `docs/` 目录 → https://manifoldtechltd.github.io/wiki/

---

## 1. 仓库分支模型

| 分支 | 用途 | 是否触发 GitHub Pages 构建 |
|---|---|---|
| `master` | 线上发布分支 | ✅ 触发，push 后 1–3 分钟生效 |
| `odin_develop` | 开发分支，所有日常改动先在这上面 commit | ❌ 不触发 |

**规则：永远不要直接在 `master` 上 commit**。
所有改动 → 先在 `odin_develop` commit → push → 再合并到 `master` → push。

---

## 2. 中英文双语维护约定

仓库下中英文是两份独立 markdown：

```
docs/odin_series/odin1/X.md         ← 中文原文（主）
docs/en/odin_series/odin1/X.md      ← 英文翻译（副）
```

**改一处必改对应另一处**（除非只改 typo / 仅改某一种语言的术语）。
图片 / PDF / 代码资源只放在中文目录下的 `assets/`，英文页通过 `{{ '...' | relative_url }}` 引用，**不要复制资源到 `docs/en/assets/`**。

---

## 3. 标准发布流程（最常用，6 步）

每次改完文档（不管改了几个文件），按这 6 步走：

### 第 1 步：确认当前在 `odin_develop` 分支

```bash
cd /home/hugo/git_wiki/wiki
git status
```

输出第一行应该是 `On branch odin_develop`。如果不是：

```bash
git checkout odin_develop
```

### 第 2 步：查看改了哪些文件

```bash
git status              # 列出修改 / 新增 / 删除的文件
git diff                # 查看具体行级改动
git diff --stat         # 只看每个文件改了几行
```

### 第 3 步：把所有改动添加到暂存区并 commit

```bash
git add -A              # -A 表示所有改动（含新增、删除）
git commit -m "docs(odin1): 一句话描述这次改了什么"
```

**commit message 建议格式**（方便你以后回看）：

```
docs(<模块>): <一句话总结>

- <文件 1>: <改了什么>
- <文件 2>: <改了什么>
- 中英已同步 / 仅中文 / 仅英文
```

实例：

```
docs(odin1): IP66 spec correction, FAQ Q2.7 USB cleanup (zh+en)

- 14. Technical Specifications: IP67 -> IP66
- 15. FAQ: add Q2.7 'LIBUSB_ERROR_BUSY' with SIGINT cleanup script
- 中英已同步
```

### 第 4 步：把 `odin_develop` 推到远端

```bash
git push origin odin_develop
```

此时改动**还没上线**，因为 GitHub Pages 只看 `master`。

### 第 5 步：合并到 `master` 并推送

```bash
git checkout master
git merge --no-ff odin_develop -m "Merge: <一句话>"
git push origin master
```

⚠️ **必须用 `--no-ff`**（创建 merge commit）。
原因：仓库历史里 `master` 已经有大量 merge commit，普通 `git merge`（默认 fast-forward）会失败报 `Not possible to fast-forward`。`--no-ff` 永远成功，且历史更清晰。

### 第 6 步：切回 `odin_develop` 继续工作

```bash
git checkout odin_develop
```

---

### 一键脚本（可选）

把上面 4–6 步合成一行，每次改完文档执行即可：

```bash
git add -A && \
git commit -m "docs: <一句话>" && \
git push origin odin_develop && \
git checkout master && \
git merge --no-ff odin_develop -m "Merge: <一句话>" && \
git push origin master && \
git checkout odin_develop
```

把两处 `<一句话>` 替换成实际描述就行。

---

## 4. 验证发布是否成功

push `master` 后 1–3 分钟，访问下面任一链接刷新（**Ctrl+F5 强刷新**避开浏览器缓存）：

- 🇨🇳 中文站：https://manifoldtechltd.github.io/wiki/
- 🇬🇧 英文站：https://manifoldtechltd.github.io/wiki/en/

也可以在 GitHub 仓库页 → **Actions** 标签查看 `pages-build-deployment` 是否成功（绿勾✅ = 已上线）。

---

## 5. 常见情况和恢复操作

### 5.1 改错了，还没 commit，想撤销

```bash
git status                           # 看哪些文件被改了
git restore <文件路径>               # 撤销单个文件
git restore .                        # 撤销当前目录所有改动（慎用！）
```

### 5.2 已经 commit，但还没 push，想撤销最后一次 commit

```bash
git reset --soft HEAD~1              # 撤销 commit，改动保留在暂存区
# 或
git reset HEAD~1                     # 撤销 commit，改动保留在工作区（更常用）
```

### 5.3 已经 push 到 odin_develop，但还没合并到 master，想撤销

修改后再 commit + push 一次即可（push 上去的改动可以被新 commit 覆盖）。
不要用 `git push -f`，除非确认没人在用这个分支。

### 5.4 已经 push 到 master 了，发现内容不对

**最安全的做法**：重新改一遍 → 走完整 6 步流程发新 commit 修复。
**不要**用 `git reset` + `git push -f master`，会破坏历史。

### 5.5 `git merge --no-ff odin_develop` 报冲突

理论上不会，因为 `master` 永远是 `odin_develop` 的子集（你只往 odin_develop 上 commit）。
如果真发生，多半是 `master` 上意外有了别的提交。这时候：

```bash
git status                           # 看冲突文件
# 手动编辑冲突文件，删掉 <<<<<<< / ======= / >>>>>>> 标记
git add <冲突文件>
git commit                           # 完成合并
git push origin master
```

冲突复杂搞不定就找我帮忙。

---

## 6. 不要做的事 ❌

- ❌ 直接在 `master` 上 commit
- ❌ `git push -f`（强制推送，会覆盖远端历史）
- ❌ `git reset --hard` 后立刻 `git push -f`
- ❌ 把改动同时 commit 在 `master` 和 `odin_develop` 上（会双倍 commit）
- ❌ 提交时漏掉中英文同步（除非这次故意只改一边）

---

## 7. 速查表

| 想做什么 | 命令 |
|---|---|
| 看当前改了什么 | `git status` |
| 看具体行级改动 | `git diff` |
| 看简要改动统计 | `git diff --stat` |
| 看历史提交 | `git log --oneline -20` |
| 看某个文件历史 | `git log --oneline -- "<文件路径>"` |
| 切换分支 | `git checkout <分支名>` |
| 当前是哪个分支 | `git branch --show-current` |
| 取消未 commit 的改动 | `git restore <文件>` |
| 撤销最后一次 commit | `git reset HEAD~1` |
| 拉取远端最新 | `git pull origin <分支>` |

---

## 8. 完整示例：改一个 typo 的全过程

假设要把 `15. FAQ.md` 里 "请鞋按 SIGINT" 改成 "请按 SIGINT"：

```bash
# 1. 进入仓库
cd /home/hugo/git_wiki/wiki

# 2. 确认在 odin_develop 上
git status
# On branch odin_develop ✅

# 3. 改文件（用 VS Code / Qoder 编辑保存）
# 改 docs/odin_series/odin1/15. FAQ.md
# 改 docs/en/odin_series/odin1/15. FAQ.md（如果英文版也错了）

# 4. 看改动
git diff

# 5. 提交并发布
git add -A
git commit -m "docs(odin1): fix typo in FAQ Q2.7 (zh+en)"
git push origin odin_develop
git checkout master
git merge --no-ff odin_develop -m "Merge: typo fix in FAQ Q2.7"
git push origin master
git checkout odin_develop

# 6. 1-3 分钟后访问 https://manifoldtechltd.github.io/wiki/ 验证（Ctrl+F5）
```

---

如果遇到任何 git 报错搞不定，把 `git status` 的完整输出发给我即可。
