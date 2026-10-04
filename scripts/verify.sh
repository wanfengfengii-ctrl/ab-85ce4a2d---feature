#!/bin/sh
# verify 一次性服务入口：在已构建镜像内、且 app 依赖健康后执行。
# 三个阶段任一失败立即以非零退出码报告。
set -u

set -u
PY=${PYTHON_BIN:-python}

echo "== [1/3] 代码测试（unittest 全量） =="
$PY -m unittest discover -s tests -v || {
  echo "!! 代码测试失败"; exit 1; }

echo
echo "== [2/3] 镜像构建检查 =="
# verify 容器本身只能由成功构建的镜像启动（构建失败时 compose 不会运行本
# 脚本）；这里再核对镜像内交付物完整、可编译、可导入。
# 工作目录即镜像构建上下文落点（容器内 /srv）；相对路径同时便于本地演练。
if [ ! -f app/server.py ] || [ ! -f app/solver.py ]; then
  echo "!! 镜像内应用交付物缺失"; exit 1
fi
$PY -m compileall -q app tests || {
  echo "!! 镜像内代码字节码编译失败"; exit 1; }
$PY -c "import app.server, app.solver; print('镜像交付物导入正常')" || {
  echo "!! 镜像内应用导入失败"; exit 1; }
echo "镜像构建检查通过（$($PY --version 2>&1)）"

echo
echo "== [3/3] 联合拾取 API 冒烟（$BASE_URL） =="
$PY tests/smoke_api.py || {
  echo "!! API 冒烟失败"; exit 1; }

echo
echo "########## verify: 全部阶段通过 ##########"
