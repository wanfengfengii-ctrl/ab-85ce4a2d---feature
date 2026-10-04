# 浅地层雷达双界面联合追踪服务
# 纯 Python 标准库实现，无第三方依赖，无需联网安装。
FROM python:3.11-slim

# 不缓冲输出，便于 docker compose logs / verify 日志即时可见。
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    API_PORT=8000

WORKDIR /srv

# 拷贝应用、测试与一次性校验脚本（同一镜像复用于常驻服务与 verify）。
COPY app/ ./app/
COPY tests/ ./tests/
COPY scripts/ ./scripts/

EXPOSE 8000

# 容器级健康检查（Compose 里另配了同样基于 /health 的 healthcheck）。
HEALTHCHECK --interval=5s --timeout=3s --start-period=3s --retries=5 \
    CMD python -m app.healthcheck || exit 1

CMD ["python", "-m", "app.server"]
