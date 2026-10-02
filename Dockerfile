FROM python:3.12-slim AS builder
WORKDIR /app
COPY requirements.txt /app/requirements.txt
COPY src /app/src
RUN pip install --no-cache-dir -r requirements.txt \
    && python -m compileall -b -q /app/src \
    && rm -f /app/src/*.py

FROM python:3.12-slim
WORKDIR /app
RUN groupadd --gid 10001 appgroup \
    && useradd --uid 10001 --gid 10001 --no-create-home --shell /usr/sbin/nologin appuser
COPY --from=builder /app/src /app/src
USER 10001
ENTRYPOINT ["python", "src/main.pyc"]
