# Virtual-robot: MQTT stand-in for the physical ESP32 arm (hardware endpoint).
# Self-contained (does not depend on a prebuilt base) so it builds standalone
# in CI. Build context = project root:  docker build -f deploy/Dockerfile.robot .
FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY scripts/ ./scripts/
COPY urdf/ ./urdf/

ENV MPLBACKEND=Agg
ENV PYTHONUNBUFFERED=1
WORKDIR /app/scripts

CMD ["python", "virtual_robot.py"]
