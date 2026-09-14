FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive \
    MUJOCO_GL=egl \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        git \
        libegl1 \
        libgl1 \
        libglfw3 \
        libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY pyproject.toml README.md LICENSE ./
COPY epuck_mujoco ./epuck_mujoco
RUN python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir -e ".[dev]"

COPY examples ./examples
COPY tests ./tests

CMD ["python", "-m", "pytest", "-q"]
