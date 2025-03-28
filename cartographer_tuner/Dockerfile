FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        python3-pip \
        curl

RUN curl -sSL https://install.python-poetry.org | python3 -
ENV PATH="/root/.local/bin:$PATH"

WORKDIR /app

COPY pyproject.toml poetry.lock ./
RUN poetry install --no-root

COPY . /app/

CMD ["poetry", "run", "pytest"]