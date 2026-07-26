FROM python:3.11-trixie AS base

# Install mamba
RUN apt update && apt install -y \
  wget \
  build-essential \
  zstd
RUN wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh -O /tmp/miniforge.sh && \
  bash /tmp/miniforge.sh -b -p /opt/conda && \
  rm /tmp/miniforge.sh && \
  /opt/conda/bin/mamba install python=3.11 -y && \
  /opt/conda/bin/mamba clean -ya


# Install pinocchio
RUN /opt/conda/bin/mamba install pinocchio -y;

WORKDIR /opt
COPY ./requirements.txt /opt/requirements.txt
RUN pip install -r requirements.txt


FROM python:3.11-slim-trixie AS final

RUN apt update && apt install -y \
  ffmpeg \
  libsm6 \
  libxext6 \
  libstdc++6 \
  can-utils \
  ethtool \
  iproute2 \
  kmod \
  build-essential

ENV PYTHONPATH="/app"
ENV LD_LIBRARY_PATH="/usr/local/lib"


COPY --from=base /opt/conda/lib/. /usr/local/lib/.
COPY --from=base /opt/conda/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=base /usr/local/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=base /usr/local/bin/. /usr/local/bin/

WORKDIR /app
COPY ./URDF /app/URDF
COPY ./piper.urdf /app/URDF
COPY ./src/. /app

ENTRYPOINT [ "python3", "main_ez.py" ]