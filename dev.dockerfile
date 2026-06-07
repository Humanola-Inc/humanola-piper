FROM python:3.11-trixie AS base

# Install Mamba
WORKDIR /opt
RUN wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh -O /tmp/miniforge.sh && \
  bash /tmp/miniforge.sh -b -p /opt/conda && \
  rm /tmp/miniforge.sh && \
  /opt/conda/bin/mamba install python=3.11 -y && \
  /opt/conda/bin/mamba clean -ya

WORKDIR /opt
COPY ./requirements.txt /opt/requirements.txt
RUN pip install -r requirements.txt

WORKDIR /app
RUN /opt/conda/bin/mamba install pinocchio

FROM python:3.11-slim-trixie AS final
COPY --from=base /opt/conda/lib/. /usr/local/lib/.
COPY --from=base /opt/conda/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=base /usr/local/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=base /usr/local/bin/. /usr/local/bin/