ARG BASE_IMAGE=ghcr.io/skymule-ai/px4-sitl:v1.17.0
FROM ${BASE_IMAGE}

USER root
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --ignore-installed --only-binary=:all: \
    numpy==1.26.4 scipy==1.13.1 numba==0.61.2 scikit-learn==1.6.1
COPY --from=mullet_interfaces / /opt/mfgp_ros/src/mullet_interfaces/
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && cd /opt/mfgp_ros && colcon build --packages-select mullet_interfaces'
WORKDIR /opt/mfgp
COPY src/ /opt/mfgp/src/
COPY docker/ /opt/mfgp/docker/
ENV PYTHONPATH=/opt/mfgp PYTHONUNBUFFERED=1 MPLBACKEND=Agg \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_NUM_THREADS=1
ENTRYPOINT ["/bin/bash", "/opt/mfgp/docker/entrypoint.sh"]
CMD ["docker.deployment_smoke"]
