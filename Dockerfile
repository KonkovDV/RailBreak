FROM osrf/ros:humble-desktop
LABEL org.opencontainers.image.title="tramDR"
LABEL org.opencontainers.image.source="https://github.com/KonkovDV/RailBreak"
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY tram_dr_localization /ws/src/tram_dr_localization
# Compose bind-mounts this path. Do not mount the whole repo over /ws.
