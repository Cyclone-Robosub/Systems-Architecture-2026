ARG DOCKER_REPO
ARG ROS_DISTRO
ARG IMAGE_SUFFIX
FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX

ARG USERNAME
ARG USER_ID=1000
ARG GROUP_ID=${USER_ID}

COPY packages.txt .

# Install packages from the file
RUN apt-get update && \
    apt-get upgrade -y && \
    xargs -a packages.txt apt-get install -y && \
    rm -rf /var/lib/apt/lists/*

RUN rm packages.txt

# Create workspace and set permissions for existing user (UID 1000)
RUN mkdir -p /home/ws && \
    chown -R ${USER_ID}:${GROUP_ID} /home/ws

ENV SHELL=/bin/bash

# The ROS base image likely already has a user with UID 1000
# We'll just switch to that user
USER ${USER_ID}

CMD ["/bin/bash"]