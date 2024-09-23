FROM ros:rolling

ARG TARGETPLATFORM

ARG ZENOH_VER=1.0.0-beta.3

# apt update & upgrade
RUN apt-get update && apt-get upgrade -y

# Install Zenoh & ros2dds plugin
RUN apt-get update && apt-get install -y \
  unzip wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN if [ ${TARGETPLATFORM} = "linux/amd64" ]; then \
  ( wget https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_VER}/zenoh-${ZENOH_VER}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-pkgs.zip \
	&& wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VER}/zenoh-plugin-ros2dds-${ZENOH_VER}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-ros2dds-pkgs.zip \
	&& unzip /tmp/zenoh-pkgs.zip -d /tmp \
	&& unzip /tmp/zenoh-ros2dds-pkgs.zip -d /tmp \
	&& apt-get install /tmp/zenoh*.deb \
	&& rm -f /tmp/* \
	) ; \
	fi;
RUN if [ ${TARGETPLATFORM} = "linux/arm64" ]; then \
  ( wget https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_VER}/zenoh-${ZENOH_VER}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-pkgs.zip \
	&& wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VER}/zenoh-plugin-ros2dds-${ZENOH_VER}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-ros2dds-pkgs.zip \
	&& unzip /tmp/zenoh-pkgs.zip -d /tmp \
	&& unzip /tmp/zenoh-ros2dds-pkgs.zip -d /tmp \
	&& apt-get install /tmp/zenoh*.deb \
	&& rm -f /tmp/* \
	) ; \
	fi;

# Install CycloneDDS, demo_nodes_cpp and turtlesim pkgs
RUN apt-get update && apt-get install -y \
	ros-rolling-rmw-cyclonedds-cpp ros-rolling-demo-nodes-cpp ros-rolling-turtlesim \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Build rmw_zenoh
RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN apt-get update \
	&& mkdir ~/ws_rmw_zenoh/src -p && cd ~/ws_rmw_zenoh/src \
	&& git clone https://github.com/ros2/rmw_zenoh.git \
	&& cd ~/ws_rmw_zenoh \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src --rosdistro rolling -y \
	&& source /opt/ros/rolling/setup.bash \
	&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

CMD ["/bin/bash"]
