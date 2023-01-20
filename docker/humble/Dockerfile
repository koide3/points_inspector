FROM ros:humble

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    libboost-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/ros2_ws/src/points_inspector

WORKDIR /root/ros2_ws

RUN /bin/bash -c ". /opt/ros/humble/setup.bash; colcon build"

RUN echo "#!/bin/bash" > /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/humble/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
