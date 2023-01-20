FROM ros:noetic

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    libboost-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY . /root/catkin_ws/src/points_inspector

WORKDIR /root/catkin_ws

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_make"

RUN echo "#!/bin/bash" > /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/noetic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
