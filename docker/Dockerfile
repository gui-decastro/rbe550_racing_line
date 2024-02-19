# ROS Noetic base images is 3.52 GB
FROM osrf/ros:noetic-desktop-full

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Create and set the working directory
WORKDIR /catkin_ws

# Copy the ROS workspace into the container
COPY . /catkin_ws

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                 cd /catkin_ws && \
                 echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
                 echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc"

# Start ROS Core by default
# ENTRYPOINT ["roscore"]
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && roscore & \
                 sleep 5 && \
                 /bin/bash"

# Default command (can be overridden at runtime)
# CMD ["/bin/bash"]