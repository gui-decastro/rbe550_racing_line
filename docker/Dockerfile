# ROS Noetic base images is 3.52 GB
FROM osrf/ros:noetic-desktop-full
ARG ROS_DISTRO=noetic

COPY requirements.txt .

# Install python dependencies
RUN apt-get update && \
    apt-get install -y locales lsb-release python3-pip && \
    pip3 install --no-cache-dir -r requirements.txt && \
    apt-get clean

# Install ros packages
RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-dwa-local-planner

ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Set the DISPLAY environment variable (without it, graphics cannot be displayed inside docker)
ENV DISPLAY=:0

# Create and set the working directory
WORKDIR /catkin_ws

# Copy the ROS workspace into the container
COPY . /catkin_ws

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                 cd /catkin_ws && \
                 echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
                 echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc"

# Install ROS packages
RUN apt-get update && \
    apt-get install -y ros-noetic-map-server

# Start ROS Core by default
# ENTRYPOINT ["roscore"]
CMD /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && roscore & \
                 sleep 5 && \
                 /bin/bash"

# Default command (can be overridden at runtime)
# CMD ["/bin/bash"]