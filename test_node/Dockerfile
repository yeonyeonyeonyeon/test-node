# Use an official ROS Noetic base image
FROM ros:noetic

# Set the working directory in the docker image
WORKDIR /root

# Create catkin workspace directory
RUN mkdir -p /root/catkin_ws/src

# Change the working directory
WORKDIR /root/catkin_ws/src/test_node

# Copy the current directory contents into the container at /root/catkin_ws/src
ADD ../src /root/catkin_ws/src
ADD ./install.sh /root/catkin_ws/src/test_node/install.sh
COPY ../src /root/catkin_ws/src
COPY ./install.sh .
COPY ./launch/test.launch ./launch/

# Install any needed packages specified in requirements.txt
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if it has not been done before
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# Update rosdep
RUN rosdep update

# Check if the workspace is empty, and if so initialize it
RUN /bin/bash -c 'if [ ! "$(ls -A /root/catkin_ws/src)" ]; then source /opt/ros/noetic/setup.bash && cd /root/catkin_ws/src && catkin_init_workspace; fi'

# Build the catkin workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make'

# Source the catkin workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Modify the install.sh script
RUN sed -i 's|/home/kyc/my/catkin_ws|/root/catkin_ws|' /root/catkin_ws/src/test_node/install.sh


# Make the install script executable
RUN chmod +x /root/catkin_ws/src/test_node/install.sh

RUN cat /root/catkin_ws/src/test_node/install.sh

# Run the install script
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash"

# Set ROS environment variables
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# Set the entrypoint to run the install.sh script at container startup
ENTRYPOINT ["/bin/bash", "/root/catkin_ws/src/test_node/install.sh"]

# Set the working directory to the package's root directory
WORKDIR /root/catkin_ws/src/test_node

# Run the launch file when the docker container is run
CMD ["roslaunch", "test_node", "test.launch"]