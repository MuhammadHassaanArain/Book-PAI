# Use ROS 2 Humble Hawksbill with Ubuntu 22.04
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js and npm
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Copy package.json if it exists (for dependency installation)
COPY package.json . 2>/dev/null || echo "{}" > package.json

# Install npm dependencies
RUN npm install

# Install Python dependencies if requirements.txt exists
COPY requirements.txt . 2>/dev/null || echo "" > requirements.txt
RUN pip3 install -r requirements.txt 2>/dev/null || echo "No requirements.txt found, skipping Python dependency installation"

# Source ROS environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Set up ROS environment in the container
ENV ROS_LOCAL_SHARE=/opt/ros/$ROS_DISTRO/share
ENV AMENT_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV COLCON_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.10/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Create a non-root user for security
RUN useradd -m -s /bin/bash -u 1000 user
USER user
WORKDIR /home/user/workspace

# Copy the project files
COPY --chown=user:user . .

# Source ROS setup in user environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Expose port for Docusaurus development server
EXPOSE 3000

CMD ["/bin/bash"]