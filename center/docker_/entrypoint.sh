#!/bin/bash
set -e
set -x

echo "**** Entrypoint script starting ****"
# Set default values for environment variables
: "${CHECK_ROS2_WS_DEPENDENCIES:=true}"
: "${BUILD_ROS2_WS:=true}"

##############################################################################
# 1) Conditionally build /ros2_ws
##############################################################################
if [ -d "/ws/ros2_src" ]; then
    echo "Setting up ROS 2 workspace..."
    mkdir /ros2_ws
    ln -s /ws/ros2_src /ros2_ws/src

    if [ "${CHECK_ROS2_WS_DEPENDENCIES}" = "true" ]; then
        echo "Checking for missing dependencies..."
        source /opt/ros_venv/bin/activate
        source /opt/ros/jazzy/setup.bash
        rosdep update --rosdistro ${ROS_DISTRO}
        output=$(rosdep install --from-paths /ros2_ws/src --ignore-src --simulate 2>&1)
        if [ -z "$output" ]; then
            echo "All dependencies are satisfied."
        else
            echo "Dependencies are missing. Add these installations to the Dockerfile:"
            echo "$output"
            exit 1
        fi
    else
        echo "Skipping dependency check."
    fi

    if [ "${BUILD_ROS2_WS}" = "true" ]; then
        echo "Building ROS 2 workspace..."
        pushd /ros2_ws
        colcon build
        popd
    else
        echo "Skipping building ROS 2 workspace."
    fi

    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc
else
    echo "No ROS 2 workspace found at /ws/ros2_src. Skipping setup of ROS 2 workspace."
fi

##############################################################################
# 2) Dynamically create non-root user if user variables are defined
##############################################################################
# Motivation: When creating files on bind-mounted volumes (e.g., catkin_ws),
# the files are owned by root if no non-root user is used.
if [ -z "$USER_UID" ] || [ -z "$USER_GID" ]; then
    echo "NOTE: \$USER_UID and/or \$USER_GID not set. Skipping user creation."
    echo "Running as root."
    exec "$@"
else
    if id "$USER_UID" &>/dev/null; then
        echo "ERROR: User with UID $USER_UID already exists."
        exit 1
    fi
    : "${USERNAME:=myuser}"
    echo "Creating user '$USERNAME' (UID=$USER_UID, GID=$USER_GID) ..."
    if ! getent group "$USER_GID" >/dev/null 2>&1; then
        groupadd --gid "$USER_GID" "$USERNAME"
    fi
    useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" -s /bin/bash
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME"
    chmod 0440 "/etc/sudoers.d/$USERNAME"
    sudo chown -R $USERNAME:$USERNAME /home/$USERNAME

    ### User Setup ###
    # Copy .bashrc
    cp /root/.bashrc "/home/$USERNAME/.bashrc"
    chown "$USER_UID:$USER_GID" "/home/$USERNAME/.bashrc"
    echo "Copied /root/.bashrc to /home/$USERNAME/.bashrc."
    # claim ownership of /ros2_ws (if it exists)
    [ -d "/ros2_ws" ] && sudo chown -R $USERNAME:$USERNAME /ros2_ws/

    echo "Switching to user '$USERNAME' (UID=$USER_UID, GID=$USER_GID)."
    exec gosu "$USERNAME" "$@"
fi
