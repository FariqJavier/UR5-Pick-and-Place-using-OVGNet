# ROS Docker Setup Example

## Install Dependencies:

* Docker: https://docs.docker.com/engine/install/ubuntu/
* docker-compose-plugin: https://docs.docker.com/compose/install/linux/

## First Time Instructions

1. On the host system, setup a ROS workspace, a data directory for moving ROS
   bags, and clone this repo:

        mkdir -p ~/ros/ros_docker_example/workspace/src
        mkdir -p ~/ros/ros_docker_example/data
        cd ~/ros/ros_docker_example
        git clone https://github.com/SyllogismRXS/ros_docker_setup.git

2. Clone ROS example repositories:

        cd ~/ros/ros_docker_example/workspace/src
        git clone -b humble https://github.com/ros2/examples.git

3. Build the docker image

        cd ~/ros/ros_docker_example/ros_docker_setup
        echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env
        docker compose build

## Run the Example

1. Run the minimal subscriber example

        docker compose up -d dev
        docker exec -it ros_humble /bin/bash
        ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function

2. In a different terminal, run the minimal publisher example:

        docker exec -it ros_humble /bin/bash
        ros2 run examples_rclcpp_minimal_publisher publisher_member_function

## Stopping Containers

Back on the host system: if you make changes to the container (e.g., installing
additional packages manually) and you want to persist those changes, you will
just want to "stop" the container:

    docker compose stop

If you want to wipe out changes to the container, use the "down"
command:

    docker compose down
