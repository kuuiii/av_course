-- How to Run build the docker:

$ docker build -t ros2-turtlebot3:v1.0 .


===================================
-- How to run the docker:

$ docker run -it --rm --privileged --net=host  --env=DISPLAY  --env=QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix  -v /home/autolab/ros2_ws:/home/student/ros2_ws ros2-turtlebot3:v1.0

** -v /home/autolab/ros2_ws:/home/student/ros2_ws this part of the command will creates a link with your drecotry in the host machine and mirror it into the docker.
============================================
-- How to run another terminal:
First run "docker ps" to find out what is the docker_name running, it will shows you similar to this(if not it means that you dont have runnign docker):

CONTAINER ID   IMAGE                  COMMAND                  CREATED          STATUS          PORTS     NAMES
9030c9baba6a   ros2-turtlebot3:v1.0   "/ros_entrypoint.sh …"   40 minutes ago   Up 40 minutes             quizzical_hugle

The name of the docker is now "quizzical_hugle". Check what is the name in your case. Then use it in the lower command to open another terminal.

$ docker exec -it <Your_Running_Docker_Name> bash
