FROM ros:kinetic-ros-base

RUN apt-get update 
RUN apt-get install -y sudo python-yaml python-sqlalchemy python-psycopg2

ADD ./drone_master_messenger /tmp/build/src/drone_master_messenger 
RUN . /opt/ros/kinetic/setup.sh \
    && cd /tmp/build/src \
    && catkin_init_workspace \
    && cd .. \
    && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y \
    && catkin_make

ADD ./docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
