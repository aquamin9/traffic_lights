FROM duckietown/rpi-duckiebot-base:master18

#RUN [ "cross-build-start" ]

COPY run_traffic_lights.sh .
COPY docker/traffic_light_node.py /home/software/catkin_ws/src/40-coordination/traffic_light/src
#COPY docker/TrafficMessage.msg /home/software/catkin_ws/src/00-infrastructure/duckietown_msgs/

# Setup the duckietown_msgs for acquisition node
# DO MANUALLY RUNS IN CONTAINER
COPY duckietown_msgs /home/software/catkin_ws/src/00-infrastructure/duckietown_msgs
#RUN mkdir -p /catkin_ws/src
#RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/software/catkin_ws/; catkin_make duckietown_msgs "
#RUN /bin/bash -c "source /catkin-ws/devel/setup.bash; cd /catkin-ws/; catkin_make"
#RUN [ "cross-build-end" ]


CMD [ "./run_traffic_lights.sh" ]
