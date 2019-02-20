FROM duckietown/rpi-duckiebot-base:master18

COPY run_traffic_lights.sh .

CMD [ "./run_traffic_lights.sh" ]
