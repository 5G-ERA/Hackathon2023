FROM python:3.8.16-slim 

RUN apt-get update \
    && apt install -y libgl1-mesa-glx libglib2.0-0

COPY era_5g_network_application_obstacle_avoidance_laser /root/era_5g_network_application_obstacle_avoidance_laser

RUN cd /root/era_5g_network_application_obstacle_avoidance_laser \
    && pip install -r requirement.txt

EXPOSE 5896

CMD ["/root/era_5g_network_application_obstacle_avoidance_laser/era_5g_network_application_obstacle_avoidance_laser/interface.py"]

ENTRYPOINT [ "python" ]
