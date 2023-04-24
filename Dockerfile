FROM python:3.8.16-slim 

RUN apt-get update \
    && apt install -y libgl1-mesa-glx libglib2.0-0

COPY era_5g_network_application_template /root/era_5g_network_application_template

RUN cd /root/era_5g_network_application_template \
    && pip install -r requirement.txt

EXPOSE 5896

CMD ["/root/era_5g_network_application_template/era_5g_network_application_template/interface.py"]

ENTRYPOINT [ "python" ]