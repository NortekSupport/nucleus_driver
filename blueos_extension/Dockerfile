FROM python:3.9-slim-bullseye

RUN apt-get update && apt-get -y upgrade

RUN useradd --create-home pi
USER pi

ENV PATH="/home/pi/.local/bin:${PATH}"

RUN pip install --upgrade pip

COPY requirements.txt /tmp/
RUN pip install --user -r /tmp/requirements.txt

COPY --chown=pi:pi . /home/pi/blueos_extension

WORKDIR /home/pi/blueos_extension

EXPOSE 80/tcp

LABEL version="1.0.1"
LABEL permissions='\
{\
  "ExposedPorts": {\
    "80/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/root/.config:/root/.config"],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "80/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors='[\
    {\
        "name": "Martin Bergene Johansen",\
        "email": "martin.johansen@nortekgroup.com"\
    }\
]'
LABEL company='{\
        "about": "",\
        "name": "Nortek",\
        "email": ""\
    }'
LABEL type="device-integration"
LABEL tags='[\
    "positioning",\
    "navigation",\
    "doppler-velocity-log",\
    "water-tracking",\
    "current-profiling"\
]'
LABEL readme='https://raw.githubusercontent.com/nortekgroup/nucleus_driver/main/blueos_extension/README.md'
LABEL links='{\
        "website": "https://github.com/nortekgroup/nucleus_driver/tree/main/blueos_extension",\
        "support": "https://support.nortekgroup.com/"\
    }'

ARG PORT=5000

ENV FLASK_APP=src/nucleus_extension/flask_app.py FLASK_DEBUG=1 PYTHONUNBUFFERED=1
ENV FLASK_RUN_PORT=${PORT}

RUN echo ${PORT}

CMD ["flask", "run", "--host=0.0.0.0", "--no-reload"]
