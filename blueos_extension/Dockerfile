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

LABEL version="0.0.1"
LABEL permissions '\
{\
  "ExposedPorts": {\
    "80/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/root/.config:/root/.config"],\
    "PortBindings": {\
      "80/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors '[\
    {\
        "name": "Martin Bergene Johansen",\
        "email": "martin.johansen@nortekgroup.com"\
    }\
]'
LABEL docs ''
LABEL company '{\
        "about": "",\
        "name": "Nortek",\
        "email": ""\
    }'
LABEL readme 'https://github.com/nortekgroup/nucleus_driver/blueos_extension/README.md'
LABEL website 'https://github.com/nortekgroup/nucleus_driver/blueos_extension'
LABEL support ''

ENV FLASK_APP=src/nucleus_extension/flask_app.py FLASK_DEBUG=1 PYTHONUNBUFFERED=1
#CMD ["flask", "run", "--host=0.0.0.0", "--port=80", "--no-reload"]  # TODO: __port should be 80 when integrating into blueos
CMD ["flask", "run", "--host=0.0.0.0", "--port=5000", "--no-reload"]