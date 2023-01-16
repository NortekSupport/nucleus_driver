FROM python:3.9-slim-bullseye

RUN apt-get update && apt-get -y upgrade

RUN useradd --create-home pi
USER pi

ENV PATH="/home/pi/.local/bin:${PATH}"

RUN pip install --upgrade pip

COPY requirements.txt /tmp/
RUN pip install --user -r /tmp/requirements.txt
RUN pip install --user Flask==2.2.2
RUN pip install --user Flask-RESTful==0.3.9
RUN pip install --user requests==2.28.2



#RUN mkdir -p /home/pi

COPY --chown=pi:pi . /home/pi/nucleus_driver

WORKDIR /home/pi/nucleus_driver

RUN pip install --user .

#RUN pip install /home/pi/nucleus_driver
#RUN cd /home/pi/nucleus_driver && pip3 install .

#WORKDIR /home/pi/nucleus_driver  # not necessary?

#CMD ["python3.9", "./src/nucleus_driver/entrypoints/flask_app.py"]

ENV FLASK_APP=src/nucleus_driver/entrypoints/flask_app.py FLASK_DEBUG=1 PYTHONUNBUFFERED=1
CMD ["flask", "run", "--host=0.0.0.0", "--port=5000"]
