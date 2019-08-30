FROM ravestate-ros2-image

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# install ravestate dependencies
ADD requirements2.txt /tmp/requirements2.txt
RUN pip install -r /tmp/requirements2.txt

# install speech recognition requirements
ADD requirements-stt.txt /tmp/stt_requirements.txt
RUN pip3 install -r /tmp/stt_requirements.txt
RUN apt install -y python3-pyaudio
