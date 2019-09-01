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

RUN cd ~/melodic_ws/src/pyroboy && git checkout melodic && git pull && \
    cd ~/melodic_ws/src/roboy_communication && git pull && \
    cd ~/melodic_ws && . /opt/ros/melodic/setup.sh && catkin_make

RUN touch /root/.bashrc && echo "source ~/melodic_ws/devel/setup.bash" >> /root/.bashrc
