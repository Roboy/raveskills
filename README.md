[![Build Status](https://travis-ci.org/Roboy/raveskills.svg?branch=master)](https://travis-ci.org/Roboy/raveskills)
[![codecov](https://codecov.io/gh/Roboy/raveskills/branch/master/graph/badge.svg)](https://codecov.io/gh/Roboy/raveskills)

# raveskills
Roboy skills developed using the ravestate framework.

There exist the ice cream selling skill called [Luigi](/modules/luigi) and the skill to order ice cream via Telegram in the [Driving](/modules/driving) module. Both modules include READMEs with information on further usage.

## Setup

### Initial configuration and setup

Clone the repository and install dependencies:

```bash
# Create a virtual python environment to not pollute the global setup
python3 -m virtualenv python-ravestate

# Source the virtual environment
. python-ravestate/bin/activate

# Clone the repo
git clone git@github.com:roboy/raveskills && cd raveskills

# Install normal requirements
pip install -r requirements.txt

# Link your local raveskills clone into your virtualenv
pip install -e .
```

Now, launch a Neo4j docker instance to serve [Scientio](https://github.com/roboy/scientio), so the dialog system has a memory:
```bash
docker run \
    --publish=7474:7474 --publish=7687:7687 \
    --volume=$HOME/neo4j/data:/data \
    --volume=$HOME/neo4j/logs:/logs \
    neo4j:latest
    
# Open the address localhost:7474 in a browser, and enter the
# credentials `neo4j`/`neo4j`. You will then be prompted to enter
# your own password. Remember this password.
```

In the `config` folder, create a file called `keys.yml`. It should have the following content:

```yaml
module: telegramio
config:
  telegram-token: <sexycactus>  # This is where your own telegram bot token
                                # will go later
---
module: ontology
config:
  neo4j_address: bolt://localhost:7687  # Your neo4j server uri here
  neo4j_username: neo4j                 # Your neo4j user here
  neo4j_pw: test                        # Your neo4j pw here
```

You may now conduct your first conversation with ravestate:
```bash
python3 -m ravestate -f config/generic.yml -f config/keys.yml
```

After the conversation, check the Neo4j interface under `localhost:7474`. It should now contain some nodes!

__Reminder: Whenever you use ravestate/raveskills from the command line, source the virtual environment first!__

### Running your Telegram bot

To test your telegram bot with a custom bot token in your `keys.yml`,
just run `telegram_test.yml` instead of `generic.yml`. This will load the `ravestate_telegramio` module.

### Setting up PyCharm

1. Open your local ravestate clone as a project in pycharm.
2. Under `Project Preferences > Python interpreter`, set your virtual environment.
3. Mark the `modules` folder as sources root via the right-click context menu.
4. Create a run config via the "Edit configurations menu":<br>
   • Create a new Python configuration.<br>
   • Instead of `Script path`, select `Module name`, enter `ravestate`<br>
   • Set parameters to `-f config/generic.yml -f config/keys.yml`.<br>
   • Alternately, set parameters to `ravestate_conio my_hello_world_skill` to test your own module.<br>    
5. You should now be able to run any ravestate config from pycharm.


## Docker for ROS and ROS2

There is a Dockerfile for ROS and ROS2 support which can be built with
```bash
docker build -t raveskills-ros2-image .
```
The image contains ROS, ROS2 and a ROS Bridge to connect ROS with ROS2.
Furthermore the roboy_communication message and service types are installed.

A container can then be created with the docker-compose.yml:
```bash
docker-compose up --detach raveskills
```
The container is now running and a connection into the container can be
established with:
```bash
docker exec -it raveskills-ros2-container bash
```
Inside the container, first source the ROS2 setups and then 
raveskills can be run with ROS2 and rclpy available.
```bash
source ~/ros2_ws/install/setup.sh
python3 -m ravestate [...]
```

### Start ROS Bridge
In order to start ROS Bridge, the image and container have to be set up
as above. After connecting into the container run from inside the container:
```bash
export ROS_IP=192.168.0.105  # Use Docker host IP
source ~/melodic_ws/devel/setup.sh
source ~/ros2_ws/install/setup.sh
source ~/ros1_bridge_ws/install/setup.sh
ros2 run ros1_bridge dynamic_bridge
```
