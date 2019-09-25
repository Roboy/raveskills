# Driving
Roboy raveskill to order ice cream via a Telegram bot.

## Setup

### Raveskills
Follow the steps described in the raveskills [README.md](../../README.md) to set up and configure the environment. You don't need to setup [Scientio](https://github.com/roboy/scientio), however, you should create a `keys.yml` as described since this is needed for the Telegram bot.

### Telegram bot
On Telegram talk to `BotFather` as described in the [Telegram documentation](https://core.telegram.org/bots#6-botfather) to create your bot. The token you receive during the configuration is the one that needs to be inserted in the `keys.yml`.


## Usage
### Starting the docker container:
We assume that you have already built a Docker container as described in the [raveskills README.md](../../README.md). Now you can execute it using the commands: 
```bash
cd /path/to/raveskills
docker-compose up --detach raveskills
docker exec -ti raveskills bash -l
```
Depending on how your container is configured, you might need to also run the following command inside the container:
```bash
source ~/melodic_ws/devel/setup.sh
```

### Running the driving module using docker:
Create a `keys.yml` under `/path/to/raveskills/config` as described in the [raveskills README.md](../../README.md).
Open 5 tabs in the `raveskills`-container as described above. In the first tab start roscore with
```bash
roscore
```
In the second tab execute
```bash
cd /raveskills/modules/luigi/ && python3 ad_server_mockup.py
```
In the third tab run
```bash
cd /raveskills/modules/luigi/ && python3 ws_server.py
```
In the fourth
```bash
cd /raveskills/modules/luigi/ && python3 ws_communication.py
```
And finally in the fifth tab: 
```bash
cd /raveskills/modules/luigi/ && python3 -m ravestate ravestate_conio -f /raveskills/config/roboy_telegram_bot_master.yml -f /raveskills/config/keys.yml
```
Now you can talk to the Driving dialogue either on telegram or the command line directly. The location will be send to autonomous driving. If the luigi dialogue is run in parallel, the mock-up arrival signal by the autonomous driving dialogue can be send and received as described in the [Luigi README.md](../luigi/README.md).