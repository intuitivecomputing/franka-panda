# Voice Control Panda with Alexa Skill

## Installations:
1. Installing Flask-ask 
```
pip3 install https://github.com/johnwheeler/flask-ask/master.zip
pip install -r requirements-dev.txt
```
Import failure might arise due to the package versions are conflicting. This problem can be resolved with the following steps: 
```
pip uninstall jinja2
pip install jinja2==3.0
pip uninstall itsdangerous
pip install itsdangerous==2.0.1
```

2. Since Alexa skills should run either behind a public HTTPS server or a Lambda function, we'll use an open source command-line program called ngrok (http://ngrok.com/) for this purpose as it helps in opening a secure tunnel to the localhost and exposes it behind the HTTPS endpoint. 
create an account and follow the setup and installation steps below:
    
    a. download the binary zip file

    b. unzip it using ```unzip /path/to/ngrok.zip```

    c. add your authtoken to the default ngrok.yml configuration file
    ```ngrok config add-authtoken 2PQRT1B4jUlqQPYK1Q0QZgKH6cL_4iDn1KPQha57UHif7hvqx```
    
    d. To start a HTTP tunnel forwarding to your local port 5000, run the following command: ```./ngrok http 5000```

<br/>

## Operating Procedure:
1. Setup the panda robot and activate FCI mode
2. Open a termianl and type in ```./ngrok http 5000```
3. Copy the HTTPS endpoint link at the end in the ngrok terminal and paste it in the alexa skill's endpoint settings
4. Open another terminal and go to the workspace
5. Build the packages and source it
```
    catkin build
    source devel/setup.bash
```
6. Launch the pnp_server for the panda robot and the alexa skill server  

```
    roslaunch panda_pnp_srvcli pick_place.launch voice:=true
```


Or if you are trying to use simulations for the robot, then launch the following file instead:
```
    roslaunch panda_pnp_srvcli pick_place.launch voice:=true sim:=true
```

7. In the "test" tab of the alexa skill, type in or say the following phrases:

    a. ```alexa launch voice control panda.```

    b. ```alexa give me the {pipe_type} pipe.``` The {pipe_type} in the command can be short or long depending on which pipe you want it to pick up.

    c. or simply just combine them: ```alexa launch voice control panda and give me the {pipe_type} pipe.```

    Or if you are using an alexa echo dot, just say the commands above.

8. Watch the robot picking up the pipe for you





## Control Panda with rostopics

If you don't want to use voice control, you can also use ros client to send request to pick up certain type of pipes.

1. Launch the pnp_server for the panda robot
```
    roslaunch panda_pnp_srvcli pick_place.launch 
```

2. Open another terminal and source the path. Then run the following command:
```
    rosrun panda_pnp_srvcli pnp_client {pipe_type}
```

