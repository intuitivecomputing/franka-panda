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

## Setting up

1. Open a termianl and type in ```./ngrok http 5000```
2. Copy the HTTPS endpoint link at the end in the ngrok terminal and paste it in the alexa skill's endpoint settings
3. Open another terminal and go to the workspace (for ICL students, panta_test_ws)
4. Build the packages and source it
```
    catkin build
    source devel/setup.bash
```
5. use the following command to launch the alexa skill server
```
    rosrun alexa_pnp alexa_skill_ros.py
```

<br/><br/>

__Reference__: https://subscription.packtpub.com/book/iot-&-hardware/9781838649326/7/ch07lvl1sec70/getting-started-with-alexa-and-connecting-with-ros
