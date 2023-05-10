#!/usr/bin/env python
import logging
import threading

import rospy
from std_msgs.msg import String

from panda_pnp_srvcli.srv import PnpRequest

from flask import Flask, render_template
from flask_ask import Ask, statement, question, session

threading.Thread(target=lambda: rospy.init_node('test_code', disable_signals=True)).start()
rospy.wait_for_service('panda_pnp_service')

app = Flask(__name__)
ask = Ask(app, "/")
logging.getLogger("flask_ask").setLevel(logging.DEBUG)

@ask.launch
def launch():
    welcome_msg = "Welcome, voice control for the panda robot has been activated. How may I help you?"
    return question(welcome_msg)

@ask.intent('HelloIntent')
def hello(pipe_type):
    pnp_request = rospy.ServiceProxy('panda_pnp_service', PnpRequest)
    result = pnp_request(pipe_type)
    text = "The result was successful!" 

    return statement(text).simple_card('Result', text)

if __name__ == '__main__':
    app.run(debug=True)
