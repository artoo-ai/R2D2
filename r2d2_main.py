#!/usr/bin/env python

import time
from frame.arms import Arms
from flask import Flask, render_template, Response, request, redirect, url_for
from threading import Thread
import os


class R2D2:

    def __init__(self):
        """
        Initialize the R2D2.
        """
        # Init the Arms
        self.arms = Arms(17, 4)

        # Init the web server for the GUI
        #self.web_thread = Thread(target=self.init_web_server)
        #self.web_thread.start()
        self.init_web_server()

    def shutdown(self):
        """
        Shutdown the R2D2
        :return:
        """
        self.arms.shutdown()

    def init_web_server(self):
        template_dir =os.path.dirname(os.path.realpath(__file__))
        template_dir = os.path.join(template_dir, 'web')
        print(template_dir)

        app = Flask(__name__, template_folder=template_dir)

        @app.route("/")
        def index():
            return render_template('index.html')
            #return "Hello World!"

        @app.route("/test_arms/", methods=['GET', 'POST'])
        def test_arms():
            self.test_arms()
            #return "Test Arms"
            return render_template('index.html')

        @app.route("/open_top_arm/", methods=['GET', 'POST'])
        def open_top_arm():
            self.arms.open_top_arm()
            time.sleep(0.5)
            self.arms.close_top_arm()
            return render_template('index.html')

        @app.route("/open_bottom_arm/", methods=['GET', 'POST'])
        def open_bottom_arm():
            self.arms.open_bottom_arm()
            time.sleep(0.5)
            self.arms.close_bottom_arm()
            return render_template('index.html')

        app.run()

    def test_arms(self):
        """
        Test the arms.
        :return:
        """
        self.arms.test_arms()


if __name__ == '__main__':
    r2d2 = R2D2()
    #r2d2.test_arms()
    #r2d2.shutdown()
