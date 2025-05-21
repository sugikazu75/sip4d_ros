#!/usr/bin/python

import argparse
import json
import os
from pathlib import Path
import requests

# Upload

parser = argparse.ArgumentParser()
parser.add_argument("--file", "-f")
args = parser.parse_args()

user = ""
passwd = ""
path = ""


def getUserdataFromFile(filename):
    global user
    global passwd
    global path
    try:
        with open(filename, "r") as f:
            userdata = json.load(f)
            user = userdata["user"]
            passwd = userdata["passwd"]
            path = userdata["path"]
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Failed to load {e}")


def getUserdataFromEnv():
    global user
    global passwd
    global path
    user = os.environ.get("SIP4D_UPLOAD_USER")
    passwd = os.environ.get("SIP4D_UPLOAD_PASSWD")
    path = os.environ.get("SIP4D_UPLOAD_URL")


getUserdataFromEnv()
getUserdataFromFile(Path(__file__).resolve().parent.parent / "userdata.json")
