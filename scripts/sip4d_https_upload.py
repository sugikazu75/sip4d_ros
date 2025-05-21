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

# upload_file = args.file
upload_file = (
    Path(__file__).resolve().parent.parent
    / "sample_data/082_99-999-99_20250317120000_1.zip"
)

with requests.Session() as s:

    with open(upload_file, "rb") as f:

        r = s.post(
            path,
            auth=(user, passwd),
            files={"zip_file": f},
            headers={"Connection": "close"},
        )

        if r.status_code != 200:
            print("cannot estabolish the connection, {}".format(r.status_code))

        print("sccueed to upload")
