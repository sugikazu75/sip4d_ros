#!/usr/bin/python

import argparse
import json
import os
from pathlib import Path
import requests


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
            return user, passwd, path
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Failed to load {e}")


def getUserdataFromEnv():
    global user
    global passwd
    global path
    user = os.environ.get("SIP4D_UPLOAD_USER")
    passwd = os.environ.get("SIP4D_UPLOAD_PASSWD")
    path = os.environ.get("SIP4D_UPLOAD_URL")
    return user, passwd, path


def upload_zip_to_sip4d(filename):
    user, passwd, path = getUserdataFromEnv()
    user, passwd, path = getUserdataFromFile(
        Path(__file__).resolve().parent.parent / "userdata.json"
    )

    with requests.Session() as s:
        with open(filename, "rb") as f:
            r = s.post(
                path,
                auth=(user, passwd),
                files={"zip_file": f},
                headers={"Connection": "close"},
            )
            if r.status_code != 200:
                print("cannot estabolish the connection, {}".format(r.status_code))
            else:
                print("sccueed to upload")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f")
    args = parser.parse_args()

    upload_zip_to_sip4d(args.file)
