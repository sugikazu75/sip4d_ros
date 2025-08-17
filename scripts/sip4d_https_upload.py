#!/usr/bin/python

import argparse
import json
import os
from pathlib import Path
import requests


def getUserdataFromFile(filename):
    try:
        with open(filename, "r") as f:
            userdata = json.load(f)
            user = userdata["user"]
            passwd = userdata["passwd"]
            path = userdata["path"]
            return user, passwd, path
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Failed to load {e}")
        return None, None, None


def getUserdataFromEnv():
    user = os.environ.get("SIP4D_UPLOAD_USER", None)
    passwd = os.environ.get("SIP4D_UPLOAD_PASSWD", None)
    path = os.environ.get("SIP4D_UPLOAD_URL", None)
    return user, passwd, path


def upload_zip_to_sip4d(filename):
    user, passwd, path = "", "", ""
    user_env, passwd_env, path_env = getUserdataFromEnv()
    user_file, passwd_file, path_file = getUserdataFromFile(
        Path(__file__).resolve().parent.parent / "userdata.json"
    )
    if (user_env is None) and (user_file is None):
        print("No user data found. Please set environment variables or provide a file.")
        return
    elif user_env is not None:
        user = user_env
        passwd = passwd_env
        path = path_env
    elif user_file is not None:
        user = user_file
        passwd = passwd_file
        path = path_file

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
