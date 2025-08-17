#!/usr/bin/python

import argparse
import base64
import json
import math
from pathlib import Path
import numpy as np


def embed_imu_data_to_json(imu_data, json_path, output_json_path):
    # read the JSON file
    with open(json_path, "r") as file:
        geojson_dict = json.load(file)

    encoded_data = base64.b64encode(json.dumps(imu_data).encode("utf-8")).decode(
        "utf-8"
    )

    # print(encoded_data)
    # decoded_data = base64.b64decode(encoded_data).decode('utf-8')
    # print(decoded_data)

    properties_dict = geojson_dict["features"][0]["properties"]

    # Embed the IMU data into the GeoJSON properties
    stride = 2000
    seg_num = math.ceil(len(encoded_data) / stride)
    properties_dict["data_type"] = "imu"
    properties_dict["seg_num"] = seg_num
    for i in range(seg_num):
        k = "encoded_data_seg" + str(i)
        start = i * stride
        end = start + stride
        properties_dict[k] = encoded_data[start:end]

    # write the updated GeoJSON back to the file
    with open(output_json_path, "w") as file:
        json.dump(geojson_dict, file, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Embed IMU data into a GeoJSON file.")
    parser.add_argument(
        "--filename", "-f", required=True, help="Input GeoJSON file path."
    )
    parser.add_argument(
        "--output", "-o", default="output.json", help="Output GeoJSON file path."
    )
    args = parser.parse_args()

    imu_data = [[0, 0, 9.81] for _ in range(200)]

    embed_imu_data_to_json(imu_data, args.filename, args.output)
    print(f"IMU data has been embedded into {args.output}.")
