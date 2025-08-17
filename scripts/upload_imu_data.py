#!/usr/bin/python

import rospy
import argparse
from pathlib import Path
import tempfile
import shutil

from make_zip import make_zip_from_directory
from sip4d_https_upload import upload_zip_to_sip4d
from imu_data_embed import embed_imu_data_to_json
from imu_data_collection import SlidingWindowBuffer

if __name__ == "__main__":
    rospy.init_node("imu_data_upload", anonymous=False)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot_ns", "-r", type=str, default="", help="Robot namespace"
    )
    args = parser.parse_args()

    collector = SlidingWindowBuffer(robot_ns=args.robot_ns)

    duration = 60
    last_upload_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        if (rospy.Time.now().to_sec() - last_upload_time) / 60.0 > duration:
            imu_data = collector.get_data()
            last_upload_time = rospy.Time.now().to_sec()
            print(
                "uploading imu data at {}. data length: {}".format(
                    last_upload_time, len(imu_data)
                )
            )

            with tempfile.TemporaryDirectory() as temp_dir:
                tempdir_path = Path(temp_dir)
                working_dir = tempdir_path / "imu_data"

                # copy template files to the temporary directory
                shutil.copytree(
                    Path(__file__).resolve().parent.parent / "template/imu", working_dir
                )
                geojson_file = list(working_dir.glob("*.geojson"))[0]

                embed_imu_data_to_json(imu_data, geojson_file, geojson_file)

                make_zip_from_directory(working_dir, tempdir_path / "imu_data.zip")
                upload_zip_to_sip4d(tempdir_path / "imu_data.zip")

                # shutil.copy(tempdir_path / "imu_data.zip", Path.cwd() / "imu_data.zip")
