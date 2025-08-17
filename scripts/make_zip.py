#!/usr/bin/python

import os
import sys
import argparse
import zipfile


def make_zip_from_directory(directory, output_filename):
    with zipfile.ZipFile(output_filename, "w", zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, directory)
                zipf.write(file_path, arcname)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", "-d", required=True, help="Directory to zip")
    parser.add_argument("--output", "-o", required=True, help="Output zip file name")
    args = parser.parse_args()

    make_zip_from_directory(args.directory, args.output)
