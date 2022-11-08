#!/usr/bin/python
# -*- coding: utf-8 -*-

  # #   ######   #####  #     #  #####   #####  
  # #   #     # #     #  #   #  #     # #     # 
####### #     # #         # #         #       # 
  # #   ######  #          #     #####   #####  
####### #     # #         # #   #       #       
  # #   #     # #     #  #   #  #       #       
  # #   ######   #####  #     # ####### #######
  
# author: Thomas.Hochstrasser@zf.com

import argparse
import subprocess
import os

CARLA_FILE_NAME = "CarlaUE4.exe"
CARLA_TASK_NAME = "CarlaUE4-Win64-Shipping.exe"
AIRSIM_FILE_NAME = "CityEnviron.exe"
CARLA_SIM_RELATIV_PATH = "./carla/" + CARLA_FILE_NAME


def run_simulator(choice):
    if choice == "carla":
        subprocess.Popen([CARLA_SIM_RELATIV_PATH])
    else:
        abs_path = os.path.abspath(AIRSIM_SIM_RELATIV_CONFIG_PATH)
        subprocess.Popen(
            [
                AIRSIM_SIM_RELATIV_PATH,
                '-settings="{}"'.format(abs_path),
                "ResX=1024",
                "ResY=800",
                "-windowed",
            ]
        )


def stop_simulator(choice):
    DETACHED_PROCESS = 0x00000008
    if choice == "carla":
        subprocess.call(
            "taskkill /F /IM {}".format(CARLA_TASK_NAME), creationflags=DETACHED_PROCESS
        )
    else:
        subprocess.call(
            "taskkill /F /IM {}".format(AIRSIM_FILE_NAME),
            creationflags=DETACHED_PROCESS,
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--simulator",
        choices=["carla", "airsim"],
        required=True,
        help="Choice of simulator",
    )
    parser.add_argument(
        "--action", required=True, choices=["run", "stop"], help="Choice of action"
    )
    args = parser.parse_args()

    if args.action == "run":
        run_simulator(args.simulator)
    else:
        stop_simulator(args.simulator)


if __name__ == "__main__":

    main()
