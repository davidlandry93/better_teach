#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import subprocess

DELTA = 0.2

if __name__ == '__main__':

    for rotationError in (0.09, 0.17):
        for lateralError in map(lambda x: x * DELTA, range(1,11)):
            name_of_dir = time.strftime("%d-%m-%Y-" + str(lateralError) + str(lateralError))

            try:
                os.mkdir(name_of_dir)
            except OSError:
                break

            os.chdir(name_of_dir)
            logfile = open("log.txt", 'w')

            print("Starting the run for " + str(lateralError) + "with rotation " + str(rotationError))
            result = subprocess.call(["optimizeteachrepeatmain",
                                      "-a", str(lateralError),
                                      "-b", str(lateralError),
                                      "-c", str(rotationError),
                                      "-i", "../../IcpConfig.yaml",
                                      "-e", str(0.1), "-r", str(0.045),
                                      "-m", "../teach/"],
                                     stdout=logfile)


            logfile.close()
            os.chdir("..")
            print("Done")
