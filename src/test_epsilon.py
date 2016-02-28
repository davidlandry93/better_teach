#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import subprocess

DELTA = 0.25

epsilons_rot = (0.09, 0.17)
epsilons_trans = (0.05, 0.1, 0.15, 0.2, 0.25, 0.30)

if __name__ == '__main__':

    for epsilon_rot in epsilons_rot:
        for epsilon_trans in epsilons_trans:
            name_of_dir = time.strftime("%d-%m-%Y-" +
                                        str(epsilon_rot) + "_" +
                                        str(epsilon_trans))

            try:
                os.mkdir(name_of_dir)
            except OSError:
                break

            os.chdir(name_of_dir)
            logfile = open("log.txt", 'w')

            print("Starting the run for " + str(epsilon_trans) +
                  "with rotation " + str(epsilon_rot))
            result = subprocess.call(["optimizeteachrepeatmain",
                                      "-a", str(1.0),
                                      "-b", str(1.0),
                                      "-c", str(0.17),
                                      "-i", "../../IcpConfig.yaml",
                                      "-e", str(epsilon_trans),
                                      "-r", str(epsilon_rot),
                                      "-m", "../teach/"],
                                     stdout=logfile)

            logfile.close()
            os.chdir("..")
            print("Done")
