import sys
from absl import logging
import time

import yaml
from common.angle import *
from fileio import *

from kf_gins_src.gi_engine import *

def main(argv):
    # --------------------------------------------------------------------------------------------- open files
    if len(argv) != 2:
        print("usage: python script.py kf-gins.yaml")
        sys.exit(-1)

    print("\nKF-GINS: An EKF-Based GNSS/INS Integrated Navigation System\n")
    ts = time.time()

    # Load configuration file
    try:
        with open(argv[1], 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
    except Exception as e:
        print("Failed to read configuration file. Please check the path and format of the configuration file!")
        sys.exit(-1)

    # Read configuration parameters into GINSOptions and construct GIEngine
    options = GINSOptions()
    judge, options = loadConfig(config, options)
    if not judge:
        print("Error occurs in the configuration file!")
        sys.exit(-1)

    # Load filepath configuration
    try:
        imupath = config["imupath"]
        gnsspath = config["gnsspath"]
        outputpath = config["outputpath"]
    except KeyError:
        print("Failed when loading configuration. Please check the file path and output path!")
        sys.exit(-1)

    # Imudata configuration, data processing interval
    try:
        imudatalen = config["imudatalen"]
        imudatarate = config["imudatarate"]
        starttime = config["starttime"]
        endtime = config["endtime"]
    except KeyError:
        print("Failed when loading configuration. Please check the data length, data rate, and the process time!")
        sys.exit(-1)

    # Load GNSS file and IMU file
    gnssfile = GnssFileLoader(gnsspath)
    imufile = ImuFileLoader(imupath, imudatalen, imudatarate)

    #  ---------------------------------------------------------------------- file opened

    #  ---------------------------------------------------------------------- load into GIEngine
    giengine = GIEngine(options)

    nav_columns = 11
    imuerr_columns = 13
    std_columns = 22
    navfile = FileSaver(outputpath + "/KF_GINS_Navresult.nav", nav_columns, FileSaver.TEXT)
    imuerrfile = FileSaver(outputpath + "/KF_GINS_IMU_ERR.txt", imuerr_columns, FileSaver.TEXT)
    stdfile = FileSaver(outputpath + "/KF_GINS_STD.txt", std_columns, FileSaver.TEXT)

    # Check if files are opened
    if not (gnssfile.isOpen() and imufile.isOpen() and navfile.isOpen() and imuerrfile.isOpen() and stdfile.isOpen()):
        print("Failed to open data file!")
        sys.exit(-1)

    # Check process time
    if endtime < 0:
        endtime = imufile.endtime()
    if endtime > 604800 or starttime < imufile.starttime() or starttime > endtime:
        print("Process time ERROR!")
        sys.exit(-1)

    # Data alignment
    imu_cur = IMU()
    imu_cur = imufile.next()
    while imu_cur.time < starttime:
        imu_cur = imufile.next()

    gnss = GNSS()
    gnss = gnssfile.next()
    while gnss.time <= starttime:
        gnss = gnssfile.next()

    # Add imudata to GIEngine and compensate IMU error
    giengine.addImuData(imu_cur, True)

    # Add gnssdata to GIEngine
    giengine.addGnssData(gnss)

    # Used to save processing results
    timestamp = 0.0
    navstate = NavState()
    cov = None

    # Used to display processing progress
    percent = 0
    lastpercent = 0
    interval = endtime - starttime

    while True:
        # Load new gnssdata when current state time is newer than GNSS time and add it to GIEngine
        if gnss.time < imu_cur.time and not gnssfile.isEof():
            gnss = gnssfile.next()
            giengine.addGnssData(gnss)

        # Load new imudata and add it to GIEngine
        imu_cur = imufile.next()
        if imu_cur.time > endtime or imufile.isEof():
            break
        giengine.addImuData(imu_cur)

        # Process new imudata
        giengine.newImuProcess()

        # Get current timestamp, navigation state, and covariance
        timestamp = giengine.timestamp()
        navstate = giengine.getNavState()
        cov = giengine.getCovariance()

        # Save processing results
        writeNavResult(timestamp, navstate, navfile, imuerrfile)
        writeSTD(timestamp, cov, stdfile)

        # Display processing progress
        percent = int((imu_cur.time - starttime) / interval * 100)
        if percent - lastpercent >= 1:
            print(f" - Processing: {percent:3d}%", end='\r', flush=True)
            lastpercent = percent

    # Close opened files
    imufile.close()
    gnssfile.close()
    navfile.close()
    imuerrfile.close()
    stdfile.close()

    te = time.time()
    print(f"\n\nKF-GINS Process Finish! From {starttime} s to {endtime} s, total {interval} s!")
    print(f"Cost {te - ts} s in total")
    #  ---------------------------------------------------------------------- loaded in


def loadConfig(config, optionin):
    """
    Load initial states of GIEngine from configuration file and convert them to standard units.
    """
    # Constants for unit conversions
    D2R = Angle.D2R
    options = GINSOptions()
    # Load initial position, velocity, and attitude from configuration file
    try:
        vec1 = config["initpos"]
        vec2 = config["initvel"]
        vec3 = config["initatt"]
    except KeyError:
        print("Failed when loading configuration. Please check initial position, velocity, and attitude!")
        return False

    for i in range(3):
        options.initstate.pos[i] = vec1[i] * D2R
        options.initstate.vel[i] = vec2[i]
        options.initstate.euler[i] = vec3[i] * D2R

    options.initstate.pos[2] *= (180.0 / np.pi)

    # Load initial IMU error (bias and scale factor) from configuration file
    try:
        vec1 = config["initgyrbias"]
        vec2 = config["initaccbias"]
        vec3 = config["initgyrscale"]
        vec4 = config["initaccscale"]
    except KeyError:
        print("Failed when loading configuration. Please check initial IMU error!")
        return False

    for i in range(3):
        options.initstate.imuerror.gyrbias[i] = vec1[i] * D2R / 3600.0
        options.initstate.imuerror.accbias[i] = vec2[i] * 1e-5
        options.initstate.imuerror.gyrscale[i] = vec3[i] * 1e-6
        options.initstate.imuerror.accscale[i] = vec4[i] * 1e-6

    # Load initial position std, velocity std and attitude std from configuration file
    try:
        vec1 = config["initposstd"]
        vec2 = config["initvelstd"]
        vec3 = config["initattstd"]
    except KeyError:
        print("Failed when loading configuration. Please check initial std of position, velocity, and attitude!")
        return False

    for i in range(3):
        options.initstate_std.pos[i] = vec1[i]
        options.initstate_std.vel[i] = vec2[i]
        options.initstate_std.euler[i] = vec3[i] * D2R

    # Load IMU noise parameters from configuration file
    try:
        vec1 = config["imunoise"]["arw"]
        vec2 = config["imunoise"]["vrw"]
        vec3 = config["imunoise"]["gbstd"]
        vec4 = config["imunoise"]["abstd"]
        vec5 = config["imunoise"]["gsstd"]
        vec6 = config["imunoise"]["asstd"]

        options.imunoise.corr_time = config["imunoise"]["corrtime"]
    except KeyError:
        print("Failed when loading configuration. Please check IMU noise!")
        return False

    for i in range(3):
        options.imunoise.gyr_arw[i] = vec1[i]
        options.imunoise.acc_vrw[i] = vec2[i]
        options.imunoise.gyrbias_std[i] = vec3[i]
        options.imunoise.accbias_std[i] = vec4[i]
        options.imunoise.gyrscale_std[i] = vec5[i]
        options.imunoise.accscale_std[i] = vec6[i]

    # Load initial imu bias and scale std, set to bias and scale instability std if load failed
    try:
        vec1 = config["initbgstd"]
    except KeyError:
        vec1 = [optionin.imunoise.gyrbias_std[0], optionin.imunoise.gyrbias_std[1], optionin.imunoise.gyrbias_std[2]]

    try:
        vec2 = config["initbastd"]
    except KeyError:
        vec2 = [optionin.imunoise.accbias_std[0], optionin.imunoise.accbias_std[1], optionin.imunoise.accbias_std[2]]

    try:
        vec3 = config["initsgstd"]
    except KeyError:
        vec3 = [optionin.imunoise.gyrscale_std[0], optionin.imunoise.gyrscale_std[1], optionin.imunoise.gyrscale_std[2]]

    try:
        vec4 = config["initsastd"]
    except KeyError:
        vec4 = [optionin.imunoise.accscale_std[0], optionin.imunoise.accscale_std[1], optionin.imunoise.accscale_std[2]]

    for i in range(3):
        options.initstate_std.imuerror.gyrbias[i] = vec1[i] * D2R / 3600.0
        options.initstate_std.imuerror.accbias[i] = vec2[i] * 1e-5
        options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6
        options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6

    # Convert imu noise parameters' units to standard units
    options.imunoise.gyr_arw *= (D2R / 60.0)
    options.imunoise.acc_vrw /= 60.0
    options.imunoise.gyrbias_std *= (D2R / 3600.0)
    options.imunoise.accbias_std *= 1e-5
    options.imunoise.gyrscale_std *= 1e-6
    options.imunoise.accscale_std *= 1e-6
    options.imunoise.corr_time *= 3600

    # Load GNSS antenna leverarm from configuration file
    try:
        vec1 = config["antlever"]
    except KeyError:
        print("Failed when loading configuration. Please check antenna leverarm!")
        return False

    options.antlever = np.array(vec1)

    return True, options

def writeNavResult(time: float, navstate: NavState, navfile: FileSaver, imuerrfile: FileSaver):
    """
    Save navigation result and IMU error.
    """
    # Constants for unit conversions
    R2D = Angle.R2D

    # Save navigation result
    result = [
        0,
        time,
        navstate.pos[0] * R2D,
        navstate.pos[1] * R2D,
        navstate.pos[2],
        navstate.vel[0],
        navstate.vel[1],
        navstate.vel[2],
        navstate.euler[0] * R2D,
        navstate.euler[1] * R2D,
        navstate.euler[2] * R2D
    ]
    navfile.dump(result)

    # Save IMU error
    imuerr = navstate.imuerror
    result = [
        time,
        imuerr.gyrbias[0] * R2D * 3600,
        imuerr.gyrbias[1] * R2D * 3600,
        imuerr.gyrbias[2] * R2D * 3600,
        imuerr.accbias[0] * 1e5,
        imuerr.accbias[1] * 1e5,
        imuerr.accbias[2] * 1e5,
        imuerr.gyrscale[0] * 1e6,
        imuerr.gyrscale[1] * 1e6,
        imuerr.gyrscale[2] * 1e6,
        imuerr.accscale[0] * 1e6,
        imuerr.accscale[1] * 1e6,
        imuerr.accscale[2] * 1e6
    ]
    imuerrfile.dump(result)

def writeSTD(time, cov, stdfile):
    """
    Save standard deviation, converted to common units.
    """
    # Constants for unit conversions
    R2D = Angle.R2D

    result = [time]

    # Save position, velocity, and attitude std
    for i in range(6):
        result.append(np.sqrt(cov[i, i]))

    for i in range(6, 9):
        result.append(np.sqrt(cov[i, i]) * R2D)

    # Save IMU error std
    for i in range(9, 12):
        result.append(np.sqrt(cov[i, i]) * R2D * 3600)

    for i in range(12, 15):
        result.append(np.sqrt(cov[i, i]) * 1e5)

    for i in range(15, 21):
        result.append(np.sqrt(cov[i, i]) * 1e6)

    stdfile.dump(result)


if __name__ == '__main__':
    app.run(main)

