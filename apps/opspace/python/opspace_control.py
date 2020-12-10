#!/usr/bin/env python

import argparse
import json
import os
import signal
import sys
import time

import redis
import numpy as np

import ctrlutils
from ctrlutils.numpy import *
import frankapanda
import spatialdyn
from spatialdyn import eigen

g_runloop = True

def signal_handler(sig, frame):
    global g_runloop
    g_runloop = False

# Redis keys
KEY_PREFIX = "franka_panda::"
KEY_MODELS_PREFIX  = KEY_PREFIX + "model::"
KEY_OBJECTS_PREFIX = KEY_PREFIX + "object::"
KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::"

# GET keys
KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q"
KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq"
KEY_MODEL_EE      = KEY_PREFIX + "model::inertia_ee"
KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status"

# SET keys
KEY_CONTROL_TAU     = KEY_PREFIX + "control::tau"
KEY_CONTROL_MODE    = KEY_PREFIX + "control::mode"
KEY_CONTROL_POS_DES = KEY_PREFIX + "control::pos_des"
KEY_CONTROL_ORI_DES = KEY_PREFIX + "control::ori_des"
KEY_CONTROL_POS     = KEY_PREFIX + "control::pos"
KEY_CONTROL_ORI     = KEY_PREFIX + "control::ori"
KEY_CONTROL_POS_ERR = KEY_PREFIX + "control::pos_err"
KEY_CONTROL_ORI_ERR = KEY_PREFIX + "control::ori_err"
KEY_TRAJ_POS        = KEY_TRAJ_PREFIX + "pos"

# Webapp keys
NAME_APP          = "simulator"
KEY_WEB_RESOURCES = "webapp::resources"
KEY_WEB_ARGS      = "webapp::" + NAME_APP + "::args"
KEY_WEB_INTERACTION = "webapp::" + NAME_APP + "::interaction"

# SUB keys
KEY_PUB_COMMAND = KEY_PREFIX + "control::pub::command"

# PUB keys
KEY_PUB_STATUS = KEY_PREFIX + "control::pub::status"

# Controller gains
KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos"
KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori"
KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint"

EE_OFFSET          = np.array([0., 0., 0.107]) + np.array([0., 0., 0.1034])
Q_HOME             = np.array([0., -np.pi/6., 0., -5./6. * np.pi, 0., 2./3. * np.pi, 0.])
KP_KV_POS          = np.array([[80., 12.],
                               [80., 12.],
                               [80., 12.]])
KP_KV_ORI          = np.array([80., 10.])
KP_KV_JOINT        = np.array([5., 0.])
TIMER_FREQ         = 1000.
GAIN_KEY_PRESS_POS = 0.1 / TIMER_FREQ
GAIN_KEY_PRESS_ORI = 0.3 / TIMER_FREQ
GAIN_CLICK_DRAG    = 100.
MAX_ERROR_POS      = 80 * 0.05
MAX_ERROR_ORI      = 80 * np.pi / 20
EPSILON_POS        = 0.05
EPSILON_ORI        = 0.2
EPSILON_VEL_POS    = 0.005
EPSILON_VEL_ORI    = 0.005
MAX_FORCE          = 100
TIME_PUB_WAIT      = 1  # sec


def pd_control(x, x_des, dx, kp_kv):
    if type(x) is eigen.Quaterniond:
        ori_err = spatialdyn.opspace.orientation_error(x, x_des)
        return -kp_kv[0] * ori_err - kp_kv[1] * dx, ori_err
    x_err = x - x_des
    return -kp_kv[0] * x_err - kp_kv[1] * dx, x_err

def main():
    # Parse args
    parser = argparse.ArgumentParser(description=(
        "Operational space controller for the Franka Panda."
    ))
    parser.add_argument("urdf", help="franka_panda.urdf")
    parser.add_argument("--sim", help="run in simulation", action="store_true")
    args = parser.parse_args()

    # Create timer and Redis client
    timer = ctrlutils.Timer(1000)
    redis_client = redis.Redis("127.0.0.1")
    redis_pipe   = redis_client.pipeline(transaction=False)

    # Load robot
    ab = frankapanda.ArticulatedBody(spatialdyn.urdf.load_model(args.urdf))
    if args.sim:
        ab.q  = Q_HOME
        ab.dq = np.zeros((ab.dof,))
    else:
        ab.q  = decode_matlab(redis_client.get(KEY_SENSOR_Q))
        ab.dq = decode_matlab(redis_client.get(KEY_SENSOR_DQ))
    # ab.inertia_compensation = np.array([0.2, 0.1, 0.1])
    # ab.stiction_coefficients = np.array([0.8, 0.8, 0.6])
    # ab.stiction_activations = np.array([0.1, 0.1, 0.1])

    # Initialize controller parameters
    q_des    = Q_HOME.copy()
    x_des    = spatialdyn.position(ab, -1, EE_OFFSET)
    quat_des = spatialdyn.orientation(ab)

    # Initialize Redis keys
    initialize_web_app(redis_pipe, ab, args.urdf)
    if args.sim:
        redis_pipe.set(KEY_SENSOR_Q, encode_matlab(ab.q))
        redis_pipe.set(KEY_SENSOR_DQ, encode_matlab(ab.dq))
    redis_pipe.set(KEY_KP_KV_POS, encode_matlab(KP_KV_POS))
    redis_pipe.set(KEY_KP_KV_ORI, encode_matlab(KP_KV_ORI))
    redis_pipe.set(KEY_KP_KV_JOINT, encode_matlab(KP_KV_JOINT))
    redis_pipe.execute()

    # Get end-effector model from driver
    redis_ee = redis_client.get(KEY_MODEL_EE)
    if redis_ee is not None:
        try:
            json_ee  = json.loads(redis_ee.decode("utf8"))
            m_ee     = float(json_ee["m"])
            com_ee   = np.array(list(map(float, json_ee["com"])))
            I_com_ee = np.array(list(map(float, json_ee["I_com"])))
            I_load   = spatialdyn.SpatialInertiad(m_ee, com_ee, I_com_ee)
            ab.replace_load(I_load)
        except:
            pass

    # Create signal handler
    global g_runloop
    signal.signal(signal.SIGINT, signal_handler)

    is_initialized = False  # Flag to set control mode on first iteration

    try:
        while g_runloop:
            # Wait for next loop
            timer.sleep()

            # Get Redis values
            redis_pipe.get(KEY_KP_KV_POS)
            redis_pipe.get(KEY_KP_KV_ORI)
            redis_pipe.get(KEY_KP_KV_JOINT)
            redis_pipe.get(KEY_WEB_INTERACTION)

            # Update robot state
            if not args.sim:
                redis_pipe.get(KEY_DRIVER_STATUS)
                redis_pipe.get(KEY_SENSOR_Q)
                redis_pipe.get(KEY_SENSOR_DQ)
                redis_results = redis_pipe.execute()

                # Break if driver is not running
                driver_status = redis_results[4].decode("utf8")
                if driver_status != "running":
                    break

                # Update robot state
                ab.q, ab.dq = tuple(map(decode_matlab, redis_results[5:]))
            else:
                redis_results = redis_pipe.execute()

            # Update gains
            kp_kv_pos, kp_kv_ori, kp_kv_joint = tuple(map(decode_matlab, redis_results[:3]))
            interaction = json.loads(redis_results[3].decode("utf-8"))

            # Compute Jacobian
            J = spatialdyn.jacobian(ab, offset=EE_OFFSET)

            # Compute position PD control
            x   = spatialdyn.position(ab, offset=EE_OFFSET)
            dx  = J[:3].dot(ab.dq)
            ddx, x_err = pd_control(x, x_des, dx, kp_kv_pos)

            # Compute orientation PD control
            quat = spatialdyn.opspace.near_quaternion(spatialdyn.orientation(ab), quat_des)
            w    = J[3:].dot(ab.dq)
            dw, ori_err = pd_control(quat, quat_des, w, kp_kv_ori)

            # Compute opspace torques
            N = np.eye(ab.dof)
            if spatialdyn.opspace.is_singular(ab, J, svd_epsilon=0.01):
                tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J[:3], ddx, N, svd_epsilon=0.01)
            else:
                ddx_dw  = np.hstack((ddx, dw))
                tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J, ddx_dw, N, svd_epsilon=0.01)

            # Add joint task in nullspace
            I   = np.eye(ab.dof)
            ddq, _ = pd_control(ab.q, q_des, ab.dq, kp_kv_joint)
            tau_cmd += spatialdyn.opspace.inverse_dynamics(ab, I, ddq, N)

            # Add friction compensation
            tau_cmd += frankapanda.friction(ab, tau_cmd)

            # Add gravity compensation
            tau_cmd += spatialdyn.gravity(ab)

            # Parse interaction from web app
            adjust_position(interaction["key_down"], x_des)
            adjust_orientation(interaction["key_down"], quat_des)

            # Send control torques
            redis_client.set(KEY_CONTROL_TAU, encode_matlab(tau_cmd))

            if args.sim:
                # Integrate
                f_ext = compute_external_forces(ab, interaction)
                spatialdyn.integrate(ab, tau_cmd, timer.dt, f_ext, friction=True)

                redis_pipe.set(KEY_SENSOR_Q, encode_matlab(ab.q))
                redis_pipe.set(KEY_SENSOR_DQ, encode_matlab(ab.dq))
            elif not is_initialized:
                # Send control mode on first iteration (after setting torques)
                redis_pipe.set(KEY_CONTROL_MODE, "torque")
                is_initialized = True

            # Send trajectory info to visualizer
            redis_pipe.set(KEY_CONTROL_POS, encode_matlab(x_des))
            redis_pipe.set(KEY_TRAJ_POS, encode_matlab(x))
            redis_pipe.set(KEY_TRAJ_ORI, encode_matlab(quat.coeffs))
            redis_pipe.set(KEY_TRAJ_POS_ERR, encode_matlab(x_err))
            redis_pipe.set(KEY_TRAJ_ORI_ERR, encode_matlab(ori_err))
            redis_pipe.execute()

    except Exception as e:
        print(e)

    # Clear torques
    if not args.sim:
        redis_client.set(KEY_CONTROL_MODE, "floating")
        print("Cleared torques and set control mode to 'floating'.")
    redis_client.set(KEY_CONTROL_TAU, encode_matlab(np.zeros((ab.dof,))))

    # Print simulation stats
    print("Simulated {}s in {}s.".format(timer.time_sim(), timer.time_elapsed()))

def initialize_web_app(redis_pipe, ab, path_urdf):
    # Register the urdf path so the server knows it's safe to fulfill requests for files in that directory.
    path_urdf = os.path.realpath(os.path.join(os.path.dirname(__file__), path_urdf))
    redis_pipe.hset(KEY_WEB_RESOURCES, NAME_APP, os.path.dirname(path_urdf))

    # Register key prefixes so the web app knows which models and objects to render.
    web_keys = {}
    web_keys["key_models_prefix"]  = KEY_MODELS_PREFIX
    web_keys["key_objects_prefix"] = KEY_OBJECTS_PREFIX
    redis_pipe.set(KEY_WEB_ARGS, json.dumps(web_keys))

    # Register the robot
    web_model = {}
    web_model["model"]    = json.loads(str(ab))
    web_model["key_q"]    = KEY_SENSOR_Q
    web_model["key_traj"] = KEY_TRAJ_POS
    redis_pipe.set(KEY_MODELS_PREFIX + ab.name, json.dumps(web_model))

    # Create a sphere marker for x_des
    web_object = {}
    x_des_marker = spatialdyn.Graphics("x_des_marker")
    x_des_marker.geometry.type = "sphere"
    x_des_marker.geometry.radius = 0.01
    web_object["graphics"] = [ json.loads(str(x_des_marker)) ]
    web_object["key_pos"]  = KEY_CONTROL_POS
    redis_pipe.set(KEY_OBJECTS_PREFIX + x_des_marker.name, json.dumps(web_object))

def compute_external_forces(ab, interaction):
    f_ext = {}

    # Check if the clicked object is the robot
    key_object = interaction["key_object"]
    if key_object != KEY_MODELS_PREFIX + ab.name:
        return f_ext

    # Extract the json fields
    idx_link = interaction["idx_link"]
    pos_mouse = np.array(interaction["pos_mouse_in_world"])
    pos_click = np.array(interaction["pos_click_in_link"])

    # Get the click position in world coordinates
    pos_click_in_world = spatialdyn.position(ab, idx_link, pos_click)

    # Set the click force
    f = GAIN_CLICK_DRAG * (pos_mouse - pos_click_in_world)
    f_click = np.hstack((f, np.zeros((3,))))

    # Translate the spatial force to the world frame
    f_ext[idx_link] = eigen.Translation3d(pos_click_in_world).fdot(f_click)

    return f_ext

def adjust_position(key, pos):
    if not key:
        return

    idx = 0
    sign = 1
    if key[0] == 'a':
        idx = 0
        sign = -1
    elif key[0] == 'd':
        idx = 0
        sign = 1
    elif key[0] == 'w':
        idx = 1
        sign = 1
    elif key[0] == 's':
        idx = 1
        sign = -1
    elif key[0] == 'e':
        idx = 2
        sign = 1
    elif key[0] == 'q':
        idx = 2
        sign = -1
    else:
        return

    pos[idx] += sign * GAIN_KEY_PRESS_POS

def adjust_orientation(key, quat):
    if not key:
        return

    idx = 0
    sign = 1
    if key[0] == 'j':
        idx = 0
        sign = -1
    elif key[0] == 'l':
        idx = 0
        sign = 1
    elif key[0] == 'i':
        idx = 1
        sign = 1
    elif key[0] == 'k':
        idx = 1
        sign = -1
    elif key[0] == 'o':
        idx = 2
        sign = 1
    elif key[0] == 'u':
        idx = 2
        sign = -1
    else:
        return

    axis = np.zeros((3,))
    axis[idx] = 1
    aa = eigen.AngleAxisd(sign * GAIN_KEY_PRESS_ORI, axis)
    quat.set(spatialdyn.opspace.near_quaternion((aa * quat).normalized(), quat))

if __name__ == "__main__":
    main()

