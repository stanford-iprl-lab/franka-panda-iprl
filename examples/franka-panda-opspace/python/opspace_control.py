#!/usr/bin/env python

import argparse
import json
import signal
import sys
import time

import redis
import numpy as np

import spatialdyn
import frankapanda
from spatialdyn_frankapanda import *

g_runloop = True

def signal_handler(sig, frame):
    global g_runloop
    g_runloop = False

def encode_matlab(A):
    if len(A.shape) == 1:
        return " ".join(map(str, A.tolist()))
    return "; ".join(" ".join(map(str, row)) for row in A.tolist())

def decode_matlab(s):
    try:
        s = s.decode("utf-8")
    except AttributeError:
        pass
    return np.array([list(map(float, row.strip().split())) for row in s.strip().split(";")]).squeeze()


def main():
    # Parse args
    parser = argparse.ArgumentParser(description=(
        "Operational space controller for the Franka Panda."
    ))
    parser.add_argument("--sim", help="run in simulation", action="store_true")
    args = parser.parse_args()

    # Redis keys
    KEY_PREFIX = "franka_panda::"

    # GET keys
    KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q"
    KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq"
    KEY_INERTIA_EE    = KEY_PREFIX + "model::inertia_ee"
    KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status"

    # SET keys
    KEY_CONTROL_TAU  = KEY_PREFIX + "control::tau"
    KEY_CONTROL_MODE = KEY_PREFIX + "control::mode"
    KEY_TRAJ_POS     = KEY_PREFIX + "trajectory::pos"
    KEY_TRAJ_ORI     = KEY_PREFIX + "trajectory::ori"
    KEY_TRAJ_POS_ERR = KEY_PREFIX + "trajectory::pos_err"
    KEY_TRAJ_ORI_ERR = KEY_PREFIX + "trajectory::ori_err"

    # Controller gains
    KEY_KP_POS   = KEY_PREFIX + "control::kp_pos"
    KEY_KV_POS   = KEY_PREFIX + "control::kv_pos"
    KEY_KP_ORI   = KEY_PREFIX + "control::kp_ori"
    KEY_KV_ORI   = KEY_PREFIX + "control::kv_ori"
    KEY_KP_JOINT = KEY_PREFIX + "control::kp_joint"
    KEY_KV_JOINT = KEY_PREFIX + "control::kv_joint"

    # Create Redis client and timer
    redis_client = redis.Redis("127.0.0.1")
    redis_pipe   = redis_client.pipeline(transaction=False)
    timer = spatialdyn.Timer(1000)

    # Load robot
    ab = ArticulatedBody(spatialdyn.urdf.load_model("resources/franka_panda.urdf"))
    q_home = np.array([0., -np.pi/6., 0., -5./6. * np.pi, 0., 2./3. * np.pi, 0.])
    if args.sim:
        ab.q  = q_home
        ab.dq = np.zeros((ab.dof,))
        redis_client.set(KEY_SENSOR_Q, encode_matlab(ab.q))
        redis_client.set(KEY_SENSOR_DQ, encode_matlab(ab.q))
    else:
        ab.q  = decode_matlab(redis_client.get(KEY_SENSOR_Q))
        ab.dq = decode_matlab(redis_client.get(KEY_SENSOR_DQ))

    # Initialize parameters
    kp_pos   = 40
    kv_pos   = 5
    kp_ori   = 40
    kv_ori   = 5
    kp_joint = 5
    kv_joint = 0

    ee_offset = np.array([0., 0., 0.107])
    q_des     = np.array(q_home)
    x_0       = spatialdyn.position(ab, offset=ee_offset)
    quat_des  = spatialdyn.orientation(ab)

    # ab.inertia_compensation = np.array([0.2, 0.1, 0.1])
    # ab.stiction_coefficients = np.array([0.8, 0.8, 0.6])
    # ab.stiction_activations = np.array([0.1, 0.1, 0.1])

    # Initialize gains in Redis
    redis_client.set(KEY_KP_POS, kp_pos)
    redis_client.set(KEY_KV_POS, kv_pos)
    redis_client.set(KEY_KP_ORI, kp_ori)
    redis_client.set(KEY_KV_ORI, kv_ori)
    redis_client.set(KEY_KP_JOINT, kp_joint)
    redis_client.set(KEY_KV_JOINT, kv_joint)

    # Get end-effector inertia from driver
    redis_ee = redis_client.get(KEY_INERTIA_EE)
    if redis_ee is not None:
        json_ee  = json.loads(redis_ee.decode("utf8"))
        m_ee     = float(json_ee["m"])
        com_ee   = np.array(list(map(float, json_ee["com"])))
        I_com_ee = np.array(list(map(float, json_ee["I_com"])))
        I_load   = spatialdyn.SpatialInertiad(m_ee, com_ee, I_com_ee)
        ab.replace_load(I_load)

    # Create signal handler
    global g_runloop
    signal.signal(signal.SIGINT, signal_handler)

    is_initialized = False  # Flag to set control mode on first iteration

    try:
        while g_runloop:
            # Wait for next loop
            timer.sleep()

            # Break if the driver is not running
            if not args.sim and redis_client.get(KEY_DRIVER_STATUS).decode("utf8") != "running":
                break

            # Update gains
            redis_pipe.get(KEY_KP_POS).get(KEY_KV_POS).get(KEY_KP_ORI).get(KEY_KV_ORI).get(KEY_KP_JOINT).get(KEY_KV_JOINT)
            kp_pos, kv_pos, kp_ori, kv_ori, kp_joint, kv_joint = tuple(map(float, redis_pipe.execute()))

            # Update robot state
            if not args.sim:
                ab.q  = decode_matlab(redis_client.get(KEY_SENSOR_Q))
                ab.dq = decode_matlab(redis_client.get(KEY_SENSOR_DQ))

            # Position
            x_des = np.array(x_0)
            x_des[1] += 0.2 * np.sin(timer.time_sim())
            x_des[0] += 0.2 * (np.cos(timer.time_sim()) - 1)
            x = spatialdyn.position(ab, offset=ee_offset)
            x_err = x - x_des
            dx_err = spatialdyn.linear_jacobian(ab, offset=ee_offset).dot(ab.dq)
            ddx = -kp_pos * x_err - kv_pos * dx_err

            # Orientation
            quat = spatialdyn.orientation(ab)
            ori_err = spatialdyn.opspace.orientation_error(quat, quat_des)
            w_err = spatialdyn.angular_jacobian(ab).dot(ab.dq)
            dw = -kp_ori * ori_err - kv_ori * w_err

            # Combine position/orientation
            ddx_dw = np.hstack((ddx, dw))
            J = spatialdyn.jacobian(ab, offset=ee_offset)
            N = np.eye(ab.dof)
            tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J, ddx_dw, N)

            # Nullspace
            I = np.eye(ab.dof)
            q_err = ab.q - q_des
            dq_err = ab.dq
            ddq = -kp_joint * q_err - kv_joint * dq_err
            tau_cmd += spatialdyn.opspace.inverse_dynamics(ab, I, ddq, N)

            # Add friction compensation
            tau_cmd += friction(ab, tau_cmd)

            # Add gravity compensation
            tau_cmd += spatialdyn.gravity(ab)

            # Send control torques
            redis_client.set(KEY_CONTROL_TAU, encode_matlab(tau_cmd))

            if args.sim:
                # Integrate
                spatialdyn.integrate(ab, tau_cmd, 0.001, friction=True)
                redis_client.set(KEY_SENSOR_Q, encode_matlab(ab.q))
                redis_client.set(KEY_SENSOR_DQ, encode_matlab(ab.dq))
            elif not is_initialized:
                # Send control mode on first iteration
                redis_client.set(KEY_CONTROL_MODE, "torque")
                is_initialized = True

            # Send trajectory info for visualizer
            redis_client.set(KEY_TRAJ_POS, encode_matlab(x))
            redis_client.set(KEY_TRAJ_ORI, encode_matlab(quat.coeffs))
            redis_client.set(KEY_TRAJ_POS_ERR, encode_matlab(x_err))
            redis_client.set(KEY_TRAJ_ORI_ERR, encode_matlab(ori_err))
    except Exception as e:
        print(e)

    # Clear torques
    redis_client.set(KEY_CONTROL_TAU, encode_matlab(np.zeros((ab.dof,))))
    if not args.sim:
        redis_client.set(KEY_CONTROL_MODE, "floating")
        print("Cleared torques and set control mode to 'floating'.")

    # Print simulation stats
    print("Simulated {}s in {}s.".format(timer.time_sim(), timer.time_elapsed()))

if __name__ == "__main__":
    main()
