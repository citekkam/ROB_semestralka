#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# This code is based on the original code from the CTU project available at
# https://github.com/cvut/pyrocon/blob/master/CRS_commander.py under following
# copyright:
#    Copyright (c) 2017 Olga Petrova <olga.petrova@cvut.cz>
#    Advisor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
#    FEE CTU Prague, Czech Republic
#
#    Permission is hereby granted, free of charge, to any person obtaining a copy
#    of this software and associated documentation files (the "Software"), to deal
#    in the Software without restriction, including without limitation the rights
#    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#    copies of the Software, and to permit persons to whom the Software is
#    furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in all
#    copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#    SOFTWARE.
#    In 2017, project funded by PiKRON s.r.o. http://www.pikron.com/

from __future__ import annotations

import time

import numpy as np
from numpy.typing import ArrayLike
from serial import Serial, EIGHTBITS, PARITY_NONE, STOPBITS_ONE


class MarsControlUnit:
    def __init__(self, tty_dev: str = "/dev/ttyUSB0", baudrate: int = 19200):
        super().__init__()

        self._stamp = int(time.time() % 0x7FFF)
        self._coordmv_commands_to_next_check = 0
        self._connection = Serial(
            tty_dev,
            baudrate=baudrate,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            rtscts=True,
            timeout=0.01,
        )
        self.init_communication()

        self._coordinate_movement_set_up = False

    def __del__(self):
        """Close the connection to the robot's control unit."""
        self.close_connection()

    def close_connection(self):
        if hasattr(self, "_connection") and self._connection is not None:
            self._connection.close()

    def send_cmd(self, cmd: str):
        """Send command to the control unit via self._connection."""
        self._connection.write(bytearray(cmd, "ascii"))

    def read_response(self, maxbytes: int = 1024, decode=True) -> str | None:
        """Read response from control unit.
        :arg maxbytes: Max number of bytes to read.
        :param decode: Boolean, whether to decode the response to ascii.
        :return Response from control unit or None if no response in a short time.
        """
        resp = self._connection.read(maxbytes)
        if resp is None:
            return None
        if not decode:
            return resp
        return resp.decode("ascii").replace("\r\n", "\n").replace("\r", "\n")

    def sync_cmd_fifo(self):
        """Internal command used to synchronize message queue."""
        self._stamp = (self._stamp + 1) & 0x7FFF
        self.send_cmd("STAMP:%d\n" % self._stamp)
        buf = "\n"
        while True:
            buf += self.read_response()
            i = buf.find("\n" + "STAMP=")
            if i < 0:
                continue
            s = "%d" % self._stamp
            r = buf[i + 7 :]
            j = r.find("\n")
            if j < 0:
                continue
            r = r[0:j].strip()
            if r == s:
                break

    def query(self, query: str) -> str:
        """Send query to control unit and return the response.
        :arg query: Query to send.
        :return Control unit's response.
        """
        buf = "\n"
        self.send_cmd("\n" + query + "?\n")
        while True:
            buf += self.read_response()
            i = buf.find("\n" + query + "=")
            if i < 0:
                continue
            j = buf[i + 1 :].find("\n")
            if j != -1:
                if buf[i + 1 + j - 1] == "\r":
                    j -= 1
                res = buf[i + 2 + len(query) : i + 1 + j]
                return res
        return ""

    def init_communication(self, print_firmware_version: bool = True):
        """Initialize communication through the serial interface."""
        # The control unit can send some nonsense data on the serial line after power up
        # therefore we are nto decoding the first response.
        self.read_response(decode=False)
        self.send_cmd("\nECHO:0\n")
        self.sync_cmd_fifo()
        s = self.query("VER")
        if print_firmware_version:
            print(f"Firmware version : {s}")

    def wait_ready(self, sync=False):
        """Wait for control unit to be ready.
        :param sync: Boolean, whether to synchronize with control unit.
        """
        buf = "\n"
        if sync:
            self.sync_cmd_fifo()
        self.send_cmd("\nR:\n")
        while True:
            buf += self.read_response()
            if buf.find("\nR!") >= 0:
                return True
            if buf.find("\nFAIL!") >= 0:
                return False

    def check_ready(self, for_coordmv_queue=False) -> bool:
        """Check robot is in "ready" state.
        :param for_coordmv_queue: Boolean, whether to check state for coordinate
            movement message queue.
        :return: Boolean, whether robot is ready or not. Raise exception on error.
        """
        a = int(self.query("ST"))
        s = ""
        if a & 0x8:
            s = "error, "
        if a & 0x10000:
            s += "arm power is off, "
        if a & 0x20000:
            s += "motion stop, "
        if s:
            raise Exception("Check ready: %s." % s[:-2])
        if for_coordmv_queue:
            return False if a & 0x80 else True
        else:
            return False if a & 0x10 else True

    # def wait_gripper_ready(self):
    #     """
    #     Wait for gripper to be ready.
    #     """
    #     if not hasattr(self.robot, "gripper_ax"):
    #         raise Exception("This robot has no gripper_ax defined.")
    #
    #     self.send_cmd("\nR%s:\n" % self.robot.gripper_ax)
    #     self.rcon.timeout = 2
    #
    #     s = self.read_resp(1024)
    #     self.rcon.timeout = 0.01
    #     if s.find("R%s!\r\n" % self.robot.gripper_ax) >= 0:
    #         last = float("inf")
    #         while True:
    #             self.send_cmd("AP%s?\n" % self.robot.gripper_ax)
    #             s = self.read_resp(1024)
    #
    #             if s.find("\nFAIL!") >= 0:
    #                 raise Exception("Command 'AP' returned 'FAIL!\n")
    #             ifs = s.find("AP%s=" % self.robot.gripper_ax)
    #             if ifs >= 0:
    #                 ifl = s.find("\r\n")
    #                 p = float(s[ifs + 4 : ifl])
    #             if abs(last - p) < self.robot.gripper_poll_diff:
    #                 break
    #             last = p
    #         time.sleep(self.robot.gripper_poll_time / 100)
    #
    #     elif s.find("\nFAIL!") >= 0:
    #         self.wait_ready()
    #         raise Exception("Command 'R:%s' returned 'FAIL!'" % self.robot.gripper_ax)

    def setup_coordmv(self, axes_list: str):
        """Setup coordinate movement of joints defined in axes_list.
        :param axes_list: List of axes that will be controlled in coordinated movement.
        """
        self.wait_ready()
        axes_coma_list = ",".join(axes_list)
        self.send_cmd("COORDGRP:" + axes_coma_list + "\n")
        self.wait_ready()
        self._coordinate_movement_set_up = True

    def throttle_coordmv(self):
        """Throttle message queue for coordinate movement.
        :return: Boolean whether the queue is throttled, None if not checked."""
        if self._coordmv_commands_to_next_check <= 0:
            self._coordmv_commands_to_next_check = 20
        else:
            self._coordmv_commands_to_next_check -= 1
            return
        throttled = False
        while not self.check_ready(for_coordmv_queue=True):
            if not throttled:
                print("coordmv queue full - waiting")
            throttled = True
        return throttled

    def coordmv(self, q_irc: ArrayLike, min_time: float | None = None, disc: int = 5):
        """
        Coordinate movement of joints from current joint configuration to :param q_irc.
        :param q_irc: Joint configuration measured in IRC to move to.
        :param min_time: Minimal time [s] for the movement, if None movement is carried
          in minimal possible time.
        :param disc: Discontinuity of movement, internal parameter, is to be found in
          control unit docs.
        """
        if not self._coordinate_movement_set_up:
            print("Coordinate movement not set up. Call setup_coordmv() first.")
            return
        self.throttle_coordmv()
        self.send_cmd(f"COORDISCONT:{disc}\n")
        cmd = "COORDMV:" if min_time is None else "COORDMVT:"
        if min_time is not None:
            cmd += str(int(round(min_time * 1000)))
            if len(q_irc) > 0:
                cmd += ","
        cmd += ",".join([str(int(round(qv))) for qv in q_irc])
        self.send_cmd(cmd + "\n")

    def get_current_q_irc(self) -> np.ndarray:
        """Get current joint configuration in IRC units."""
        resp = self.query("COORDAP")
        try:
            resp = np.fromiter(map(int, resp.split(",")), int)
        except Exception:
            raise Exception("Error response %s" % resp)
        q_irc = resp[3:]
        return q_irc
