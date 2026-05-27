#!/usr/bin/env python3
"""
Simple FSM Node - 6 states, keyboard-driven (Manual + Auto unified)
States: IDLE / DRIVING / STOPPED / AVOIDING / EMERGENCY / TURNING_LEFT
"""

import sys
import termios
import tty
import select
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class State(Enum):
    IDLE = 'IDLE'
    DRIVING = 'DRIVING'
    STOPPED = 'STOPPED'
    AVOIDING = 'AVOIDING'
    EMERGENCY = 'EMERGENCY'
    TURNING_LEFT = 'TURNING_LEFT'


KEY_HELP = """
============= Simple FSM Keyboard =============
[Manual drive] DRIVING 상태에서만 작동
  w : forward    s : backward
  a : left turn  d : right turn
  x : stop

[State transitions]
  1 : IDLE
  2 : DRIVING (manual mode)
  3 : STOPPED (auto -> DRIVING after 3s)
  4 : AVOIDING (auto left-avoid 2s)
  5 : EMERGENCY (latched; 'r' to release)
  6 : TURNING_LEFT (timed left turn 2s)

  space : instant EMERGENCY
  r     : release EMERGENCY -> IDLE
  h     : help
  q     : quit
================================================
"""

# Speed params
LIN_SPEED = 0.2
ANG_SPEED = 0.5
AVOID_LIN = 0.1
AVOID_ANG = 0.5
TURN_LIN  = 0.1
TURN_ANG  = 0.7

# Timers (sec)
STOPPED_DURATION = 3.0
AVOID_DURATION   = 2.0
TURN_DURATION    = 2.0


class SimpleFSM(Node):
    def __init__(self):
        super().__init__('simple_fsm')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10Hz

        self.state = State.IDLE
        self.state_enter_time = time.time()

        # manual command cache (used only in DRIVING)
        self.manual_lin = 0.0
        self.manual_ang = 0.0

        self.running = True
        self.key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(KEY_HELP)
        self.get_logger().info(f'[FSM] start in {self.state.value}')

    def transition(self, new_state):
        if self.state == State.EMERGENCY and new_state != State.IDLE:
            self.get_logger().warn('[FSM] EMERGENCY latched. press "r" to release.')
            return
        old = self.state.value
        self.state = new_state
        self.state_enter_time = time.time()
        if new_state == State.DRIVING:
            self.manual_lin = 0.0
            self.manual_ang = 0.0
        self.get_logger().info(f'[FSM] {old} -> {new_state.value}')

    def handle_key(self, key):
        if key == '1':
            self.transition(State.IDLE)
        elif key == '2':
            self.transition(State.DRIVING)
        elif key == '3':
            self.transition(State.STOPPED)
        elif key == '4':
            self.transition(State.AVOIDING)
        elif key == '5':
            self.transition(State.EMERGENCY)
        elif key == '6':
            self.transition(State.TURNING_LEFT)
        elif key == ' ':
            self.state = State.EMERGENCY
            self.state_enter_time = time.time()
            self.get_logger().warn('[FSM] EMERGENCY (space)')
        elif key == 'r':
            if self.state == State.EMERGENCY:
                self.state = State.IDLE
                self.state_enter_time = time.time()
                self.get_logger().info('[FSM] EMERGENCY -> IDLE')
            else:
                self.get_logger().info('[FSM] "r" only releases EMERGENCY')
        elif key == 'h':
            self.get_logger().info(KEY_HELP)
        elif key == 'q':
            self.get_logger().info('[FSM] quit')
            self.running = False
            try:
                rclpy.shutdown()
            except Exception:
                pass
        elif key in ('w', 'a', 's', 'd', 'x'):
            if self.state != State.DRIVING:
                self.get_logger().info(
                    f'[FSM] "{key}" ignored (state={self.state.value}, need DRIVING)')
                return
            if key == 'w':
                self.manual_lin, self.manual_ang =  LIN_SPEED, 0.0
            elif key == 's':
                self.manual_lin, self.manual_ang = -LIN_SPEED, 0.0
            elif key == 'a':
                self.manual_lin, self.manual_ang = 0.0,  ANG_SPEED
            elif key == 'd':
                self.manual_lin, self.manual_ang = 0.0, -ANG_SPEED
            elif key == 'x':
                self.manual_lin, self.manual_ang = 0.0, 0.0

    def compute_cmd(self):
        lin, ang = 0.0, 0.0
        elapsed = time.time() - self.state_enter_time

        if self.state == State.IDLE:
            pass
        elif self.state == State.DRIVING:
            lin, ang = self.manual_lin, self.manual_ang
        elif self.state == State.STOPPED:
            if elapsed >= STOPPED_DURATION:
                self.transition(State.DRIVING)
        elif self.state == State.AVOIDING:
            if elapsed < AVOID_DURATION:
                lin, ang = AVOID_LIN, AVOID_ANG
            else:
                self.transition(State.DRIVING)
        elif self.state == State.EMERGENCY:
            pass
        elif self.state == State.TURNING_LEFT:
            if elapsed < TURN_DURATION:
                lin, ang = TURN_LIN, TURN_ANG
            else:
                self.transition(State.DRIVING)

        return lin, ang

    def tick(self):
        lin, ang = self.compute_cmd()
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub_cmd.publish(msg)

    def key_loop(self):
        fd = sys.stdin.fileno()
        try:
            old_term = termios.tcgetattr(fd)
        except termios.error:
            self.get_logger().error('[FSM] stdin is not a TTY. '
                                    'docker run에 -it 옵션 필요')
            return
        try:
            # raw 모드 대신 ICANON/ECHO만 끄기 (output \n->\r\n 변환 유지)
            new_term = termios.tcgetattr(fd)
            new_term[3] = new_term[3] & ~(termios.ICANON | termios.ECHO)
            termios.tcsetattr(fd, termios.TCSANOW, new_term)
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
