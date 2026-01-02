"""UR5e Driver - Hardware interface using ur_rtde.

Simulation: Use MakeHardwareStation with !InverseDynamicsDriver (no custom code needed)
Hardware: Use UR5eDriver class directly

Control modes:
- "velocity": speedJ (joint velocities)
- "position": servoJ (joint positions)
"""
import numpy as np
from pathlib import Path
from pydrake.all import LeafSystem

UR5E_PACKAGE_XML = str(Path(__file__).parent.parent / 'models' / 'package.xml')

try:
    from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
    _UR_RTDE_AVAILABLE = True
except ImportError:
    _UR_RTDE_AVAILABLE = False


class UR5eDriver(LeafSystem):
    """Hardware driver for UR5e using ur_rtde.

    Ports (match simulation for drop-in replacement):
        Input:  ur5e.desired_state (12) = [q(6), v(6)]
        Output: ur5e.state_estimated (12) = [q(6), v(6)]
        Output: ur5e.torque_measured (6) = joint torques
        Output: ur5e.tcp_force (6) = TCP force/torque

    Control modes:
        "velocity": Uses speedJ with velocity part of desired_state
        "position": Uses servoJ with position part of desired_state
    """

    def __init__(self, robot_ip, control_mode="velocity", dt=0.002):
        """
        Args:
            robot_ip: IP address of the UR5e
            control_mode: "velocity" (speedJ) or "position" (servoJ)
            dt: Control loop period in seconds (default 0.002 = 500Hz)
        """
        super().__init__()
        if not _UR_RTDE_AVAILABLE:
            raise ImportError("ur_rtde not installed. Run: pip install ur-rtde")

        self._control_mode = control_mode
        self._dt = dt
        self._rtde_c = RTDEControlInterface(robot_ip)
        self._rtde_r = RTDEReceiveInterface(robot_ip)

        # Input: desired state [q, v]
        self.DeclareVectorInputPort("ur5e.desired_state", 12)

        # Outputs: match simulation interface
        self.DeclareVectorOutputPort("ur5e.state_estimated", 12, self._CalcState)
        self.DeclareVectorOutputPort("ur5e.torque_measured", 6, self._CalcTorque)
        self.DeclareVectorOutputPort("ur5e.tcp_force", 6, self._CalcTCPForce)

        # Control loop
        self.DeclarePeriodicPublishEvent(dt, 0.0, self._SendCommand)

    def _CalcState(self, context, output):
        """Output: [q(6), v(6)] joint positions and velocities."""
        q = np.array(self._rtde_r.getActualQ())
        v = np.array(self._rtde_r.getActualQd())
        output.SetFromVector(np.concatenate([q, v]))

    def _CalcTorque(self, context, output):
        """Output: joint torques (6)."""
        tau = np.array(self._rtde_c.getJointTorques())
        output.SetFromVector(tau)

    def _CalcTCPForce(self, context, output):
        """Output: TCP force/torque (6) - useful for NetFT probe."""
        ft = np.array(self._rtde_r.getActualTCPForce())
        output.SetFromVector(ft)

    def _SendCommand(self, context):
        """Send command based on control mode."""
        state = self.get_input_port(0).Eval(context)
        q_des = state[:6]
        v_des = state[6:]

        if self._control_mode == "velocity":
            # speedJ: command joint velocities
            self._rtde_c.speedJ(v_des.tolist(), acceleration=0.5, time=self._dt)
        elif self._control_mode == "position":
            # servoJ: command joint positions
            self._rtde_c.servoJ(q_des.tolist(),
                               speed=0.5, acceleration=0.5,
                               time=self._dt,
                               lookahead_time=0.1, gain=300)
        else:
            raise ValueError(f"Unknown control_mode: {self._control_mode}")

    def stop(self):
        """Stop robot motion and disconnect."""
        if self._control_mode == "velocity":
            self._rtde_c.speedStop(0.5)
        else:
            self._rtde_c.servoStop(0.5)
        self._rtde_c.stopScript()

    # Direct access to ur_rtde for advanced use
    @property
    def rtde_control(self):
        """Direct access to RTDEControlInterface."""
        return self._rtde_c

    @property
    def rtde_receive(self):
        """Direct access to RTDEReceiveInterface."""
        return self._rtde_r
