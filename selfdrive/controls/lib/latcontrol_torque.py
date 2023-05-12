import math

from cereal import log
from collections import deque
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, sign
from common.params import Params
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NNFF = [13, 5, 0, 0]

LAT_PLAN_MIN_IDX = 5

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki, k_d=0.03,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.error_downscale = 4.0
    self.error_scale_factor = FirstOrderFilter(1.0, 2.5, 0.01)
    self.use_nn = CI.initialize_ff_nn(CP.carFingerprint)
    if self.use_nn:
      self.torque_from_nn = CI.get_ff_nn
      # NNFF model takes current v_ego, a_ego, lat_accel, lat_jerk, roll, and past/desired data
      # of lat accel, lat jerk, and roll
      # Past value is computed using observed car lat accel, jerk, and roll
      self.nnff_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.75]
      self.nnff_future_times = [i + self.nnff_time_offset for i in future_times]
      self.nnff_lat_accels_filtered = [FirstOrderFilter(0.0, 0.0, 0.01) for i in [0.0] + future_times]
      self.nnff_lat_jerks_filtered = [FirstOrderFilter(0.0, 0.0, 0.01) for i in [0.0] + future_times]
      history_frames = 30
      self.lat_accel_deque = deque(maxlen=history_frames)
      self.lat_jerk_deque = deque(maxlen=history_frames)
      self.roll_deque = deque(maxlen=history_frames)
      self.nnff_alpha_up_down = [0.3, 0.15] # for increasing/decreasing magnitude of lat accel/jerk
      # Scale down desired lateral acceleration under moderate curvature to prevent cutting corners.

    self.param_s = Params()
    self.custom_torque = self.param_s.get_bool("CustomTorqueLateral")
    self._frame = 0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update_live_tune(self):
    if not self.custom_torque:
      return
    self._frame += 1
    if self._frame % 300 == 0:
      self.torque_params.latAccelFactor = float(self.param_s.get("TorqueMaxLatAccel", encoding="utf8")) * 0.01
      self.torque_params.friction = float(self.param_s.get("TorqueFriction", encoding="utf8")) * 0.01
      self._frame = 0

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, lat_plan=None, model_data=None):
    self.update_live_tune()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
        actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y_NNFF if self.use_nn else LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                     lateral_accel_deadzone, friction_compensation=False)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                     lateral_accel_deadzone, friction_compensation=False)
      
      # error downscaling before entering curves
      max_future_lateral_accel = max([abs(i) * CS.vEgo**2 for i in list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:16]] + [abs(desired_lateral_accel)])
      error_scale_factor = 1.0 / (1.0 + min(apply_deadzone(max_future_lateral_accel, 0.5) * self.error_downscale, self.error_downscale - 1))
      if error_scale_factor < self.error_scale_factor.x:
        self.error_scale_factor.x = error_scale_factor
      else:
        self.error_scale_factor.update(error_scale_factor)
      error_rate=((desired_lateral_jerk - actual_lateral_jerk) * self.error_scale_factor.x) if self.use_steering_angle else 0.0
      error = torque_from_setpoint - torque_from_measurement
      error *= self.error_scale_factor.x
      pid_log.error = error
      
      if self.use_nn:
        # prepare input data for NNFF model        
        future_speeds = [CS.vEgo] + [math.sqrt(interp(t, T_IDXS, model_data.velocity.x)**2 \
                                            + interp(t, T_IDXS, model_data.velocity.y)**2) \
                                              for t in self.nnff_future_times]
        future_curvatures = [desired_curvature] + [interp(t, T_IDXS, lat_plan.curvatures) for t in self.nnff_future_times]
        future_curvature_rates = [desired_curvature_rate] + [interp(t, T_IDXS, lat_plan.curvatureRates) for t in self.nnff_future_times]
        alpha = self.nnff_alpha_up_down[0 if abs(desired_lateral_accel) > abs(self.nnff_lat_accels_filtered[0].x) else 1]
        for i,(k, dk, v) in enumerate(zip(future_curvatures, future_curvature_rates, future_speeds)):
          self.nnff_lat_accels_filtered[i].update_alpha(alpha)
          self.nnff_lat_jerks_filtered[i].update_alpha(alpha/2)
          self.nnff_lat_accels_filtered[i].update(k * v**2)
          self.nnff_lat_jerks_filtered[i].update(dk * v**2)
        
        lat_accels_filtered = [i.x for i in self.nnff_lat_accels_filtered]
        lat_jerks_filtered = [i.x for i in self.nnff_lat_jerks_filtered]
        roll = params.roll
        future_rolls = [interp(t, T_IDXS, model_data.orientation.x) for t in self.nnff_future_times]
        
        self.lat_accel_deque.append(lat_accels_filtered[0])
        self.lat_jerk_deque.append(lat_jerks_filtered[0])
        self.roll_deque.append(roll)
          
        friction = self.torque_from_lateral_accel(0.0, self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel,
                                          lateral_accel_deadzone, friction_compensation=True)
        friction *= self.error_scale_factor.x
        
        nnff_input = [CS.vEgo, lat_accels_filtered[0], lat_jerks_filtered[0], roll] \
                    + [self.lat_accel_deque[0]] + lat_accels_filtered[1:] \
                    + [self.lat_jerk_deque[0]] + lat_jerks_filtered[1:] \
                    + [self.roll_deque[0]] + future_rolls
        ff = self.torque_from_nn(nnff_input)
        ff += friction * self.error_scale_factor.x
      else:
        ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel,
                                          lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      error_rate=error_rate,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      if self.use_nn:
        pid_log.nnffInput = nnff_input
      pid_log.errorScaleFactor = self.error_scale_factor.x
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
