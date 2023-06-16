#!/usr/bin/env python3
import os
import sys
import signal
import numpy as np
from collections import deque, defaultdict

import cereal.messaging as messaging
from cereal import car, log
from common.params import Params
from common.realtime import config_realtime_process, DT_MDL
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import sign, mean, clip
from system.swaglog import cloudlog
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

HISTORY = 5  # secs
POINTS_PER_BUCKET = 1500
MIN_POINTS_TOTAL = 4000
MIN_POINTS_TOTAL_QLOG = 600
FIT_POINTS_TOTAL = 2000
FIT_POINTS_TOTAL_QLOG = 600
MIN_VEL = 15  # m/s
FRICTION_FACTOR = 1.5  # ~85% of data coverage
FACTOR_SANITY = 0.3
FACTOR_SANITY_QLOG = 0.5
FRICTION_SANITY = 0.5
FRICTION_SANITY_QLOG = 0.8
STEER_MIN_THRESHOLD = 0.02
MIN_FILTER_DECAY = 50
MAX_FILTER_DECAY = 250
LAT_ACC_THRESHOLD = 1
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = np.array([100, 300, 500, 500, 500, 500, 300, 100])
MIN_ENGAGE_BUFFER = 2  # secs
MIN_ENGAGE_LEN = int(MIN_ENGAGE_BUFFER / DT_MDL)

VERSION = 1  # bump this to invalidate old parameter caches
ALLOWED_CARS = ['toyota', 'hyundai']


# LiveKF is an autotuned split left/right kf value based on lateral acceleration
# error when lateral jerk is near zero.
# Bins are used so that the amount of change in the kf value is proportional to
# the completeness of the data.
class LiveKF:
  def __init__(self, CP):
    self.hist_len = int((MIN_ENGAGE_BUFFER + 2) / DT_MDL)
    self.lag = CP.steerActuatorDelay + .2   # from controlsd
    self.total_num_points = 0
    self.weighted_mean_lat_accel_ratio = 1.0
    self.kf_shift = 0.0
    self.bin_size = 500
    self.speed_min = 5.0
    self.speed_max = 35.0
    self.speed_num_bins = 10
    speed_step = (self.speed_max - self.speed_min) / self.speed_num_bins
    self.speed_bin_limits = [(float(x), float(x + speed_step)) for x in np.linspace(self.speed_min, self.speed_max, self.speed_num_bins)]
    self.speed_num_bins = len(self.speed_bin_limits)
    
    self.abs_lat_accel_min = 0.5
    abs_lat_accel_max = 3.0
    self.lat_accel_num_bins = 5
    lat_accel_step = (abs_lat_accel_max - self.abs_lat_accel_min) / self.lat_accel_num_bins
    self.lat_accel_bin_limits = [(float(x), float(x + lat_accel_step)) for x in np.linspace(self.abs_lat_accel_min, abs_lat_accel_max, self.lat_accel_num_bins)]
    self.lat_accel_num_bins = len(self.lat_accel_bin_limits)
    
    num_bins = self.speed_num_bins * self.lat_accel_num_bins
    
    self.max_num_points = self.speed_num_bins * self.lat_accel_num_bins * self.bin_size
    self.min_points_total = self.max_num_points / num_bins # one full bin worth
    
    self.abs_lat_jerk_max = 0.25
    self.abs_lat_jerk_check_pts = min(3, int(round(CP.steerActuatorDelay / DT_MDL)))
    self.abs_lat_jerk_alpha = 0.2
    self.lat_jerk_filtered = FirstOrderFilter(0.0, self.abs_lat_jerk_alpha, DT_MDL)
    self.lat_jerk_history = deque(maxlen=self.abs_lat_jerk_check_pts)
    
    self.kf_limits = (0.5, 2.0)
    self.kf_alpha = 50.0
    if CP.lateralTuning.which() == 'torque':
      self.offline_kf = CP.lateralTuning.torque.kf
    elif CP.lateralTuning.which() == 'pid':
      self.offline_kf = CP.lateralTuning.pid.kf
    else:
      raise ValueError('lateralTuning must be torque or pid')
    self.offline_kf = clip(self.offline_kf, self.kf_limits[0], self.kf_limits[1])
    self.kf = FirstOrderFilter(self.offline_kf, self.kf_alpha, DT_MDL)
    
    self.reset()
    
  def reset(self):
    # Bins aren't actually error, but rather the quotient of desired / actual lat accel,
    # so it's like a ratio that describes error
    # call like self.lat_accel_ratio_bins[speed_bin][lat_accel_bin]
    self.lat_accel_ratio_bins = [[deque(maxlen=self.bin_size) for _ in range(self.lat_accel_num_bins)] for _ in range(self.speed_num_bins)]
    self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
    self.lat_jerk_filtered.x = 0.0
    self.lat_jerk_history = deque(maxlen=self.abs_lat_jerk_check_pts)
    self.kf.x = self.offline_kf
    return
  
  def record_point(self, speed, desired_lateral_accel, actual_lateral_accel):
    if actual_lateral_accel == 0.0 or sign(desired_lateral_accel) != sign(actual_lateral_accel):
      return
    self.add_point(speed, desired_lateral_accel, actual_lateral_accel)
  
  def add_point(self, speed, desired_lateral_accel, actual_lateral_accel):
    # get speed and lat accel bins
    speed_bin = [i for i, (lower, upper) in enumerate(self.speed_bin_limits) if lower <= speed < upper]
    lat_accel_bin = [i for i, (lower, upper) in enumerate(self.lat_accel_bin_limits) if lower <= abs(desired_lateral_accel) < upper]
    if all([len(i) > 0 for i in [speed_bin, lat_accel_bin]]):
      self.lat_accel_ratio_bins[speed_bin[0]][lat_accel_bin[0]].append(desired_lateral_accel / actual_lateral_accel)
  
  def load_points(self, points):
    for speed, lat_accel, lat_accel_ratio in points:
      self.add_point(speed, lat_accel, lat_accel_ratio / lat_accel)

  def get_points(self):
    points = []
    for ispeed, (vl, vh) in enumerate(self.speed_bin_limits):
      v = (vl + vh) / 2
      for ilat_accel, (lal, lah) in enumerate(self.lat_accel_bin_limits):
        la = (lal + lah) / 2
        for lat_accel_ratio in self.lat_accel_ratio_bins[ispeed][ilat_accel]:
          points.append([v, la, lat_accel_ratio])
    return points

  def adjust_kf(self):
    # adjust kf using a weighted average of the binned desired:measured lat accel values,
    # where each bin's average is weighted in the total average by the number of points in the bin.
    self.total_num_points = sum(len(d) for sublist in self.lat_accel_ratio_bins for d in sublist)
    if self.total_num_points < self.min_points_total:
      return
    
    self.weighted_mean_lat_accel_ratio = 0.0
    for speed_bin in range(self.speed_num_bins):
      for lat_accel_bin in range(self.lat_accel_num_bins):
        num_points = len(self.lat_accel_ratio_bins[speed_bin][lat_accel_bin])
        if num_points > 0:
          self.weighted_mean_lat_accel_ratio += num_points / self.total_num_points * mean(self.lat_accel_ratio_bins[speed_bin][lat_accel_bin])
    
    # If kf was perfect, then the weighted mean lat accel ratio would be 1.0.
    # If the ratio is > 1, then kf needs to increase, or decrease if the ratio is < 1.
    self.kf_shift = (self.weighted_mean_lat_accel_ratio - 1.0) * self.total_num_points / self.max_num_points * DT_MDL
    kf_last = clip(self.kf.x, self.kf_limits[0], self.kf_limits[1])
    self.kf.update(self.kf.x + self.kf_shift)
    self.kf.x = clip(self.kf.x, self.kf_limits[0], self.kf_limits[1])
    
    # Here we assume that had the new kf been used, the old points
    # would have been closer to 1.0.
    # So go back and adjust the points in the bins, bringing them closer to 1.0
    # using the ratio to old to new kf.
    # If kf went up, then the ratios in the bins will be > 1 and they need to be browght down,
    # and vice versa if kf went down.
    # So we scale the points by the ratio of the old to new kf.
    kf_ratio = self.kf.x / kf_last
    for i in range(len(self.lat_accel_ratio_bins)):
      for j in range(len(self.lat_accel_ratio_bins[i])):
        for k in range(len(self.lat_accel_ratio_bins[i][j])):
          self.lat_accel_ratio_bins[i][j][k] += kf_ratio * (1.0 - self.lat_accel_ratio_bins[i][j][k])
  
  def handle_log(self, t, which, msg):
    if which == "carControl":
      self.raw_points["active"].append(msg.latActive)
    elif which == "controlsState":
      self.raw_points["desired_curvature"].append(msg.desiredCurvature)
      self.raw_points["desired_curvature_rate"].append(msg.desiredCurvatureRate)
      self.raw_points["actual_curvature"].append(msg.curvature)
    elif which == "carState":
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(msg.steeringPressed)
    elif which == "liveLocationKalman":
      if all([len(self.raw_points[i]) == self.hist_len for i in ["active","vego","desired_curvature"]]):
        active = all(list(self.raw_points["active"])[-MIN_ENGAGE_LEN:])
        steer_override = any(list(self.raw_points["steer_override"])[-MIN_ENGAGE_LEN:])
        vego = self.raw_points['vego'][-1]
        self.lat_jerk_filtered.update(self.raw_points["desired_curvature_rate"][-1] * vego**2)
        self.lat_jerk_history.append(self.lat_jerk_filtered.x)
        if active and not steer_override \
            and all([abs(x) <= self.abs_lat_jerk_max for x in self.lat_jerk_history]):
          desired_lateral_accel = self.raw_points['desired_curvature'][-1] * vego**2
          actual_lateral_accel = self.raw_points['actual_curvature'][-1] * vego**2
          self.record_point(float(vego), float(desired_lateral_accel), float(actual_lateral_accel))


def slope2rot(slope):
  sin = np.sqrt(slope**2 / (slope**2 + 1))
  cos = np.sqrt(1 / (slope**2 + 1))
  return np.array([[cos, -sin], [sin, cos]])


class NPQueue:
  def __init__(self, maxlen, rowsize):
    self.maxlen = maxlen
    self.arr = np.empty((0, rowsize))

  def __len__(self):
    return len(self.arr)

  def append(self, pt):
    if len(self.arr) < self.maxlen:
      self.arr = np.append(self.arr, [pt], axis=0)
    else:
      self.arr[:-1] = self.arr[1:]
      self.arr[-1] = pt


class PointBuckets:
  def __init__(self, x_bounds, min_points, min_points_total):
    self.x_bounds = x_bounds
    self.buckets = {bounds: NPQueue(maxlen=POINTS_PER_BUCKET, rowsize=3) for bounds in x_bounds}
    self.buckets_min_points = {bounds: min_point for bounds, min_point in zip(x_bounds, min_points)}
    self.min_points_total = min_points_total

  def bucket_lengths(self):
    return [len(v) for v in self.buckets.values()]

  def __len__(self):
    return sum(self.bucket_lengths())

  def is_valid(self):
    return all(len(v) >= min_pts for v, min_pts in zip(self.buckets.values(), self.buckets_min_points.values())) and (self.__len__() >= self.min_points_total)

  def add_point(self, x, y):
    for bound_min, bound_max in self.x_bounds:
      if (x >= bound_min) and (x < bound_max):
        self.buckets[(bound_min, bound_max)].append([x, 1.0, y])
        break

  def get_points(self, num_points=None):
    points = np.vstack([x.arr for x in self.buckets.values()])
    if num_points is None:
      return points
    return points[np.random.choice(np.arange(len(points)), min(len(points), num_points), replace=False)]

  def load_points(self, points):
    for x, y in points:
      self.add_point(x, y)


class TorqueEstimator:
  def __init__(self, CP, decimated=False):
    self.live_kf = LiveKF(CP)
    self.hist_len = int(HISTORY / DT_MDL)
    self.lag = CP.steerActuatorDelay + .2   # from controlsd
    if decimated:
      self.min_bucket_points = MIN_BUCKET_POINTS / 10
      self.min_points_total = MIN_POINTS_TOTAL_QLOG
      self.fit_points = FIT_POINTS_TOTAL_QLOG
      self.factor_sanity = FACTOR_SANITY_QLOG
      self.friction_sanity = FRICTION_SANITY_QLOG

    else:
      self.min_bucket_points = MIN_BUCKET_POINTS
      self.min_points_total = MIN_POINTS_TOTAL
      self.fit_points = FIT_POINTS_TOTAL
      self.factor_sanity = FACTOR_SANITY
      self.friction_sanity = FRICTION_SANITY

    self.offline_friction = 0.0
    self.offline_latAccelFactor = 0.0
    self.resets = 0.0
    self.use_params = CP.carName in ALLOWED_CARS

    if CP.lateralTuning.which() == 'torque':
      self.offline_friction = CP.lateralTuning.torque.friction
      self.offline_latAccelFactor = CP.lateralTuning.torque.latAccelFactor

    self.reset()

    initial_params = {
      'latAccelFactor': self.offline_latAccelFactor,
      'latAccelOffset': 0.0,
      'frictionCoefficient': self.offline_friction,
      'points': []
    }
    self.decay = MIN_FILTER_DECAY
    self.min_lataccel_factor = (1.0 - self.factor_sanity) * self.offline_latAccelFactor
    self.max_lataccel_factor = (1.0 + self.factor_sanity) * self.offline_latAccelFactor
    self.min_friction = (1.0 - self.friction_sanity) * self.offline_friction
    self.max_friction = (1.0 + self.friction_sanity) * self.offline_friction

    # try to restore cached params
    params = Params()
    params_cache = params.get("LiveTorqueCarParams")
    torque_cache = params.get("LiveTorqueParameters")
    if params_cache is not None and torque_cache is not None:
      try:
        cache_ltp = log.Event.from_bytes(torque_cache).liveTorqueParameters
        cache_CP = car.CarParams.from_bytes(params_cache)
        if self.get_restore_key(cache_CP, cache_ltp.version) == self.get_restore_key(CP, VERSION):
          if cache_ltp.liveValid:
            initial_params = {
              'latAccelFactor': cache_ltp.latAccelFactorFiltered,
              'latAccelOffset': cache_ltp.latAccelOffsetFiltered,
              'frictionCoefficient': cache_ltp.frictionCoefficientFiltered
            }
          initial_params['points'] = cache_ltp.points
          self.decay = cache_ltp.decay
          self.filtered_points.load_points(initial_params['points'])
          
          # restore live kf
          self.live_kf.load_points(cache_ltp.liveKfPoints)
          self.live_kf.kf.x = cache_ltp.liveKf
          
          cloudlog.info("restored torque params from cache")
      except Exception:
        cloudlog.exception("failed to restore cached torque params")
        params.remove("LiveTorqueCarParams")
        params.remove("LiveTorqueParameters")

    self.filtered_params = {}
    for param in initial_params:
      self.filtered_params[param] = FirstOrderFilter(initial_params[param], self.decay, DT_MDL)

  def get_restore_key(self, CP, version):
    a, b = None, None
    if CP.lateralTuning.which() == 'torque':
      a = CP.lateralTuning.torque.friction
      b = CP.lateralTuning.torque.latAccelFactor
    return (CP.carFingerprint, CP.lateralTuning.which(), a, b, version)

  def reset(self):
    self.resets += 1.0
    self.decay = MIN_FILTER_DECAY
    self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
    self.filtered_points = PointBuckets(x_bounds=STEER_BUCKET_BOUNDS, min_points=self.min_bucket_points, min_points_total=self.min_points_total)

  def estimate_params(self):
    points = self.filtered_points.get_points(self.fit_points)
    # total least square solution as both x and y are noisy observations
    # this is empirically the slope of the hysteresis parallelogram as opposed to the line through the diagonals
    try:
      _, _, v = np.linalg.svd(points, full_matrices=False)
      slope, offset = -v.T[0:2, 2] / v.T[2, 2]
      _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
      friction_coeff = np.std(spread) * FRICTION_FACTOR
    except np.linalg.LinAlgError as e:
      cloudlog.exception(f"Error computing live torque params: {e}")
      slope = offset = friction_coeff = np.nan
    return slope, offset, friction_coeff

  def update_params(self, params):
    self.decay = min(self.decay + DT_MDL, MAX_FILTER_DECAY)
    for param, value in params.items():
      self.filtered_params[param].update(value)
      self.filtered_params[param].update_alpha(self.decay)

  def handle_log(self, t, which, msg):
    self.live_kf.handle_log(t, which, msg)
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["steer_torque"].append(-msg.actuatorsOutput.steer)
      self.raw_points["active"].append(msg.latActive)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(msg.steeringPressed)
    elif which == "liveLocationKalman":
      if len(self.raw_points['steer_torque']) == self.hist_len:
        yaw_rate = msg.angularVelocityCalibrated.value[2]
        roll = msg.orientationNED.value[0]
        active = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL), self.raw_points['carControl_t'], self.raw_points['active']).astype(bool)
        steer_override = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL), self.raw_points['carState_t'], self.raw_points['steer_override']).astype(bool)
        vego = np.interp(t, self.raw_points['carState_t'], self.raw_points['vego'])
        steer = np.interp(t, self.raw_points['carControl_t'], self.raw_points['steer_torque'])
        lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)
        if all(active) and (not any(steer_override)) and (vego > MIN_VEL) and (abs(steer) > STEER_MIN_THRESHOLD) and (abs(lateral_acc) <= LAT_ACC_THRESHOLD):
          self.filtered_points.add_point(float(steer), float(lateral_acc))

  def get_msg(self, valid=True, with_points=False):
    self.live_kf.adjust_kf()
    msg = messaging.new_message('liveTorqueParameters')
    msg.valid = valid
    liveTorqueParameters = msg.liveTorqueParameters
    liveTorqueParameters.version = VERSION
    liveTorqueParameters.useParams = self.use_params

    if self.filtered_points.is_valid():
      latAccelFactor, latAccelOffset, frictionCoeff = self.estimate_params()
      liveTorqueParameters.latAccelFactorRaw = float(latAccelFactor)
      liveTorqueParameters.latAccelOffsetRaw = float(latAccelOffset)
      liveTorqueParameters.frictionCoefficientRaw = float(frictionCoeff)

      if any([val is None or np.isnan(val) for val in [latAccelFactor, latAccelOffset, frictionCoeff]]):
        cloudlog.exception("Live torque parameters are invalid.")
        liveTorqueParameters.liveValid = False
        self.reset()
      else:
        liveTorqueParameters.liveValid = True
        latAccelFactor = np.clip(latAccelFactor, self.min_lataccel_factor, self.max_lataccel_factor)
        frictionCoeff = np.clip(frictionCoeff, self.min_friction, self.max_friction)
        self.update_params({'latAccelFactor': latAccelFactor, 'latAccelOffset': latAccelOffset, 'frictionCoefficient': frictionCoeff})
    else:
      liveTorqueParameters.liveValid = False

    if with_points:
      liveTorqueParameters.points = self.filtered_points.get_points()[:, [0, 2]].tolist()

    liveTorqueParameters.latAccelFactorFiltered = float(self.filtered_params['latAccelFactor'].x)
    liveTorqueParameters.latAccelOffsetFiltered = float(self.filtered_params['latAccelOffset'].x)
    liveTorqueParameters.frictionCoefficientFiltered = float(self.filtered_params['frictionCoefficient'].x)
    liveTorqueParameters.totalBucketPoints = len(self.filtered_points)
    liveTorqueParameters.decay = self.decay
    liveTorqueParameters.maxResets = self.resets
    
    #liveKF
    liveTorqueParameters.liveKf = self.live_kf.kf.x
    liveTorqueParameters.liveKfLatAccelRatio = self.live_kf.weighted_mean_lat_accel_ratio
    liveTorqueParameters.liveKfShift = self.live_kf.kf_shift
    liveTorqueParameters.liveKfNumPoints = self.live_kf.total_num_points
    liveTorqueParameters.liveKfLatJerkFiltered = self.live_kf.lat_jerk_filtered.x
    if with_points:
      liveTorqueParameters.liveKfPoints = self.live_kf.get_points()
      raw_pts_len = [len(self.live_kf.raw_points[i]) for i in ["active","vego","desired_curvature"]]
      cloudlog.warning(f"LiveKF: kf = {self.live_kf.kf.x}, la ratio = {self.live_kf.weighted_mean_lat_accel_ratio}, #pts = {self.live_kf.total_num_points}, #raw pts = {raw_pts_len}, lat jerk = {self.live_kf.lat_jerk_filtered.x:.1f}")
    
    return msg


def main(sm=None, pm=None):
  config_realtime_process([0, 1, 2, 3], 5)

  if sm is None:
    sm = messaging.SubMaster(['carControl', 'carState', 'liveLocationKalman', 'controlsState'], poll=['liveLocationKalman'])

  if pm is None:
    pm = messaging.PubMaster(['liveTorqueParameters'])

  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  estimator = TorqueEstimator(CP)

  def cache_params(sig, frame):
    signal.signal(sig, signal.SIG_DFL)
    cloudlog.warning("caching torque params")

    params = Params()
    params.put("LiveTorqueCarParams", CP.as_builder().to_bytes())

    msg = estimator.get_msg(with_points=True)
    params.put("LiveTorqueParameters", msg.to_bytes())

    sys.exit(0)
  if "REPLAY" not in os.environ:
    signal.signal(signal.SIGINT, cache_params)

  while True:
    sm.update()
    if sm.all_checks():
      for which in sm.updated.keys():
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          estimator.handle_log(t, which, sm[which])

    # 4Hz driven by liveLocationKalman
    if sm.frame % 5 == 0:
      pm.send('liveTorqueParameters', estimator.get_msg(valid=sm.all_checks(), with_points=(sm.frame%500==0)))


if __name__ == "__main__":
  main()
