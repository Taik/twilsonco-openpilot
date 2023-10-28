#!/usr/bin/env python3
import json
import threading
from traceback import print_exception
import numpy as np
from time import strftime, gmtime
from cereal import custom
import cereal.messaging as messaging
from common.params import Params
from common.realtime import set_core_affinity, set_thread_affinity, Ratekeeper
from selfdrive.mapd.lib.osm import OSM
from selfdrive.mapd.lib.geo import distance_to_points
from selfdrive.mapd.lib.WayCollection import WayCollection
from selfdrive.mapd.config import QUERY_RADIUS, QUERY_RADIUS_OFFLINE, MIN_DISTANCE_FOR_NEW_QUERY, FULL_STOP_MAX_SPEED, LOOK_AHEAD_HORIZON_TIME
from system.swaglog import cloudlog

DataType = custom.LiveMapDataSP.DataType


_DEBUG = False
_CLOUDLOG_DEBUG = True
ROAD_NAME_TIMEOUT = 30 # secs


def _debug(msg, log_to_cloud=True):
  if _CLOUDLOG_DEBUG and log_to_cloud:
    cloudlog.debug(msg)
  if _DEBUG:
    print(msg)


def excepthook(args):
  _debug(f'MapD: Threading exception:\n{args}')
  print_exception(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = excepthook


class MapD():
  def __init__(self):
    self.osm = OSM()
    self.way_collection = None
    self.route = None
    self.last_gps_fix_timestamp = 0
    self.last_gps = None
    self.location_deg = None  # The current location in degrees.
    self.location_rad = None  # The current location in radians as a Numpy array.
    self.bearing_rad = None
    self.location_stdev = None  # The current location accuracy in mts. 1 standard devitation.
    self.gps_speed = 0.
    self.last_fetch_location = None
    self.last_route_update_fix_timestamp = 0
    self.last_publish_fix_timestamp = 0
    self.data_type = DataType.default
    self._op_enabled = False
    self._disengaging = False
    self._query_thread = None
    self._lock = threading.RLock()
    self.gps_sock = 'gpsLocationExternal'

    # dp - use LastGPSPosition as init position (if we are in a undercover car park?)
    # this way we can prefetch osm data before we get a fix.
    last_pos = Params().get("LastGPSPosition")
    if last_pos is not None and last_pos != "":
      l = json.loads(last_pos)
      lat = float(l["latitude"])
      lon = float(l["longitude"])
      self.location_rad = np.radians(np.array([lat, lon], dtype=float))
      self.location_deg = (lat, lon)
      self.bearing_rad = np.radians(0, dtype=float)
      _debug("Use LastGPSPosition position - lat: %s, lon: %s" % (lat, lon))

  def udpate_state(self, sm):
    sock = 'carControl'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    hud_control = sm[sock].hudControl
    self._disengaging = not hud_control.speedVisible and self._op_enabled
    self._op_enabled = hud_control.speedVisible

  def update_gps(self, sm):
    self.gps_sock = 'gpsLocationExternal' if sm.rcv_frame['gpsLocationExternal'] > 1 else 'gpsLocation'
    if not sm.updated[self.gps_sock] or not sm.valid[self.gps_sock]:
      return

    log = sm[self.gps_sock]
    self.last_gps = log

    # ignore the message if the fix is invalid
    if log.flags % 2 == 0:
      return

    self.last_gps_fix_timestamp = log.unixTimestampMillis  # Unix TS. Milliseconds since January 1, 1970.
    self.location_rad = np.radians(np.array([log.latitude, log.longitude], dtype=float))
    self.location_deg = (log.latitude, log.longitude)
    self.bearing_rad = np.radians(log.bearingDeg, dtype=float)
    self.gps_speed = log.speed
    self.location_stdev = log.accuracy if self.gps_sock == 'gpsLocationExternal' else 1  # gpsLocation doesn't report accuracy

    _debug('Mapd: ********* Got GPS fix'
           + f'Pos: {self.location_deg} +/- {self.location_stdev * 2.} mts.\n'
           + f'Bearing: {log.bearingDeg} +/- {log.bearingAccuracyDeg * 2.} deg.\n'
           + f'timestamp: {strftime("%d-%m-%y %H:%M:%S", gmtime(self.last_gps_fix_timestamp * 1e-3))}'
           + '*******', log_to_cloud=False)

  def _query_osm_not_blocking(self):
    def query(osm, location_deg, location_rad, radius):
      _debug(f'Mapd: Start query for OSM map data at {location_deg}')
      lat, lon = location_deg
      areas, ways, self.data_type = osm.fetch_road_ways_around_location(lat, lon, radius)
      _debug(f'Mapd: Query to OSM finished with {len(ways)} ways')

      # Only issue an update if we received some ways. Otherwise it is most likely a connectivity issue.
      # Will retry on next loop.
      if len(ways) > 0:
        new_way_collection = WayCollection(areas, ways, location_rad)

        # Use the lock to update the way_collection as it might be being used to update the route.
        _debug('Mapd: Locking to write results from osm.', log_to_cloud=False)
        with self._lock:
          self.way_collection = new_way_collection
          self.last_fetch_location = location_rad
          _debug(f'Mapd: Updated map data @ {location_deg} - got {len(ways)} ways')

        _debug('Mapd: Releasing Lock to write results from osm', log_to_cloud=False)

    # Ignore if we have a query thread already running.
    if self._query_thread is not None and self._query_thread.is_alive():
      return

    self._query_thread = threading.Thread(target=query, args=(self.osm, self.location_deg, self.location_rad,
                                                              QUERY_RADIUS_OFFLINE if self.data_type == DataType.offline else QUERY_RADIUS))
    set_thread_affinity(self._query_thread, [0, 1, 2, 3])
    self._query_thread.start()

  def updated_osm_data(self):
    if self.route is not None:
      distance_to_end = self.route.distance_to_end
      if distance_to_end is not None and distance_to_end >= MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query as long as we have a route with enough distance ahead.
        return

    if self.location_rad is None:
      return

    if self.last_fetch_location is not None:
      distance_since_last = distance_to_points(self.last_fetch_location, np.array([self.location_rad]))[0]
      if distance_since_last < (QUERY_RADIUS_OFFLINE if self.data_type == DataType.offline else QUERY_RADIUS) - MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query if are still not close to the border of previous query area
        return

    self._query_osm_not_blocking()

  def update_route(self):
    def update_proc():
      # Ensure we clear the route on op disengage, this way we can correct possible incorrect map data due
      # to wrongly locating or picking up the wrong route.
      if self._disengaging:
        self.route = None
        _debug('Mapd *****: Clearing Route as system is disengaging. ********')

      if self.way_collection is None or self.location_rad is None or self.bearing_rad is None:
        _debug('Mapd *****: Can not update route. Missing WayCollection, location or bearing ********')
        return

      if self.route is not None and self.last_route_update_fix_timestamp == self.last_gps_fix_timestamp:
        _debug('Mapd *****: Skipping route update. No new fix since last update ********')
        return

      self.last_route_update_fix_timestamp = self.last_gps_fix_timestamp

      # Create the route if not existent or if it was generated by an older way collection
      if self.route is None or self.route.way_collection_id != self.way_collection.id:
        self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.location_stdev)
        _debug(f'Mapd *****: Route created: \n{self.route}\n********')
        return

      # Do not attempt to update the route if the car is going close to a full stop, as the bearing can start
      # jumping and creating unnecessary losing of the route. Since the route update timestamp has been updated
      # a new liveMapDataSP message will be published with the current values (which is desirable)
      if self.gps_speed < FULL_STOP_MAX_SPEED:
        _debug('Mapd *****: Route Not updated as car has Stopped ********')
        return

      self.route.update(self.location_rad, self.bearing_rad, self.location_stdev)
      if self.route.located:
        _debug(f'Mapd *****: Route updated: \n{self.route}\n********')
        return

      # if an old route did not mange to locate, attempt to regenerate form way collection.
      self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.location_stdev)
      _debug(f'Mapd *****: Failed to update location in route. Regenerated with route: \n{self.route}\n********')

    # We use the lock when updating the route, as it reads `way_collection` which can ben updated by
    # a new query result from the _query_thread.
    _debug('Mapd: Locking to update route.', log_to_cloud=False)
    with self._lock:
      update_proc()

    _debug('Mapd: Releasing Lock to update route', log_to_cloud=False)

  def publish(self, pm, sm):
    # Ensure we have a route currently located
    if self.route is None or not self.route.located:
      _debug('Mapd: Skipping liveMapDataSP message as there is no route or is not located.')
      return

    # Ensure we have a route update since last publish
    if self.last_publish_fix_timestamp == self.last_route_update_fix_timestamp:
      _debug('Mapd: Skipping liveMapDataSP since there is no new gps fix.')
      return

    self.last_publish_fix_timestamp = self.last_route_update_fix_timestamp

    speed_limit = self.route.current_speed_limit
    next_speed_limit_section = self.route.next_speed_limit_section
    turn_speed_limit_section = self.route.current_curvature_speed_limit_section
    horizon_mts = self.gps_speed * LOOK_AHEAD_HORIZON_TIME
    next_turn_speed_limit_sections = self.route.next_curvature_speed_limit_sections(horizon_mts)
    current_road_name = self.route.current_road_name

    map_data_msg = messaging.new_message('liveMapDataSP')
    map_data_msg.valid = sm.all_alive(service_list=[self.gps_sock]) and \
                         sm.all_valid(service_list=[self.gps_sock])

    liveMapDataSP = map_data_msg.liveMapDataSP
    liveMapDataSP.lastGpsTimestamp = self.last_gps.unixTimestampMillis
    liveMapDataSP.lastGpsLatitude = float(self.last_gps.latitude)
    liveMapDataSP.lastGpsLongitude = float(self.last_gps.longitude)
    liveMapDataSP.lastGpsSpeed = float(self.last_gps.speed)
    liveMapDataSP.lastGpsBearingDeg = float(self.last_gps.bearingDeg)
    liveMapDataSP.lastGpsAccuracy = float(self.last_gps.accuracy if self.gps_sock == 'gpsLocationExternal' else 1)  # gpsLocation doesnt report accuracy
    liveMapDataSP.lastGpsBearingAccuracyDeg = float(self.last_gps.bearingAccuracyDeg)

    liveMapDataSP.speedLimitValid = bool(speed_limit is not None)
    liveMapDataSP.speedLimit = float(speed_limit if speed_limit is not None else 0.0)
    liveMapDataSP.speedLimitAheadValid = bool(next_speed_limit_section is not None)
    liveMapDataSP.speedLimitAhead = float(next_speed_limit_section.value
                                                     if next_speed_limit_section is not None else 0.0)
    liveMapDataSP.speedLimitAheadDistance = float(next_speed_limit_section.start
                                                             if next_speed_limit_section is not None else 0.0)

    liveMapDataSP.turnSpeedLimitValid = bool(turn_speed_limit_section is not None)
    liveMapDataSP.turnSpeedLimit = float(turn_speed_limit_section.value
                                                    if turn_speed_limit_section is not None else 0.0)
    liveMapDataSP.turnSpeedLimitSign = int(turn_speed_limit_section.curv_sign
                                                      if turn_speed_limit_section is not None else 0)
    liveMapDataSP.turnSpeedLimitEndDistance = float(turn_speed_limit_section.end
                                                               if turn_speed_limit_section is not None else 0.0)
    liveMapDataSP.turnSpeedLimitsAhead = [float(s.value) for s in next_turn_speed_limit_sections]
    liveMapDataSP.turnSpeedLimitsAheadDistances = [float(s.start) for s in next_turn_speed_limit_sections]
    liveMapDataSP.turnSpeedLimitsAheadSigns = [float(s.curv_sign) for s in next_turn_speed_limit_sections]

    liveMapDataSP.currentRoadName = str(current_road_name if current_road_name is not None else "")

    liveMapDataSP.dataType = self.data_type

    pm.send('liveMapDataSP', map_data_msg)
    _debug(f'Mapd *****: Publish: \n{map_data_msg}\n********', log_to_cloud=False)


# provides live map data information
def mapd_thread(sm=None, pm=None):
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("mapd: failed to set core affinity")
  mapd = MapD()
  rk = Ratekeeper(1., print_delay_threshold=None)  # Keeps rate at 1 hz

  # *** setup messaging
  if sm is None:
    sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'carControl'])
  if pm is None:
    pm = messaging.PubMaster(['liveMapDataSP'])

  while True:
    sm.update()
    mapd.udpate_state(sm)
    mapd.update_gps(sm)
    mapd.updated_osm_data()
    mapd.update_route()
    mapd.publish(pm, sm)
    rk.keep_time()


def main(sm=None, pm=None):
  mapd_thread(sm, pm)


if __name__ == "__main__":
  main()
