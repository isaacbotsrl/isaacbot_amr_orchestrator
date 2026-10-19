# Copyright 2026 IsaacBot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Use cases of the indoor navigation manager.

One class, because every use case reads or changes the same small state: which
localization mode runs, on which map, and that map's places. Calls that switch the
localization sub-stack block for seconds; the caller runs them off the ROS executor.

Localization follows the range sensors on its own:

* lidar and/or camera working, a map selected -> AMCL on that map;
* lidar and/or camera working, no map selected -> SLAM;
* neither working -> ODOMETRY: SLAM/AMCL keep running but get no scan, so map -> odom is
  held and the isaacbot dead-reckons until a sensor is back.
"""

import math
import threading
from typing import List, Optional, Tuple

from ..domain.model import (
    DomainError,
    LocalizationMode,
    LocalizationState,
    MapRecord,
    Place,
    PlaceBook,
    Pose2D,
    SeedSource,
    validate_map_name,
)
from ..domain.motion import StopDetector, StopThresholds
from ..domain.ports import (
    IndoorNavObserver,
    LocalizationController,
    MapOdomHold,
    MapRepository,
    MapSaver,
    RobotPoseSource,
)
from ..domain.sensors import ScanSource

ORIGIN = Pose2D(0.0, 0.0, 0.0)

# Spread AMCL gets around a remembered pose (standard deviations). AMCL's own default is about
# 0.5 m / 15 deg, too tight if the isaacbot was nudged or the pose is a few seconds stale.
RESTORE_SIGMA_XY = 1.5
RESTORE_SIGMA_YAW = math.pi / 4

NO_SENSOR = ('No lidar or camera works, so SLAM and AMCL have no scan. Wait until one is '
             'back.')


class IndoorNavService:

    def __init__(self, maps: MapRepository, localization: LocalizationController,
                 saver: MapSaver, robot: RobotPoseSource, observer: IndoorNavObserver,
                 odom_hold: MapOdomHold,
                 stop_thresholds: StopThresholds = StopThresholds(),
                 restore_sigma_xy: float = RESTORE_SIGMA_XY,
                 restore_sigma_yaw: float = RESTORE_SIGMA_YAW):
        self._maps = maps
        self._localization = localization
        self._saver = saver
        self._robot = robot
        self._observer = observer
        self._odom_hold = odom_hold
        self._lock = threading.RLock()
        self._state = LocalizationState()
        self._book = PlaceBook(map_name='')
        # Name the running SLAM session was last saved under: loading that map right after
        # saving it seeds AMCL with the pose SLAM had, so the isaacbot stays localized.
        self._saved_from_mapping = ''
        self._stops = StopDetector(stop_thresholds)
        self._restore_sigma_xy = restore_sigma_xy
        self._restore_sigma_yaw = restore_sigma_yaw
        # Updated by on_scan_source() before and after startup(); NONE until a sensor is seen.
        self._scan_source = ScanSource.NONE
        self._started = False
        # In ODOMETRY: the mode to go back to once a sensor works again.
        self._resume_mode: Optional[LocalizationMode] = None

    # --- queries ---------------------------------------------------------------------
    @property
    def state(self) -> LocalizationState:
        return self._state

    def places(self) -> List[Place]:
        return list(self._book.places)

    def maps(self) -> List[MapRecord]:
        return self._maps.list_maps()

    # --- lifecycle -------------------------------------------------------------------
    def startup(self) -> None:
        """Resume on the last active map, or start mapping when there is none.

        Without a working sensor neither can localize: hold map -> odom (placing the isaacbot
        at its last pose on the active map) and start SLAM/AMCL once a sensor is back.
        """
        with self._lock:
            self._started = True
            self._publish_maps()
            active = self._maps.active_map()
            if not (active and self._maps.exists(active)):
                active = ''
            if self._scan_source == ScanSource.NONE:
                self._odom_hold.hold((self._maps.last_pose(active) or ORIGIN) if active else None)
                self._resume_mode = (LocalizationMode.LOCALIZATION if active
                                     else LocalizationMode.MAPPING)
                self._set_state(LocalizationState(
                    mode=LocalizationMode.ODOMETRY, map_name=active,
                    message='No lidar or camera: dead-reckoning on odometry until one works.'))
            elif active:
                self._load_map(active)
            else:
                self._start_mapping()

    def shutdown(self) -> None:
        with self._lock:
            # While AMCL still runs and TF resolves: the pose the next boot starts from.
            self.record_pose()
            self._localization.stop()
            self._set_state(LocalizationState(message='Stopped.'))

    def on_scan_source(self, source: ScanSource) -> None:
        """The range sensor feeding SLAM/AMCL changed (NONE: none works).

        Losing the last one falls back to odometry; getting one back resumes. A switch between
        lidar and camera needs nothing here: both reach SLAM/AMCL on the same topic.
        """
        with self._lock:
            changed = source != self._scan_source
            self._scan_source = source
            mode = self._state.mode
            if source == ScanSource.NONE and mode in (LocalizationMode.MAPPING,
                                                      LocalizationMode.LOCALIZATION):
                self._enter_odometry()
            elif source != ScanSource.NONE and mode == LocalizationMode.ODOMETRY:
                self._resume()
            elif changed and self._started:  # before startup() there is no state to show
                self._set_state(self._state)

    def check_health(self) -> None:
        """Report a sub-stack that died on its own; it is not restarted automatically."""
        with self._lock:
            if self._state.mode in (LocalizationMode.MAPPING, LocalizationMode.LOCALIZATION) \
                    and not self._localization.running():
                self._set_state(self._state.with_(
                    mode=LocalizationMode.UNAVAILABLE,
                    message='The localization stack exited unexpectedly. Load a map or start '
                            'mapping again.'))

    # --- mapping / localization ------------------------------------------------------
    def start_mapping(self) -> None:
        with self._lock:
            if self._state.mode == LocalizationMode.ODOMETRY:
                raise DomainError(NO_SENSOR)
            self._start_mapping()

    def _start_mapping(self) -> None:
        with self._lock:
            self.record_pose()  # remember where we were on the map we are leaving
            self._switching('Starting SLAM...')
            try:
                self._localization.stop()
                self._localization.start_mapping()
            except Exception as error:  # noqa: BLE001 - reported to the operator
                self._set_state(LocalizationState(message=f'Could not start SLAM: {error}'))
                raise
            self._saved_from_mapping = ''
            self._set_places(PlaceBook(map_name=''))
            self._set_state(LocalizationState(
                mode=LocalizationMode.MAPPING, message='Drive the isaacbot around to build the map.'))

    def save_map(self, name: str) -> None:
        with self._lock:
            validate_map_name(name)
            if not self._mapping():
                raise DomainError('Start mapping before saving a map.')
            if self._maps.exists(name):
                raise DomainError(f"A map named '{name}' already exists; delete it first or "
                                  'pick another name.')
            self._maps.prepare(name)
            try:
                self._saver.save(self._maps.map_url(name))
            except Exception as error:  # noqa: BLE001
                self._maps.delete(name)
                raise DomainError(f'Saving the map failed: {error}') from error
            pose = self._robot.current_pose()
            if pose is not None:
                self._maps.save_last_pose(name, pose)
            self._saved_from_mapping = name
            self._set_state(self._state.with_(map_name=name, message=f"Saved as '{name}'."))
            self._publish_maps()

    def load_map(self, name: str, initial_pose: Optional[Pose2D] = None) -> None:
        with self._lock:
            validate_map_name(name)
            if not self._maps.exists(name):
                raise DomainError(f"No map named '{name}'.")
            if self._state.mode == LocalizationMode.ODOMETRY:
                raise DomainError(NO_SENSOR)
            self._load_map(name, initial_pose,
                           SeedSource.EXPLICIT if initial_pose is not None else None)

    def _load_map(self, name: str, pose: Optional[Pose2D] = None,
                  source: Optional[SeedSource] = None) -> None:
        with self._lock:
            # Remember where we were on the map we are leaving (or reloading) first, so a
            # reload of the same map starts from the freshest pose.
            self.record_pose()
            if pose is None:
                pose, source = self._seed_pose(name)
            # Already localizing: swap the map under the running map_server + AMCL. Stopping
            # the stack is slow and disturbs every other ROS process, so it is only the
            # fallback here.
            in_place = (self._state.mode == LocalizationMode.LOCALIZATION
                        and self._localization.running())
            map_yaml = self._maps.map_yaml_path(name)
            self._switching(f"Loading map '{name}'...")
            fallback_note = ''
            try:
                if in_place:
                    try:
                        self._localization.switch_map(map_yaml, pose)
                    except Exception as error:  # noqa: BLE001 - restarting still works
                        fallback_note = f' (in-place switch failed: {error}; restarted AMCL)'
                        in_place = False
                if not in_place:
                    self._localization.stop()
                    self._localization.start_localization(map_yaml, pose)
            except Exception as error:  # noqa: BLE001
                self._set_state(LocalizationState(message=f'Could not start AMCL: {error}'))
                raise
            self._maps.set_active_map(name)
            self._saved_from_mapping = ''
            self._set_places(PlaceBook(map_name=name, places=self._maps.load_places(name)))
            message = 'Localized on the saved map. Use Set pose if the isaacbot is elsewhere.'
            if source == SeedSource.LAST_POSE:
                message = self._widen(pose)
            elif source == SeedSource.ODOMETRY:
                message = self._widen(pose, from_odometry=True)
            self._set_state(LocalizationState(
                mode=LocalizationMode.LOCALIZATION, map_name=name,
                message=message + fallback_note))
            self._publish_maps()

    def delete_map(self, name: str) -> None:
        with self._lock:
            validate_map_name(name)
            if not self._maps.exists(name):
                raise DomainError(f"No map named '{name}'.")
            if self._localized_map() == name:
                raise DomainError(f"'{name}' is the map in use; load another map or start "
                                  'mapping first.')
            self._maps.delete(name)
            if self._maps.active_map() == name:
                self._maps.set_active_map('')
            self._publish_maps()

    def on_motion(self, linear: float, angular: float, now: float) -> None:
        """Speed sample from odometry; records the pose each time the isaacbot comes to rest.

        Periodic recording alone can leave the saved pose seconds behind if the isaacbot is
        switched off right after it stops. Saving at the stop makes it exact.
        """
        with self._lock:
            if self._stops.update(linear, angular, now):
                self.record_pose()

    def record_pose(self) -> None:
        """Remember where the isaacbot is, so AMCL starts there after a restart.

        Also while dead-reckoning on a saved map: a drifted pose beats the one from before the
        sensors went, and AMCL gets a wide spread around it either way.
        """
        with self._lock:
            name = self._localized_map()
            if not name:
                return
            pose = self._robot.current_pose()
            if pose is not None:
                self._maps.save_last_pose(name, pose)

    # --- places ----------------------------------------------------------------------
    def save_place(self, name: str, pose: Pose2D, place_id: str = '') -> Place:
        with self._lock:
            if self._state.mode != LocalizationMode.LOCALIZATION:
                raise DomainError('Places belong to a saved map: load one first.')
            if place_id and self._book.get(place_id) is None:
                raise DomainError(f"No place with id '{place_id}'.")
            place = self._book.upsert(Place.create(name, self._state.map_name, pose, place_id))
            self._maps.save_places(self._book.map_name, self._book.places)
            self._observer.on_places(self._book.map_name, self.places())
            return place

    def delete_place(self, place_id: str) -> Place:
        with self._lock:
            place = self._book.remove(place_id)
            self._maps.save_places(self._book.map_name, self._book.places)
            self._observer.on_places(self._book.map_name, self.places())
            return place

    # --- helpers ---------------------------------------------------------------------
    def _seed_pose(self, name: str) -> Tuple[Pose2D, SeedSource]:
        if self._state.mode == LocalizationMode.MAPPING and self._saved_from_mapping == name:
            current = self._robot.current_pose()
            if current is not None:
                return current, SeedSource.SLAM
        remembered = self._maps.last_pose(name)
        if remembered is not None:
            return remembered, SeedSource.LAST_POSE
        return ORIGIN, SeedSource.ORIGIN

    def _widen(self, pose: Pose2D, from_odometry: bool = False) -> str:
        """Give AMCL a wider start around an uncertain pose. Returns a note for the state."""
        what = 'the odometry pose' if from_odometry else 'the remembered pose'
        try:
            self._localization.widen_initial_estimate(
                pose, self._restore_sigma_xy, self._restore_sigma_yaw)
        except Exception as error:  # noqa: BLE001 - AMCL still runs with its default spread
            return f'Localized from {what} (could not widen the search: {error}).'
        if from_odometry:
            return ('Sensor back: localizing again from the odometry pose (search widened). '
                    'If AMCL does not converge, use Set pose or Find me.')
        return ('Localized from the last remembered pose (search widened once AMCL has its map). '
                'If the isaacbot was moved while off, use Set pose or Find me.')

    def _enter_odometry(self) -> None:
        """The last sensor is gone: keep SLAM/AMCL (and an unsaved map) but hold map -> odom."""
        resume = self._state.mode
        self.record_pose()  # the last pose AMCL vouched for
        self._odom_hold.hold()
        self._resume_mode = resume
        what = ('Mapping paused' if resume == LocalizationMode.MAPPING
                else 'Dead-reckoning on odometry')
        self._set_state(self._state.with_(
            mode=LocalizationMode.ODOMETRY,
            message=f'Lidar and camera lost. {what} until one is back.'))

    def _resume(self) -> None:
        resume, self._resume_mode = self._resume_mode, None
        if self._localization.running():
            # SLAM/AMCL kept running; AMCL moved its particles with the odometry meanwhile.
            self._odom_hold.release()
            message = 'Sensor back: mapping again.'
            if resume == LocalizationMode.LOCALIZATION:
                pose = self._robot.current_pose()
                message = (self._widen(pose, from_odometry=True) if pose is not None
                           else 'Sensor back: localizing again.')
            self._set_state(self._state.with_(mode=resume, message=message))
            return
        # Started without a sensor, or the stack died meanwhile: start it now. The hold is
        # released first but only lets go once SLAM/AMCL publish map -> odom themselves.
        self._odom_hold.release()
        active = self._maps.active_map()
        if resume == LocalizationMode.LOCALIZATION and active and self._maps.exists(active):
            # Where odometry has taken the isaacbot on the held map -> odom.
            pose = self._robot.current_pose()
            self._load_map(active, pose, SeedSource.ODOMETRY if pose is not None else None)
        else:
            self._start_mapping()

    def _mapping(self) -> bool:
        """SLAM is running, or paused while no sensor works."""
        return (self._state.mode == LocalizationMode.MAPPING
                or (self._state.mode == LocalizationMode.ODOMETRY
                    and self._resume_mode == LocalizationMode.MAPPING))

    def _localized_map(self) -> str:
        """The saved map the isaacbot is on (by AMCL or dead-reckoning), else ''."""
        mode = self._state.mode
        if mode == LocalizationMode.LOCALIZATION or (
                mode == LocalizationMode.ODOMETRY
                and self._resume_mode == LocalizationMode.LOCALIZATION):
            return self._state.map_name
        return ''

    def _switching(self, message: str) -> None:
        self._set_state(self._state.with_(mode=LocalizationMode.SWITCHING, message=message))

    def _set_state(self, state: LocalizationState) -> None:
        state = state.with_(scan_source=self._scan_source)
        self._state = state
        self._observer.on_state(state)

    def _set_places(self, book: PlaceBook) -> None:
        self._book = book
        self._observer.on_places(book.map_name, self.places())

    def _publish_maps(self) -> None:
        self._observer.on_maps(self._maps.list_maps(), self._maps.active_map())
