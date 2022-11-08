#!/usr/bin/python
# -*- coding: utf-8 -*-

  # #   ######   #####  #     #  #####   #####  
  # #   #     # #     #  #   #  #     # #     # 
####### #     # #         # #         #       # 
  # #   ######  #          #     #####   #####  
####### #     # #         # #   #       #       
  # #   #     # #     #  #   #  #       #       
  # #   ######   #####  #     # ####### ####### 

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref

try:
	import pygame
	from pygame.locals import KMOD_CTRL
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_q
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
	import pygame
	from pygame.locals import KMOD_CTRL
	from pygame.locals import KMOD_SHIFT
	from pygame.locals import K_0
	from pygame.locals import K_9
	from pygame.locals import K_BACKQUOTE
	from pygame.locals import K_BACKSPACE
	from pygame.locals import K_COMMA
	from pygame.locals import K_DOWN
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_F1
	from pygame.locals import K_LEFT
	from pygame.locals import K_PERIOD
	from pygame.locals import K_RIGHT
	from pygame.locals import K_SLASH
	from pygame.locals import K_SPACE
	from pygame.locals import K_TAB
	from pygame.locals import K_UP
	from pygame.locals import K_a
	from pygame.locals import K_b
	from pygame.locals import K_c
	from pygame.locals import K_d
	from pygame.locals import K_g
	from pygame.locals import K_h
	from pygame.locals import K_i
	from pygame.locals import K_l
	from pygame.locals import K_m
	from pygame.locals import K_n
	from pygame.locals import K_o
	from pygame.locals import K_p
	from pygame.locals import K_q
	from pygame.locals import K_r
	from pygame.locals import K_s
	from pygame.locals import K_t
	from pygame.locals import K_v
	from pygame.locals import K_w
	from pygame.locals import K_x
	from pygame.locals import K_z
	from pygame.locals import K_MINUS
	from pygame.locals import K_EQUALS
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
	import numpy as np
except ImportError:
	raise RuntimeError(
		'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Ecal Input Manager --------------------------------------------------------
# ==============================================================================
import sys
import time

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

sys.path.insert(1, os.path.join(sys.path[0], 'datatypes_python'))
from datatypes_python.HMI import HMICanKeyboard_pb2
from datatypes_python.SensorNearData import Brake_pb2
from datatypes_python.SensorNearData import VehicleDynamics_pb2

from ecal.core.publisher import StringPublisher

import ecal.core.core as ecal_core


class eCAL_Interface:
	def __init__(self):
		self.brake = 42 # 0 = off,  1= on, 42 = init
		self.steering = 0 # 0= init, 0.4 = left_max, -0.4 = right_max
		self.steering_tux = 42 #42 = init, 0= neutral, 1 = left, -1 = right

		def brake_cb(topic_name, msg, time):
			global eCAL_Command
			if msg.signals.is_brake_applied == True:
				#print("Break active")
				eCAL_Command.brake = 1
			else:
				#print("Break off")
				eCAL_Command.brake = 0

		def steering_cb(topic_name, msg , time):
			global eCAL_Command
			eCAL_Command.steering = msg.signals.steering_wheel_angle
		ecal_core.initialize(sys.argv, "carla")
		self._pub = StringPublisher("Carla")

		#sub_hmican = ProtoSubscriber("HmiCanKeyboardStatePb", HMICanKeyboard_pb2.HmiCanKeyboardState)
		#sub_hmican.set_callback(can_callback)
		sub_brake = ProtoSubscriber("BrakeInPb", Brake_pb2.Brake)
		sub_brake.set_callback(brake_cb)
		sub_steering = ProtoSubscriber("VehicleDynamicsInPb", VehicleDynamics_pb2.VehicleDynamics)
		sub_steering.set_callback(steering_cb)

	def sendMessage(self, message):
		self._pub.send(message)

	def destroy(self):
		ecal_core.finalize()

eCAL_Command = eCAL_Interface()

# ==============================================================================
# -- Add PythonAPI for release ode --------------------------------------------
# ==============================================================================
try:
	sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
	pass

import carla
from carla import ColorConverter as cc


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
	"""Method to find weather presets"""
	rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
	def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
	presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
	return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
	"""Method to get actor display name"""
	name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
	return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
	""" Class representing the surrounding environment """

	def __init__(self, carla_world, hud, args):
		"""Constructor method"""
		self._args = args
		self.world = carla_world
		try:
			self.map = self.world.get_map()
		except RuntimeError as error:
			print('RuntimeError: {}'.format(error))
			print('  The server could not send the OpenDRIVE (.xodr) file:')
			print('  Make sure it exists, has the same name of your town, and is correct.')
			sys.exit(1)
		self.hud = hud
		self.player = None
		self.collision_sensor = None
		self.lane_invasion_sensor = None
		self.gnss_sensor = None
		self.camera_manager = None
		self._weather_presets = find_weather_presets()
		self._weather_index = 0
		self._actor_filter = args.filter
		self.restart()
		self.world.on_tick(hud.on_world_tick)
		self.recording_enabled = False
		self.recording_start = 0

	def restart(self):
		"""Restart the world"""
		# Keep same camera config if the camera manager exists.
		cam_index = self.camera_manager.index if self.camera_manager is not None else 0
		cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

		# Get a random blueprint.
		for a in self.world.get_blueprint_library().filter(self._actor_filter):
			print(a)
		blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
		blueprint.set_attribute('role_name', 'hero')
		if blueprint.has_attribute('color'):
			color = random.choice(blueprint.get_attribute('color').recommended_values)
			blueprint.set_attribute('color', color)

		# Spawn the player.
		# if self.player is not None:
		#     spawn_point = self.player.get_transform()
		#     spawn_point.location.z += 2.0
		#     spawn_point.rotation.roll = 0.0
		#     spawn_point.rotation.pitch = 0.0
		#     self.destroy()
		#     self.player = self.world.try_spawn_actor(blueprint, spawn_point)
		#     self.modify_vehicle_physics(self.player)
		

		# Spawn the player.
		if self.player is not None:
			self.destroy()
		self.player = None
		while self.player is None:
			if not self.map.get_spawn_points():
				print('There are no spawn points available in your map/town.')
				print('Please add some Vehicle Spawn Point to your UE4 scene.')
				sys.exit(1)
			spawn_points = self.map.get_spawn_points()
			spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
			self.player = self.world.try_spawn_actor(blueprint, spawn_point)
			self.modify_vehicle_physics(self.player)

		if self._args.sync:
			self.world.tick()
		else:
			self.world.wait_for_tick()

		# Set up the sensors.
		self.collision_sensor = CollisionSensor(self.player, self.hud)
		self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
		self.gnss_sensor = GnssSensor(self.player)
		self.camera_manager = CameraManager(self.player, self.hud)
		self.camera_manager.transform_index = cam_pos_id
		self.camera_manager.set_sensor(cam_index, notify=False)
		actor_type = get_actor_display_name(self.player)
		self.hud.notification(actor_type)

	def next_weather(self, reverse=False):
		"""Get next weather setting"""
		self._weather_index += -1 if reverse else 1
		self._weather_index %= len(self._weather_presets)
		preset = self._weather_presets[self._weather_index]
		self.hud.notification('Weather: %s' % preset[1])
		self.player.get_world().set_weather(preset[0])

	def modify_vehicle_physics(self, actor):
		#If actor is not a vehicle, we cannot use the physics control
		try:
			physics_control = actor.get_physics_control()
			physics_control.use_sweep_wheel_collision = True
			actor.apply_physics_control(physics_control)
		except Exception:
			pass

	def tick(self, clock):
		"""Method for every tick"""
		self.hud.tick(self, clock)

	def render(self, display):
		"""Render world"""
		self.camera_manager.render(display)
		self.hud.render(display)

	def destroy_sensors(self):
		"""Destroy sensors"""
		self.camera_manager.sensor.destroy()
		self.camera_manager.sensor = None
		self.camera_manager.index = None

	def destroy(self):
		"""Destroys all actors"""
		actors = [
			self.camera_manager.sensor,
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.gnss_sensor.sensor,
			self.player]
		for actor in actors:
			if actor is not None:
				actor.destroy()

# ==============================================================================
# -- Generate Traffic ----------------------------------------------------------
# ==============================================================================
class Traffic:
	def __init__(self, client, args):
		self.client = client
		world = client.get_world()
		self.vehicles_list = []
		self.walkers_list = []
		self.all_id = []

		traffic_manager = client.get_trafficmanager()
		traffic_manager.set_global_distance_to_leading_vehicle(2.5)
		if args.hybrid:
			traffic_manager.set_hybrid_physics_mode(True)
			traffic_manager.set_hybrid_physics_radius(70.0)
		if args.seed is not None:
			traffic_manager.set_random_device_seed(args.seed)

		settings = world.get_settings()
		if args.sync:
			traffic_manager.set_synchronous_mode(True)
			if not settings.synchronous_mode:
				synchronous_master = True
				settings.synchronous_mode = True
				settings.fixed_delta_seconds = 0.05
			else:
				synchronous_master = False
		else:
			print("You are currently in asynchronous mode. If this is a traffic simulation, \
			you could experience some issues. If it's not working correctly, switch to synchronous \
			mode by using traffic_manager.set_synchronous_mode(True)")

		world.apply_settings(settings)

		blueprints = self.get_actor_blueprints(world, args.filterv, args.generationv)
		blueprintsWalkers = self.get_actor_blueprints(world, args.filterw, args.generationw)

		if True:
			blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
			blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
			blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
			blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
			blueprints = [x for x in blueprints if not x.id.endswith('t2')]
			blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
			blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
			blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

		blueprints = sorted(blueprints, key=lambda bp: bp.id)

		spawn_points = world.get_map().get_spawn_points()
		number_of_spawn_points = len(spawn_points)

		if args.number_of_vehicles < number_of_spawn_points:
			random.shuffle(spawn_points)
		elif args.number_of_vehicles > number_of_spawn_points:
			msg = 'requested %d vehicles, but could only find %d spawn points'
			logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
			args.number_of_vehicles = number_of_spawn_points

		# @todo cannot import these directly.
		SpawnActor = carla.command.SpawnActor
		SetAutopilot = carla.command.SetAutopilot
		FutureActor = carla.command.FutureActor

		# --------------
		# Spawn vehicles
		# --------------
		batch = []
		for n, transform in enumerate(spawn_points):
			if n >= args.number_of_vehicles:
				break
			blueprint = random.choice(blueprints)
			if blueprint.has_attribute('color'):
				color = random.choice(blueprint.get_attribute('color').recommended_values)
				blueprint.set_attribute('color', color)
			if blueprint.has_attribute('driver_id'):
				driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
				blueprint.set_attribute('driver_id', driver_id)
			else:
				blueprint.set_attribute('role_name', 'autopilot')

			# spawn the cars and set their autopilot and light state all together
			batch.append(SpawnActor(blueprint, transform)
				.then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

		for response in client.apply_batch_sync(batch, synchronous_master):
			if response.error:
				logging.error(response.error)
			else:
				self.vehicles_list.append(response.actor_id)

		# Set automatic vehicle lights update if specified
		if args.car_lights_on:
			all_vehicle_actors = world.get_actors(self.vehicles_list)
			for actor in all_vehicle_actors:
				traffic_manager.update_vehicle_lights(actor, True)

		# -------------
		# Spawn Walkers
		# -------------
		# some settings
		percentagePedestriansRunning = 0.0      # how many pedestrians will run
		percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
		if args.seedw:
			world.set_pedestrians_seed(args.seedw)
			random.seed(args.seedw)
		# 1. take all the random locations to spawn
		spawn_points = []
		for i in range(args.number_of_walkers):
			spawn_point = carla.Transform()
			loc = world.get_random_location_from_navigation()
			if (loc != None):
				spawn_point.location = loc
				spawn_points.append(spawn_point)
		# 2. we spawn the walker object
		batch = []
		walker_speed = []
		for spawn_point in spawn_points:
			walker_bp = random.choice(blueprintsWalkers)
			# set as not invincible
			if walker_bp.has_attribute('is_invincible'):
				walker_bp.set_attribute('is_invincible', 'false')
			# set the max speed
			if walker_bp.has_attribute('speed'):
				if (random.random() > percentagePedestriansRunning):
					# walking
					walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
				else:
					# running
					walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
			else:
				print("Walker has no speed")
				walker_speed.append(0.0)
			batch.append(SpawnActor(walker_bp, spawn_point))
		results = client.apply_batch_sync(batch, True)
		walker_speed2 = []
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				self.walkers_list.append({"id": results[i].actor_id})
				walker_speed2.append(walker_speed[i])
		walker_speed = walker_speed2
		# 3. we spawn the walker controller
		batch = []
		walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
		for i in range(len(self.walkers_list)):
			batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
		results = client.apply_batch_sync(batch, True)
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				self.walkers_list[i]["con"] = results[i].actor_id
		# 4. we put together the walkers and controllers id to get the objects from their id
		for i in range(len(self.walkers_list)):
			self.all_id.append(self.walkers_list[i]["con"])
			self.all_id.append(self.walkers_list[i]["id"])
		self.all_actors = world.get_actors(self.all_id)

		# wait for a tick to ensure client receives the last transform of the walkers we have just created
		if args.sync:
			world.tick()
		else:
			world.wait_for_tick()

		# 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
		# set how many pedestrians can cross the road
		world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
		for i in range(0, len(self.all_id), 2):
			# start walker
			self.all_actors[i].start()
			# set walk to random point
			self.all_actors[i].go_to_location(world.get_random_location_from_navigation())
			# max speed
			self.all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

		print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(self.vehicles_list), len(self.walkers_list)))

		# Example of how to use Traffic Manager parameters
		traffic_manager.global_percentage_speed_difference(30.0)


	def get_actor_blueprints(self, world, filter, generation):
		bps = world.get_blueprint_library().filter(filter)

		if generation.lower() == "all":
			return bps

		# If the filter returns only one bp, we assume that this one needed
		# and therefore, we ignore the generation
		if len(bps) == 1:
			return bps

		try:
			int_generation = int(generation)
			# Check if generation is in available generations
			if int_generation in [1, 2]:
				bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
				return bps
			else:
				print("   Warning! Actor Generation is not valid. No actor will be spawned.")
				return []
		except:
			print("   Warning! Actor Generation is not valid. No actor will be spawned.")
			return []
	def destroy(self):
		print('\ndestroying %d vehicles' % len(self.vehicles_list))
		self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

		# stop walker controllers (list is [controller, actor, controller, actor ...])
		for i in range(0, len(self.all_id), 2):
			self.all_actors[i].stop()

		print('\ndestroying %d walkers' % len(self.walkers_list))
		self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
	"""Class for HUD text"""

	def __init__(self, width, height):
		"""Constructor method"""
		self.dim = (width, height)
		font = pygame.font.Font(pygame.font.get_default_font(), 20)
		font_name = 'courier' if os.name == 'nt' else 'mono'
		fonts = [x for x in pygame.font.get_fonts() if font_name in x]
		default_font = 'ubuntumono'
		mono = default_font if default_font in fonts else fonts[0]
		mono = pygame.font.match_font(mono)
		self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
		self._notifications = FadingText(font, (width, 40), (0, height - 40))
		self.help = HelpText(pygame.font.Font(mono, 24), width, height)
		self.server_fps = 0
		self.frame = 0
		self.simulation_time = 0
		self._show_info = True
		self._info_text = []
		self._server_clock = pygame.time.Clock()

	def on_world_tick(self, timestamp):
		"""Gets informations from the world at every tick"""
		self._server_clock.tick()
		self.server_fps = self._server_clock.get_fps()
		self.frame = timestamp.frame_count
		self.simulation_time = timestamp.elapsed_seconds

	def tick(self, world, clock):
		"""HUD method for every tick"""
		self._notifications.tick(world, clock)
		if not self._show_info:
			return
		transform = world.player.get_transform()
		vel = world.player.get_velocity()
		control = world.player.get_control()
		heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
		heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
		heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
		heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
		colhist = world.collision_sensor.get_collision_history()
		collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
		max_col = max(1.0, max(collision))
		collision = [x / max_col for x in collision]
		vehicles = world.world.get_actors().filter('vehicle.*')

		self._info_text = [
			'Server:  % 16.0f FPS' % self.server_fps,
			'Client:  % 16.0f FPS' % clock.get_fps(),
			'',
			'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
			'Map:     % 20s' % world.map.name.split('/')[-1],
			'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
			'',
			'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)),
			u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
			'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
			'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
			'Height:  % 18.0f m' % transform.location.z,
			'']
		if isinstance(control, carla.VehicleControl):
			self._info_text += [
				('Throttle:', control.throttle, 0.0, 1.0),
				('Steer:', control.steer, -1.0, 1.0),
				('Brake:', control.brake, 0.0, 1.0),
				('Reverse:', control.reverse),
				('Hand brake:', control.hand_brake),
				('Manual:', control.manual_gear_shift),
				'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
		elif isinstance(control, carla.WalkerControl):
			self._info_text += [
				('Speed:', control.speed, 0.0, 5.556),
				('Jump:', control.jump)]
		self._info_text += [
			'',
			'Collision:',
			collision,
			'',
			'Number of vehicles: % 8d' % len(vehicles)]

		if len(vehicles) > 1:
			self._info_text += ['Nearby vehicles:']

		def dist(l):
			return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)
							 ** 2 + (l.z - transform.location.z)**2)
		vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

		for dist, vehicle in sorted(vehicles):
			if dist > 200.0:
				break
			vehicle_type = get_actor_display_name(vehicle, truncate=22)
			self._info_text.append('% 4dm %s' % (dist, vehicle_type))

	def toggle_info(self):
		"""Toggle info on or off"""
		self._show_info = not self._show_info

	def notification(self, text, seconds=2.0):
		"""Notification text"""
		self._notifications.set_text(text, seconds=seconds)

	def error(self, text):
		"""Error text"""
		self._notifications.set_text('Error: %s' % text, (255, 0, 0))

	def render(self, display):
		"""Render for HUD class"""
		if self._show_info:
			info_surface = pygame.Surface((220, self.dim[1]))
			info_surface.set_alpha(100)
			display.blit(info_surface, (0, 0))
			v_offset = 4
			bar_h_offset = 100
			bar_width = 106
			for item in self._info_text:
				if v_offset + 18 > self.dim[1]:
					break
				if isinstance(item, list):
					if len(item) > 1:
						points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
						pygame.draw.lines(display, (255, 136, 0), False, points, 2)
					item = None
					v_offset += 18
				elif isinstance(item, tuple):
					if isinstance(item[1], bool):
						rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
						pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
					else:
						rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
						pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
						fig = (item[1] - item[2]) / (item[3] - item[2])
						if item[2] < 0.0:
							rect = pygame.Rect(
								(bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
						else:
							rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
						pygame.draw.rect(display, (255, 255, 255), rect)
					item = item[0]
				if item:  # At this point has to be a str.
					surface = self._font_mono.render(item, True, (255, 255, 255))
					display.blit(surface, (8, v_offset))
				v_offset += 18
		self._notifications.render(display)
		self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
	""" Class for fading text """

	def __init__(self, font, dim, pos):
		"""Constructor method"""
		self.font = font
		self.dim = dim
		self.pos = pos
		self.seconds_left = 0
		self.surface = pygame.Surface(self.dim)

	def set_text(self, text, color=(255, 255, 255), seconds=2.0):
		"""Set fading text"""
		text_texture = self.font.render(text, True, color)
		self.surface = pygame.Surface(self.dim)
		self.seconds_left = seconds
		self.surface.fill((0, 0, 0, 0))
		self.surface.blit(text_texture, (10, 11))

	def tick(self, _, clock):
		"""Fading text method for every tick"""
		delta_seconds = 1e-3 * clock.get_time()
		self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
		self.surface.set_alpha(500.0 * self.seconds_left)

	def render(self, display):
		"""Render fading text method"""
		display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
	""" Helper class for text render"""

	def __init__(self, font, width, height):
		"""Constructor method"""
		lines = __doc__.split('\n')
		self.font = font
		self.dim = (680, len(lines) * 22 + 12)
		self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
		self.seconds_left = 0
		self.surface = pygame.Surface(self.dim)
		self.surface.fill((0, 0, 0, 0))
		for i, line in enumerate(lines):
			text_texture = self.font.render(line, True, (255, 255, 255))
			self.surface.blit(text_texture, (22, i * 22))
			self._render = False
		self.surface.set_alpha(220)

	def toggle(self):
		"""Toggle on or off the render help"""
		self._render = not self._render

	def render(self, display):
		"""Render help text method"""
		if self._render:
			display.blit(self.surface, self.pos)

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
	""" Class for collision sensors"""

	def __init__(self, parent_actor, hud):
		"""Constructor method"""
		self.sensor = None
		self.history = []
		self._parent = parent_actor
		self.hud = hud
		world = self._parent.get_world()
		blueprint = world.get_blueprint_library().find('sensor.other.collision')
		self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to
		# self to avoid circular reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

	def get_collision_history(self):
		"""Gets the history of collisions"""
		history = collections.defaultdict(int)
		for frame, intensity in self.history:
			history[frame] += intensity
		return history

	@staticmethod
	def _on_collision(weak_self, event):
		"""On collision method"""
		self = weak_self()
		if not self:
			return
		actor_type = get_actor_display_name(event.other_actor)
		self.hud.notification('Collision with %r' % actor_type)
		eCAL_Command.sendMessage('Crashed')
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
		self.history.append((event.frame, intensity))
		if len(self.history) > 4000:
			self.history.pop(0)

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
	"""Class for lane invasion sensors"""

	def __init__(self, parent_actor, hud):
		"""Constructor method"""
		self.sensor = None
		self._parent = parent_actor
		self.hud = hud
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
		self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

	@staticmethod
	def _on_invasion(weak_self, event):
		"""On invasion method"""
		self = weak_self()
		if not self:
			return
		lane_types = set(x.type for x in event.crossed_lane_markings)
		for crossed in event.crossed_lane_markings:
			if crossed.type in [carla.LaneMarkingType.SolidSolid, carla.LaneMarkingType.Solid]:
				self.hud.notification('Crossed line %s' % crossed.type)
				eCAL_Command.sendMessage('Crossed Line')

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
	""" Class for GNSS sensors"""

	def __init__(self, parent_actor):
		"""Constructor method"""
		self.sensor = None
		self._parent = parent_actor
		self.lat = 0.0
		self.lon = 0.0
		world = self._parent.get_world()
		blueprint = world.get_blueprint_library().find('sensor.other.gnss')
		self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)),
										attach_to=self._parent)
		# We need to pass the lambda a weak reference to
		# self to avoid circular reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

	@staticmethod
	def _on_gnss_event(weak_self, event):
		"""GNSS method"""
		self = weak_self()
		if not self:
			return
		self.lat = event.latitude
		self.lon = event.longitude

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
	""" Class for camera management"""

	def __init__(self, parent_actor, hud):
		"""Constructor method"""
		self.sensor = None
		self.surface = None
		self._parent = parent_actor
		self.hud = hud
		self.recording = False
		bound_y = 0.5 + self._parent.bounding_box.extent.y
		attachment = carla.AttachmentType
		self._camera_transforms = [
			(carla.Transform(
				carla.Location(x=2.6, y=-0.8, z=2.7)), attachment.Rigid),
			(carla.Transform(
				carla.Location(x=-9.5, z=6.5), carla.Rotation(pitch=8.0)), attachment.SpringArm),
			]
		self.transform_index = 1
		self.sensors = [
			['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
			['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
			['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
			['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
			['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
			['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
			 'Camera Semantic Segmentation (CityScapes Palette)'],
			['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
		world = self._parent.get_world()
		bp_library = world.get_blueprint_library()
		for item in self.sensors:
			blp = bp_library.find(item[0])
			if item[0].startswith('sensor.camera'):
				blp.set_attribute('image_size_x', str(hud.dim[0]))
				blp.set_attribute('image_size_y', str(hud.dim[1]))
			elif item[0].startswith('sensor.lidar'):
				blp.set_attribute('range', '50')
			item.append(blp)
		self.index = None

	def toggle_camera(self):
		"""Activate a camera"""
		self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
		self.set_sensor(self.index, notify=False, force_respawn=True)

	def set_sensor(self, index, notify=True, force_respawn=False):
		"""Set a sensor"""
		index = index % len(self.sensors)
		needs_respawn = True if self.index is None else (
			force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
		if needs_respawn:
			if self.sensor is not None:
				self.sensor.destroy()
				self.surface = None
			self.sensor = self._parent.get_world().spawn_actor(
				self.sensors[index][-1],
				self._camera_transforms[self.transform_index][0],
				attach_to=self._parent,
				attachment_type=self._camera_transforms[self.transform_index][1])

			# We need to pass the lambda a weak reference to
			# self to avoid circular reference.
			weak_self = weakref.ref(self)
			self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
		if notify:
			self.hud.notification(self.sensors[index][2])
		self.index = index

	def next_sensor(self):
		"""Get the next sensor"""
		self.set_sensor(self.index + 1)

	def toggle_recording(self):
		"""Toggle recording on or off"""
		self.recording = not self.recording
		self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

	def render(self, display):
		"""Render method"""
		if self.surface is not None:
			display.blit(self.surface, (0, 0))

	@staticmethod
	def _parse_image(weak_self, image):
		self = weak_self()
		if not self:
			return
		if self.sensors[self.index][0].startswith('sensor.lidar'):
			points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
			points = np.reshape(points, (int(points.shape[0] / 4), 4))
			lidar_data = np.array(points[:, :2])
			lidar_data *= min(self.hud.dim) / 100.0
			lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
			lidar_data = np.fabs(lidar_data)  # pylint: disable=assignment-from-no-return
			lidar_data = lidar_data.astype(np.int32)
			lidar_data = np.reshape(lidar_data, (-1, 2))
			lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
			lidar_img = np.zeros(lidar_img_size)
			lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
			self.surface = pygame.surfarray.make_surface(lidar_img)
		else:
			image.convert(self.sensors[self.index][1])
			array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
			array = np.reshape(array, (image.height, image.width, 4))
			array = array[:, :, :3]
			array = array[:, :, ::-1]
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
		if self.recording:
			image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
	"""Class that handles keyboard input."""
	def __init__(self, control):
		self._autopilot_enabled = False
		self._control  = control
		self._steer_cache = 0.0
		self._lights = carla.VehicleLightState.NONE

	def parse_events(self, client, world, clock, sync_mode):
		if isinstance(self._control, carla.VehicleControl):
			current_lights = self._lights
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return True
			elif event.type == pygame.KEYUP:
				if self._is_quit_shortcut(event.key):
					return True
				elif event.key == K_BACKSPACE:
					if self._autopilot_enabled:
						world.player.set_autopilot(False)
						world.restart()
						world.player.set_autopilot(True)
					else:
						world.restart()
				elif event.key == K_F1:
					world.hud.toggle_info()
				elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
					world.next_map_layer(reverse=True)
				elif event.key == K_v:
					world.next_map_layer()
				elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
					world.load_map_layer(unload=True)
				elif event.key == K_b:
					world.load_map_layer()
				elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
					world.hud.help.toggle()
				elif event.key == K_TAB:
					world.camera_manager.toggle_camera()
				elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
					world.next_weather(reverse=True)
				elif event.key == K_c:
					world.next_weather()
				elif event.key == K_g:
					world.toggle_radar()
				elif event.key == K_BACKQUOTE:
					world.camera_manager.next_sensor()
				elif event.key == K_n:
					world.camera_manager.next_sensor()
				elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
					if world.constant_velocity_enabled:
						world.player.disable_constant_velocity()
						world.constant_velocity_enabled = False
						world.hud.notification("Disabled Constant Velocity Mode")
					else:
						world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
						world.constant_velocity_enabled = True
						world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
				elif event.key == K_o:
					try:
						if world.doors_are_open:
							world.hud.notification("Closing Doors")
							world.doors_are_open = False
							world.player.close_door(carla.VehicleDoor.All)
						else:
							world.hud.notification("Opening doors")
							world.doors_are_open = True
							world.player.open_door(carla.VehicleDoor.All)
					except Exception:
						pass
				elif event.key == K_t:
					if world.show_vehicle_telemetry:
						world.player.show_debug_telemetry(False)
						world.show_vehicle_telemetry = False
						world.hud.notification("Disabled Vehicle Telemetry")
					else:
						try:
							world.player.show_debug_telemetry(True)
							world.show_vehicle_telemetry = True
							world.hud.notification("Enabled Vehicle Telemetry")
						except Exception:
							pass
				elif event.key > K_0 and event.key <= K_9:
					index_ctrl = 0
					if pygame.key.get_mods() & KMOD_CTRL:
						index_ctrl = 9
					world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
				elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
					world.camera_manager.toggle_recording()
				elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
					if (world.recording_enabled):
						client.stop_recorder()
						world.recording_enabled = False
						world.hud.notification("Recorder is OFF")
					else:
						client.start_recorder("manual_recording.rec")
						world.recording_enabled = True
						world.hud.notification("Recorder is ON")
				elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
					# stop recorder
					client.stop_recorder()
					world.recording_enabled = False
					# work around to fix camera at start of replaying
					current_index = world.camera_manager.index
					world.destroy_sensors()
					# disable autopilot
					self._autopilot_enabled = False
					world.player.set_autopilot(self._autopilot_enabled)
					world.hud.notification("Replaying file 'manual_recording.rec'")
					# replayer
					client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
					world.camera_manager.set_sensor(current_index)
				elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
					if pygame.key.get_mods() & KMOD_SHIFT:
						world.recording_start -= 10
					else:
						world.recording_start -= 1
					world.hud.notification("Recording start time is %d" % (world.recording_start))
				elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
					if pygame.key.get_mods() & KMOD_SHIFT:
						world.recording_start += 10
					else:
						world.recording_start += 1
					world.hud.notification("Recording start time is %d" % (world.recording_start))
				if isinstance(self._control, carla.VehicleControl):
					if event.key == K_q:
						self._control.gear = 1 if self._control.reverse else -1
					elif event.key == K_m:
						self._control.manual_gear_shift = not self._control.manual_gear_shift
						self._control.gear = world.player.get_control().gear
						world.hud.notification('%s Transmission' %
											   ('Manual' if self._control.manual_gear_shift else 'Automatic'))
					elif self._control.manual_gear_shift and event.key == K_COMMA:
						self._control.gear = max(-1, self._control.gear - 1)
					elif self._control.manual_gear_shift and event.key == K_PERIOD:
						self._control.gear = self._control.gear + 1
					elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
						if not self._autopilot_enabled and not sync_mode:
							print("WARNING: You are currently in asynchronous mode and could "
								  "experience some issues with the traffic simulation")
						self._autopilot_enabled = not self._autopilot_enabled
						world.player.set_autopilot(self._autopilot_enabled)
						world.hud.notification(
							'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
					elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
						current_lights ^= carla.VehicleLightState.Special1
					elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
						current_lights ^= carla.VehicleLightState.HighBeam
					elif event.key == K_l:
						# Use 'L' key to switch between lights:
						# closed -> position -> low beam -> fog
						if not self._lights & carla.VehicleLightState.Position:
							world.hud.notification("Position lights")
							current_lights |= carla.VehicleLightState.Position
						else:
							world.hud.notification("Low beam lights")
							current_lights |= carla.VehicleLightState.LowBeam
						if self._lights & carla.VehicleLightState.LowBeam:
							world.hud.notification("Fog lights")
							current_lights |= carla.VehicleLightState.Fog
						if self._lights & carla.VehicleLightState.Fog:
							world.hud.notification("Lights off")
							current_lights ^= carla.VehicleLightState.Position
							current_lights ^= carla.VehicleLightState.LowBeam
							current_lights ^= carla.VehicleLightState.Fog
					elif event.key == K_i:
						current_lights ^= carla.VehicleLightState.Interior
					elif event.key == K_z:
						current_lights ^= carla.VehicleLightState.LeftBlinker
					elif event.key == K_x:
						current_lights ^= carla.VehicleLightState.RightBlinker

		if not self._autopilot_enabled:
			if isinstance(self._control, carla.VehicleControl):
				self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
				self._control.reverse = self._control.gear < 0
				# Set automatic control-related vehicle lights
				if self._control.brake:
					current_lights |= carla.VehicleLightState.Brake
				else: # Remove the Brake flag
					current_lights &= ~carla.VehicleLightState.Brake
				if self._control.reverse:
					current_lights |= carla.VehicleLightState.Reverse
				else: # Remove the Reverse flag
					current_lights &= ~carla.VehicleLightState.Reverse
				if current_lights != self._lights: # Change the light state only if necessary
					self._lights = current_lights
					world.player.set_light_state(carla.VehicleLightState(self._lights))
			elif isinstance(self._control, carla.WalkerControl):
				self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)

	def _parse_vehicle_keys(self, keys, milliseconds):
		if keys[K_UP] or keys[K_w]:
			self._control.throttle = min(self._control.throttle + 0.01, 1.00)
		else:
			self._control.throttle = 0.0

		if keys[K_DOWN] or keys[K_s]:
			self._control.brake = min(self._control.brake + 0.2, 1)
		else:
			self._control.brake = 0

		steer_increment = 5e-4 * milliseconds
		if keys[K_LEFT] or keys[K_a]:
			if self._steer_cache > 0:
				self._steer_cache = 0
			else:
				self._steer_cache -= steer_increment
		elif keys[K_RIGHT] or keys[K_d]:
			if self._steer_cache < 0:
				self._steer_cache = 0
			else:
				self._steer_cache += steer_increment
		else:
			self._steer_cache = 0.0
		self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
		self._control.steer = round(self._steer_cache, 1)
		self._control.hand_brake = keys[K_SPACE]

	def _parse_walker_keys(self, keys, milliseconds, world):
		self._control.speed = 0.0
		if keys[K_DOWN] or keys[K_s]:
			self._control.speed = 0.0
		if keys[K_LEFT] or keys[K_a]:
			self._control.speed = .01
			self._rotation.yaw -= 0.08 * milliseconds
		if keys[K_RIGHT] or keys[K_d]:
			self._control.speed = .01
			self._rotation.yaw += 0.08 * milliseconds
		if keys[K_UP] or keys[K_w]:
			self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
		self._control.jump = keys[K_SPACE]
		self._rotation.yaw = round(self._rotation.yaw, 1)
		self._control.direction = self._rotation.get_forward_vector()

	@staticmethod
	def _is_quit_shortcut(key):
		return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================


def game_loop(args):
	"""
	Main loop of the simulation. It handles updating all the HUD information,
	ticking the agent and, if needed, the world.
	"""

	pygame.init()
	pygame.font.init()
	world = None
	traffic = None

	try:
		if args.seed:
			random.seed(args.seed)

		client = carla.Client(args.host, args.port)
		client.set_timeout(4.0)

		traffic_manager = client.get_trafficmanager()
		sim_world = client.get_world()

		if args.sync:
			settings = sim_world.get_settings()
			settings.synchronous_mode = True
			settings.fixed_delta_seconds = 0.05
			sim_world.apply_settings(settings)

			traffic_manager.set_synchronous_mode(True)

		display = pygame.display.set_mode(
			(args.width, args.height),
			pygame.HWSURFACE | pygame.DOUBLEBUF)
		if(args.fullscreen):
			pygame.display.toggle_fullscreen()

		hud = HUD(args.width, args.height)
		world = World(client.get_world(), hud, args)
		control = carla.VehicleControl()
		controller = KeyboardControl(control)
		

		world.camera_manager.toggle_camera

		traffic = Traffic(client, args)

		clock = pygame.time.Clock()

		while True:
			clock.tick()
			if args.sync:
				world.world.tick()
			else:
				world.world.wait_for_tick()
			if controller.parse_events(client, world, clock, args.sync):
				return

			world.tick(clock)
			world.render(display)
			pygame.display.flip()
			if(eCAL_Command.brake != 42):
				control.throttle = eCAL_Command.brake
				control.steer = -eCAL_Command.steering
			world.player.apply_control(control)

	finally:

		if world is not None:
			settings = world.world.get_settings()
			settings.synchronous_mode = False
			settings.fixed_delta_seconds = None
			world.world.apply_settings(settings)
			traffic_manager.set_synchronous_mode(True)

			world.destroy()
		if traffic is not None:
			traffic.destroy()
		if eCAL_Command:
			eCAL_Command.destroy()
		pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
	"""Main method"""

	argparser = argparse.ArgumentParser(
		description='CARLA Automatic Control Client')
	argparser.add_argument(
		'-v', '--verbose',
		action='store_true',
		dest='debug',
		help='Print debug information')
	argparser.add_argument(
		'--host',
		metavar='H',
		default='127.0.0.1',
		help='IP of the host server (default: 127.0.0.1)')
	argparser.add_argument(
		'-p', '--port',
		metavar='P',
		default=2000,
		type=int,
		help='TCP port to listen to (default: 2000)')
	argparser.add_argument(
		'--res',
		metavar='WIDTHxHEIGHT',
		default='1280x720',
		help='Window resolution (default: 1280x720)')
	argparser.add_argument(
		'--fullscreen',
		action='store_true',
		help='start in fullscreen')
	argparser.add_argument(
		'--sync',
		action='store_true',
		help='Synchronous mode execution')
	argparser.add_argument(
		'--filter',
		metavar='PATTERN',
		default='vehicle.carlamotors.firetruck',
		help='Actor filter (default: "vehicle.carlacola")')
	argparser.add_argument(
		'-l', '--loop',
		action='store_true',
		dest='loop',
		help='Sets a new random destination upon reaching the previous one (default: False)')
	argparser.add_argument(
		"-a", "--agent", type=str,
		choices=["Behavior", "Basic"],
		help="select which agent to run",
		default="Behavior")
	argparser.add_argument(
		'-b', '--behavior', type=str,
		choices=["cautious", "normal", "aggressive"],
		help='Choose one of the possible agent behaviors (default: normal) ',
		default='normal')
	argparser.add_argument(
		'-s', '--seed',
		help='Set seed for repeating executions (default: None)',
		default=None,
		type=int)

	# Traffic Params
	argparser.add_argument(
		'--filterv',
		metavar='PATTERN',
		default='vehicle.*',
		help='Filter vehicle model (default: "vehicle.*")')
	argparser.add_argument(
		'--generationv',
		metavar='G',
		default='All',
		help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
	argparser.add_argument(
		'--filterw',
		metavar='PATTERN',
		default='walker.pedestrian.*',
		help='Filter pedestrian type (default: "walker.pedestrian.*")')
	argparser.add_argument(
		'--generationw',
		metavar='G',
		default='2',
		help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
	argparser.add_argument(
		'--hybrid',
		action='store_true',
		help='Activate hybrid mode for Traffic Manager')
	argparser.add_argument(
		'--seedw',
		metavar='S',
		default=0,
		type=int,
		help='Set the seed for pedestrians module')
	argparser.add_argument(
		'--car-lights-on',
		action='store_true',
		default=False,
		help='Enable automatic car light management')
	argparser.add_argument(
		'-n', '--number-of-vehicles',
		metavar='N',
		default=30,
		type=int,
		help='Number of vehicles (default: 30)')
	argparser.add_argument(
		'-w', '--number-of-walkers',
		metavar='W',
		default=10,
		type=int,
		help='Number of walkers (default: 10)')

	args = argparser.parse_args()

	args.width, args.height = [int(x) for x in args.res.split('x')]

	log_level = logging.DEBUG if args.debug else logging.INFO
	logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

	logging.info('listening to server %s:%s', args.host, args.port)

	print(__doc__)

	try:
		game_loop(args)

	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')


if __name__ == '__main__':
	main()
