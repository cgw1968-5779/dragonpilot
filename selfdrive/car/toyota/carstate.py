import math
#from math import floor
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR
from common.travis_checker import travis
from common.op_params import opParams

op_params = opParams()
set_speed_offset = op_params.get('set_speed_offset')

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']

    # initial states
    self.engineRPM = 0
    self.headlightON = 0
    self.parkingLightON = 0

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = False
    self.angle_offset = 0.
    self.setspeedcounter = 0
    self.pcm_acc_active = False
    self.main_on = False
    self.v_cruise_pcmactivated = False
    self.v_cruise_pcmlast = 0
    self.setspeedoffset = 34
    self.setspeedcounter = 0

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
                        cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED'] != 0

    ret.brakePressed = (cp.vl["BRAKE_MODULE"]['BRAKE_PRESSED'] != 0) or not bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.brakeLights = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["GAS_PEDAL"]['GAS_PEDAL']
      ret.gasPressed = cp.vl["PCM_CRUISE"]['GAS_RELEASED'] == 0

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001

    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
    if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      ret.steeringAngleDeg = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset
      if self.needs_angle_offset:
        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']
        if abs(angle_wheel) > 1e-3:
          self.needs_angle_offset = False
          self.angle_offset = ret.steeringAngleDeg - angle_wheel
    else:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    can_gear = int(cp.vl["GEAR_PACKET"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2

    ret.parkingLightON = cp.vl["LIGHT_STALK"]['PARKING_LIGHT'] == 1
    ret.headlightON = cp.vl["LIGHT_STALK"]['LOW_BEAM'] == 1
    self.engineRPM = cp.vl["ENGINE_RPM"]['RPM']

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]

    if self.CP.carFingerprint == CAR.LEXUS_IS:
      self.main_on = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = False
    else:
      self.main_on = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
      ret.cruiseState.available = self.main_on

     ####################
     ## arne + - 5 mph ##
     ####################

    #print("ret.cruiseState.speed =" + str(ret.cruiseState.speed))
    if self.CP.carFingerprint in TSS2_CAR:
      minimum_set_speed = 27
    elif self.CP.carFingerprint == CAR.RAV4:
      minimum_set_speed = 44
    else:
      minimum_set_speed = 41
    maximum_set_speed = 169
    if self.CP.carFingerprint == CAR.LEXUS_RXH:
      maximum_set_speed = 177
    v_cruise_pcm_max = ret.cruiseState.speed
    if v_cruise_pcm_max < minimum_set_speed and self.pcm_acc_active:
      #print("Min set speed changed. Was " + str(minimum_set_speed) + ", now " + str(v_cruise_pcm_max))
      minimum_set_speed = v_cruise_pcm_max
    if v_cruise_pcm_max > maximum_set_speed and self.pcm_acc_active:
      #print("Max set speed changed. Was " + str(maximum_set_speed) + ", now " + str(v_cruise_pcm_max))
      maximum_set_speed = v_cruise_pcm_max
    speed_range = maximum_set_speed - minimum_set_speed
    #if self.v_cruise_pcmactivated:
      #print("self.v_cruise_pcmlast after activated = " + str(self.v_cruise_pcmlast))
      #print("ret.cruiseState.speed  after activated = " + str(ret.cruiseState.speed))
    if (self.v_cruise_pcmactivated or (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and not
                                       self.pcm_acc_active)) and self.v_cruise_pcmlast != ret.cruiseState.speed:
      #print("Engage with different speed than before")
      if ret.vEgo * CV.MS_TO_KPH < minimum_set_speed:
        #print("speed lower than min_set_speed")
        self.setspeedoffset = max(min(int(minimum_set_speed - ret.vEgo * CV.MS_TO_KPH),(minimum_set_speed-7.0)),0.0)
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        self.v_cruise_pcmlast = ret.cruiseState.speed
      else:
        #print("speed is higher than min_set_speed")
        self.setspeedoffset = 0
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        self.v_cruise_pcmlast = ret.cruiseState.speed
    if ret.cruiseState.speed < self.v_cruise_pcmlast and (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and self.pcm_acc_active):
      #print("Speed lowered")
      if self.setspeedcounter > 0 and ret.cruiseState.speed > minimum_set_speed:
        self.setspeedoffset = self.setspeedoffset + 4
        #print("Speed lowered by 5")
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      else:
        if math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed-7.0)/speed_range
                           + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                       - self.setspeedoffset)/(ret.cruiseState.speed - (minimum_set_speed-1.0))) > 0:
          self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed - 7.0)/speed_range
                                                                      + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                                                                  - self.setspeedoffset)/(ret.cruiseState.speed - (minimum_set_speed - 1.0)))
          #print("Speed lowered, self.setspeedoffset is now " + str(self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      self.setspeedcounter = 50
    if self.v_cruise_pcmlast < ret.cruiseState.speed and (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and self.pcm_acc_active):
      #print("Speed raised")
      if self.setspeedcounter > 0 and (self.setspeedoffset - 4) > 0:
        #print("Speed raised by 5")
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
        self.setspeedoffset = self.setspeedoffset - 4
      else:
        self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed) * (minimum_set_speed - 7.0)/speed_range
                                                                    + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                                                                - self.setspeedoffset)/(maximum_set_speed + 1.0 - ret.cruiseState.speed))
        #print("Speed raised, self.setspeedoffset is now " + str(self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      self.setspeedcounter = 50
    if self.setspeedcounter > 0:
      self.setspeedcounter = self.setspeedcounter - 1
    if bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and not self.pcm_acc_active:
      #print("self.v_cruise_pcmlast on activated = " + str(self.v_cruise_pcmlast))
      #print("ret.cruiseState.speed  on activated = " + str(ret.cruiseState.speed))
      self.v_cruise_pcmactivated = True
    else:
      self.v_cruise_pcmactivated = False
    self.v_cruise_pcmlast = ret.cruiseState.speed
    if ret.cruiseState.speed - self.setspeedoffset < 7:
      #print("Set speed lower than 7 kph.")
      self.setspeedoffset = ret.cruiseState.speed - 7
      #print("self.setspeedoffset = " + str (self.setspeedoffset))
    if ret.cruiseState.speed - self.setspeedoffset > maximum_set_speed:
      #print("Set speed higher than max_set_speed")
      self.setspeedoffset = ret.cruiseState.speed - maximum_set_speed
      #print("self.setspeedoffset = " + str (self.setspeedoffset))

    if set_speed_offset or travis:
      self.setspeedoffset = 0.0

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    self.pcm_acc_active = bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.cruiseState.enabled = self.pcm_acc_active

    if self.CP.carFingerprint == CAR.PRIUS:
      ret.genericToggle = cp.vl["AUTOPARK_STATUS"]['STATE'] != 0
    else:
      ret.genericToggle = bool(cp.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["ESP_CONTROL"]['TC_DISABLED'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']

    if self.CP.carFingerprint in TSS2_CAR:
      ret.leftBlindspot = (cp.vl["BSM"]['L_ADJACENT'] == 1) or (cp.vl["BSM"]['L_APPROACHING'] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]['R_ADJACENT'] == 1) or (cp.vl["BSM"]['R_APPROACHING'] == 1)

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("BRAKE_PRESSED", "BRAKE_MODULE", 0),
      ("GAS_PEDAL", "GAS_PEDAL", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("RPM", "ENGINE_RPM", 0),
      ("DOOR_OPEN_FL", "SEATS_DOORS", 1),
      ("DOOR_OPEN_FR", "SEATS_DOORS", 1),
      ("DOOR_OPEN_RL", "SEATS_DOORS", 1),
      ("DOOR_OPEN_RR", "SEATS_DOORS", 1),
      ("SEATBELT_DRIVER_UNLATCHED", "SEATS_DOORS", 1),
      ("TC_DISABLED", "ESP_CONTROL", 1),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
      ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
      ("CRUISE_ACTIVE", "PCM_CRUISE", 0),
      ("CRUISE_STATE", "PCM_CRUISE", 0),
      ("GAS_RELEASED", "PCM_CRUISE", 1),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 3),   # 3 is no blinkers
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
      ("PARKING_LIGHT", "LIGHT_STALK", 0),
      ("LOW_BEAM", "LIGHT_STALK", 0),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
    ]

    checks = [
      ("BRAKE_MODULE", 40),
      ("GAS_PEDAL", 33),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
      ("EPS_STATUS", 25),
    ]

    if CP.carFingerprint == CAR.LEXUS_IS:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    if CP.carFingerprint in TSS2_CAR:
      signals += [("L_ADJACENT", "BSM", 0)]
      signals += [("L_APPROACHING", "BSM", 0)]
      signals += [("R_ADJACENT", "BSM", 0)]
      signals += [("R_APPROACHING", "BSM", 0)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0)
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
