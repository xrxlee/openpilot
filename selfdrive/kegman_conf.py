import json
import os

class kegman_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['tuneGernby'] != "1":
      self.conf['tuneGernby'] = str(1)
      write_conf = True
    if self.conf['Kp'] == "-1":
      self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
      write_conf = True
    if self.conf['Ki'] == "-1":
      self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
      write_conf = True
    if self.conf['Kf'] == "-1":
      self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
      write_conf = True
    if self.conf['steerRatio'] == "-1":
      self.conf['steerRatio'] = str(round(CP.steerRatio,3))
      write_conf = True
    if self.conf['steerRateCost'] == "-1":
      self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/kegman.json'):
      with open('/data/kegman.json', 'r') as f:
        self.config = json.load(f)

      if "battPercOff" not in self.config:
        self.config.update({"battPercOff":"25"})
        self.config.update({"carVoltageMinEonShutdown":"11800"})
        self.config.update({"brakeStoppingTarget":"0.25"})
        self.element_updated = True

      if "tuneGernby" not in self.config:
        self.config.update({"tuneGernby":"1"})
        self.config.update({"Kp":"-1"})
        self.config.update({"Ki":"-1"})
	self.element_updated = True

      if "liveParams" not in self.config:
        self.config.update({"liveParams":"1"})
        self.element_updated = True
	
      if "steerRatio" not in self.config:
	self.config.update({"steerRatio":"-1"})
	self.config.update({"steerRateCost":"-1"})
	self.element_updated = True
	
      if "leadDistance" not in self.config:
        self.config.update({"leadDistance":"5"})
        self.element_updated = True
	
      if "deadzone" not in self.config:
        self.config.update({"deadzone":"0.0"})
        self.element_updated = True
	
      if "1barBP0" not in self.config:
        self.config.update({"1barBP0":"-0.25"})
	self.config.update({"1barBP1":"2.75"})
	self.config.update({"2barBP0":"-0.25"})
	self.config.update({"2barBP1":"3.0"})
	self.config.update({"3barBP0":"0.0"})
	self.config.update({"3barBP1":"4.0"})
        self.element_updated = True


      if "1barMax" not in self.config:
        self.config.update({"1barMax":"2.5"})
	self.config.update({"2barMax":"2.5"})
	self.config.update({"3barMax":"2.5"})
        self.element_updated = True
	
      if "slowOnCurves" not in self.config:
        self.config.update({"slowOnCurves":"0"})
        self.element_updated = True
	
      if "Kf" not in self.config:
        self.config.update({"Kf":"-1"})
        self.element_updated = True

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"cameraOffset":"0.06", "lastTrMode":"1", "battChargeMin":"60", "battChargeMax":"70", \
                     "wheelTouchSeconds":"180", "battPercOff":"25", "carVoltageMinEonShutdown":"11800", \
                     "brakeStoppingTarget":"0.25", "tuneGernby":"1", \
                     "Kp":"-1", "Ki":"-1", "liveParams":"1", "leadDistance":"5", "deadzone":"0.0", \
		     "1barBP0":"-0.25", "1barBP1":"2.75", "2barBP0":"-0.25", "2barBP1":"3.0", "3barBP0":"0.0", \
		     "3barBP1":"4.0", "1barMax":"2.5", "2barMax":"2.5", "3barMax":"2.5", \
		     "steerRatio":"-1", "steerRateCost":"-1", "slowOnCurves":"0", "Kf":"-1"}

      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
