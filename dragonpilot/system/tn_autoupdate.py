#!/usr/bin/env python3
# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Ported from the owner's sunnypilot branch tnpb3 (openpilot/sunnypilot/system/tn_autoupdate.py) to dragonpilot.
"""
Drive-gear auto update (owner 2026-10-11): "update automatically when the car is in D with MAIN ACC off and the device
has a connection - I am an expert user, I accept the risk", restarting with `sudo systemctl restart comma` (not a full
reboot) so openpilot is back quickly.

The stock updater only runs offroad and installs at the next boot, and the C3X rarely reboots, so the owner has been
updating by hand over SSH (fetch + reset --hard + restart). This process does the same thing on its own:
  - every FETCH_PERIOD it fetches the checked-out branch (UpdaterTargetBranch only on a detached HEAD) from origin, and
    right away (then every DRIVE_FETCH_PERIOD) once the car is in D with MAIN ACC off - owner: "after MAIN ACC goes
    off, update within 30 s, not after 5 minutes";
  - when the fetched commit differs from HEAD, the working tree is clean, and the car has been in D with MAIN ACC off and
    openpilot not engaged for HOLD_TIME, it resets the checkout to that commit, drops any update the stock updater has
    staged (the launch script would otherwise swap it in), and restarts the comma service.
While openpilot restarts its steering / ACC messages stop and the car may show a system warning until the next
ignition cycle. DisableUpdates=1 turns it off.

dragonpilot: this tree has no prebuilt marker, so after the restart launch_chffrplus.sh runs build.py first; scons only
rebuilds what the new commit changed (seconds for Python-only commits, longer when C++ changed) before manager starts.

OFF unless ENABLE_FLAG exists (owner: "I do not recommend this to other users - anyone who installs this branch gets it
off; only on my own device, switched on once by the AI"). A file under /data, not a Params key: a new key has to be
registered in params_keys.h (C++ rebuild), and the flag must survive switching branches. Create it once on the owner's device over SSH
(`touch /data/tn_autoupdate_enabled`); remove it to turn the feature off. Checked every cycle, no restart needed.
"""
import datetime
import os
import subprocess
import shutil
import time

from cereal import car
import cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

FETCH_PERIOD = 60.  # s (owner 2026-10-16: fetch every minute, 5 minutes is too long)
FIRST_FETCH_DELAY = 30.  # s after start
DRIVE_FETCH_PERIOD = 20.  # s between fetches while in D with MAIN ACC off (the first one is immediate)
FETCH_TIMEOUT = 120.  # s
HOLD_TIME = 3.0  # s the drive conditions must hold before the restart
STAGING_FINALIZED = "/data/safe_staging/finalized"
RESTART_CMD = ["sudo", "systemctl", "restart", "comma"]
ENABLE_FLAG = "/data/tn_autoupdate_enabled"


def enabled() -> bool:
  return os.path.isfile(ENABLE_FLAG)


# P or D (owner 2026-10-11, routes 00000128/129: the test was done standing in P - D lasted 2 s - and nothing happened;
# "add P"). R and N never.
INSTALL_GEARS = (car.CarState.GearShifter.park, car.CarState.GearShifter.drive)


def why_not_now(gear, cruise_available: bool, engaged: bool) -> str:
  """'' when the car is in a state to install, else the reason"""
  if gear not in INSTALL_GEARS:
    return f"gear {gear}"
  if cruise_available:
    return "MAIN ACC on"
  if engaged:
    return "engaged"
  return ""


def drive_gear_main_off(gear, cruise_available: bool, engaged: bool) -> bool:
  return why_not_now(gear, cruise_available, engaged) == ""


def git(*args, timeout=30.) -> str:
  return subprocess.check_output(["git", "-C", BASEDIR, *args], text=True, timeout=timeout,
                                 stderr=subprocess.STDOUT).strip()


class DriveGearUpdater:
  def __init__(self, params: Params):
    self.params = params
    self.remote_sha: str | None = None
    self.next_fetch = time.monotonic() + FIRST_FETCH_DELAY
    self.last_fetch = -1e9
    self.hold_t = 0.0
    self.fetch_error = "not fetched yet"
    self.reason = ""

  def branch(self) -> str:
    # the branch the device is running, not UpdaterTargetBranch: owner 2026-10-11 switched tnpb3 -> tnpb2 by hand with the
    # param still naming tnpb3, and following the param would have installed tnpb3 again on the next drive
    head = git("rev-parse", "--abbrev-ref", "HEAD")
    return head if head and head != "HEAD" else (self.params.get("UpdaterTargetBranch") or head)

  def fetch(self) -> None:
    try:
      b = self.branch()
      git("fetch", "--quiet", "origin", f"+{b}:refs/remotes/origin/{b}", timeout=FETCH_TIMEOUT)
      self.remote_sha = git("rev-parse", f"refs/remotes/origin/{b}")
      self.fetch_error = ""
    except Exception as e:  # no connection, timeout, branch gone: try again next period
      self.fetch_error = f"fetch failed: {str(e)[-120:]}"

  def why_not_pending(self) -> str:
    """'' when a new commit can be installed, else the reason"""
    if not self.remote_sha:
      return self.fetch_error or "no remote commit"
    try:
      if self.remote_sha == git("rev-parse", "HEAD"):
        return "up to date" + (f" ({self.fetch_error})" if self.fetch_error else "")
      dirty = git("status", "--porcelain", "--untracked-files=no")
      return f"local changes: {' '.join(dirty.split())[:120]}" if dirty else ""
    except Exception as e:
      return f"git error: {str(e)[-120:]}"

  def pending(self) -> bool:
    return self.why_not_pending() == ""

  def log_reason(self, reason: str) -> None:
    # every change of the reason goes to the log (errorLogMessage, kept in the qlog) so an upload shows why it waited
    if reason != self.reason:
      self.reason = reason
      cloudlog.event("tn_autoupdate: state", reason=reason, remote=(self.remote_sha or "")[:7], error=True)

  def install(self) -> None:
    sha = self.remote_sha
    cloudlog.event("tn_autoupdate: installing", sha=sha, error=True)
    b = self.branch()
    git("checkout", "-q", "-B", b, sha, timeout=120.)
    shutil.rmtree(STAGING_FINALIZED, ignore_errors=True)
    self.params.put_bool("UpdateAvailable", False)
    subprocess.run(["sync"], timeout=30)
    # systemd stops and restarts every openpilot process, this one included
    subprocess.Popen(RESTART_CMD, start_new_session=True)

  def step(self, dt: float, condition: bool, why: str = "") -> bool:
    """returns True when it started an install; why = the reason the car state is not right (when condition is False)"""
    now = time.monotonic()
    if not enabled():
      self.hold_t = 0.0
      return False
    if self.params.get_bool("DisableUpdates"):
      self.hold_t = 0.0
      self.log_reason("DisableUpdates")
      return False
    if now >= self.next_fetch or (condition and now - self.last_fetch >= DRIVE_FETCH_PERIOD):
      self.next_fetch, self.last_fetch = now + FETCH_PERIOD, now
      self.fetch()
    self.hold_t = self.hold_t + dt if condition else 0.0
    if not condition:
      self.log_reason(why or "car state")
      return False
    reason = self.why_not_pending()
    if reason:
      self.log_reason(reason)
      return False
    if self.hold_t < HOLD_TIME:
      self.log_reason("holding")
      return False
    self.log_reason("installing")
    self.install()
    return True


def current_description() -> str:
  """same text as the stock updater's UpdaterCurrentDescription: version / branch / commit / date"""
  with open(os.path.join(BASEDIR, "common", "version.h")) as f:
    version = f.read().split('"')[1]
  branch = git("rev-parse", "--abbrev-ref", "HEAD")
  commit = git("rev-parse", "--short=7", "HEAD")
  date = datetime.datetime.fromtimestamp(int(git("show", "-s", "--format=%ct", "HEAD"))).strftime("%b %d")
  return f"{version} / {branch} / {commit} / {date}"


def main():
  params = Params()
  # Settings -> Software "Current Version" shows UpdaterCurrentDescription, which the stock updater refreshes only offroad;
  # after an install in D it would still name the old commit. Refresh it at every start (owner 2026-10-11: "the
  # software tab does not show which commit is running").
  try:
    params.put("UpdaterCurrentDescription", current_description())
  except Exception as e:
    cloudlog.warning(f"tn_autoupdate: description failed: {e}")
  # conflated sockets: read once a second, always the newest message
  socks = {s: messaging.sub_sock(s, conflate=True) for s in ('carState', 'selfdriveState')}
  last: dict = {}
  last_t: dict = {}
  updater = DriveGearUpdater(params)
  while True:
    now = time.monotonic()
    for s, sock in socks.items():
      msg = messaging.recv_one_or_none(sock)
      if msg is not None:
        last[s], last_t[s] = getattr(msg, s), now
    alive = all(now - last_t.get(s, -1e9) < 2.0 for s in socks)
    why = why_not_now(last['carState'].gearShifter, last['carState'].cruiseState.available,
                      last['selfdriveState'].enabled) if alive else "offroad / no carState"
    if updater.step(1.0, why == "", why):
      time.sleep(60.)  # the restart takes this process down
    time.sleep(1.0)


if __name__ == "__main__":
  main()
