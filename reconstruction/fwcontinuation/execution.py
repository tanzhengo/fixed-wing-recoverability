"""Timestamped dispatch and remaining-sequence validation.

Rejection is a supervisor event, not a physically executable stop command.
All timestamps in a call must use the same monotonic or simulation clock.
"""
from dataclasses import dataclass, replace
import math
import time
from .types import Control
from .dynamics import step, rollout

@dataclass(frozen=True)
class TimedState:
    state: object
    timestamp: float

@dataclass(frozen=True)
class AppliedControl:
    start: float
    end: float
    control: Control

@dataclass(frozen=True)
class Sequence:
    controls: tuple
    start: float

@dataclass
class DispatchResult:
    accepted: bool
    reason: str
    sequence: object = None
    source: str = ''
    attempts: int = 0
    steps: int = 0
    aligned_state: object = None
    map_version: int = -1
    validation_started: float = 0.0
    validation_finished: float = 0.0

def remaining_controls(sequence,now):
    if not math.isfinite(now) or not math.isfinite(sequence.start) or now<sequence.start:
        raise ValueError('Invalid sequence time')
    elapsed=now-sequence.start; remaining=[]
    for control in sequence.controls:
        if not math.isfinite(control.duration) or control.duration<=0:raise ValueError('Invalid sequence duration')
        if elapsed>=control.duration-1e-12:elapsed=max(0,elapsed-control.duration)
        else:
            remaining.append(replace(control,duration=control.duration-elapsed));elapsed=0.0
    return tuple(remaining)

def align_state(observation,now,history,config):
    if not math.isfinite(now) or not math.isfinite(observation.timestamp) or now<observation.timestamp:
        raise ValueError('Future or invalid observation timestamp')
    state=observation.state;t=observation.timestamp
    # Reject ambiguous overlapping intervals; never invent a command in a gap.
    intervals=sorted(history,key=lambda item:item.start)
    if any(not math.isfinite(i.start+i.end) or i.end<=i.start for i in intervals):
        raise ValueError('Invalid control-history interval')
    if any(a.end>b.start+1e-10 for a,b in zip(intervals,intervals[1:])):
        raise ValueError('Overlapping control history')
    for item in intervals:
        if item.end<=t:continue
        if t>=now-1e-10:break
        if item.start>t+1e-10:raise ValueError('Missing control history')
        end=min(item.end,now)
        while t<end-1e-10:
            dt=min(config.dt,end-t);state=step(state,item.control,dt,method='euler');t+=dt
    if t<now-1e-10:raise ValueError('Missing control history')
    return state

def validate_sequence(state,controls,observed_map,config):
    if not controls:return False,0,'empty_sequence'
    total=0;elapsed=0.0
    for control in controls:
        path=rollout(state,control,config,observed_map,start_time=elapsed,proposal=False)
        total+=path.steps
        if not path.valid:return False,total,path.reason
        state=path.states[-1];elapsed+=control.duration
    return True,total,'accepted'

class ForecastEstimator:
    def __init__(self):self.estimate=.05
    def update(self,duration):
        if not math.isfinite(duration) or duration<0:raise ValueError('Invalid generation duration')
        self.estimate=min(.15,.5*self.estimate+.5*duration)
    def predict(self,state,previous,observed_map,config):
        if previous is None or self.estimate<=0:return state
        path=rollout(state,replace(previous,duration=self.estimate),config,observed_map,proposal=False)
        return path.states[-1] if path.valid else state

def dispatch(observation,proposal,cache,history,map_provider,config,clock=time.monotonic):
    """Revalidate at most twice; map_provider returns a stable snapshot.

    A proposed pair's controls are re-integrated from the aligned actual state.
    Cached controls are trimmed by their original issue time before validation.
    Nothing is published by this function.
    """
    total=0
    for attempt in (1,2):
        start=float(clock());observed_map=map_provider()
        state_age=start-observation.timestamp;map_age=start-observed_map.timestamp
        if not all(math.isfinite(v) for v in (start,state_age,map_age)) or state_age<0 or map_age<0:
            return DispatchResult(False,'invalid_timestamp',attempts=attempt,steps=total)
        if state_age>config.max_state_age:
            return DispatchResult(False,'stale_state',attempts=attempt,steps=total)
        if map_age>config.max_map_age:
            return DispatchResult(False,'stale_map',attempts=attempt,steps=total)
        try:state=align_state(observation,start,history,config)
        except ValueError as error:return DispatchResult(False,str(error),attempts=attempt,steps=total)
        passing=None;source='';reason='no_verified_sequence';version=observed_map.version
        if proposal is not None and proposal.witness is not None:
            witness=proposal.witness
            controls=(witness.candidate.control,witness.control)
            ok,count,reason=validate_sequence(state,controls,observed_map,config);total+=count
            if ok:passing=controls;source='fresh'
        if passing is None and cache is not None:
            try:controls=remaining_controls(cache,start)
            except ValueError:controls=()
            ok,count,reason=validate_sequence(state,controls,observed_map,config);total+=count
            if ok:passing=controls;source='cached'
        end=float(clock())
        if not math.isfinite(end) or end<start:
            return DispatchResult(False,'invalid_clock',attempts=attempt,steps=total)
        if observed_map.version!=version:
            reason='map_changed_during_validation'
            if attempt==1:continue
            return DispatchResult(False,reason,attempts=attempt,steps=total)
        if end-start>config.max_validation_age:
            if attempt==1:continue
            return DispatchResult(False,'validation_overrun',attempts=attempt,steps=total,
                                  validation_started=start,validation_finished=end)
        if passing is None:return DispatchResult(False,reason,attempts=attempt,steps=total)
        # Validation wall time is not execution time for a newly issued pair.
        # For cached controls, retain their original expiry instead of extending
        # the old sequence by the time spent validating it.
        if source=='cached':
            passing=remaining_controls(cache,end)
            if not passing:return DispatchResult(False,'cached_sequence_expired',attempts=attempt,steps=total)
        return DispatchResult(True,'accepted',Sequence(passing,end),source,attempt,total,state,version,start,end)
    raise AssertionError('Unreachable')
