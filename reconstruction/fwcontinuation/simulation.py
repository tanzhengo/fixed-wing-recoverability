"""Recorded synchronous diagnostics; not historical, real-time or Gazebo flight.

Physical geometry is supplied only to the ideal sensor and independent truth
evaluator. The planner sees an initially empty accumulated ObservedMap. CPU
work does NOT advance the simulation clock: dispatch has zero computational
latency. This intentionally cannot establish asynchronous deadline behavior.

The plant below separately implements the nine point-mass equations and RK4,
without calling the planning integrator. It is numerically independent, not an
independent aerodynamic model. World-from-body sensor pose uses inherited
plugin convention roll=-phi, pitch=-(gamma+alpha), yaw=psi, xyzw quaternion.
"""
from contextlib import ExitStack
from dataclasses import asdict,dataclass,fields,replace
import json
import math
from pathlib import Path
import time
import numpy as np

from .types import State,Control,Config
from .observed_map import ObservedMap
from .sensor import Sphere,Cylinder,Pose,lidar_directions,trace_scan
from .planner import plan,MODES
from .execution import dispatch,TimedState,AppliedControl
from .records import result_record,canonical_hash,source_hashes,write_json


@dataclass(frozen=True)
class SimulationConfig:
    duration: float=40.0
    control_interval: float=0.2
    initialization_seconds: float=2.0
    sensor_hz: float=5.0
    azimuth_count: int=512
    elevation_count: int=32
    min_range: float=1.0
    max_range: float=120.0
    condition: str="nominal"
    horizontal_goal_tolerance: float=15.0
    altitude_goal_tolerance: float=3.0

    def __post_init__(self):
        if self.condition not in ("nominal","crosswind","model_mismatch","delay100ms"):
            raise ValueError("Unsupported execution condition")
        numeric=[v for k,v in asdict(self).items() if k!="condition"]
        if not all(math.isfinite(v) for v in numeric):raise ValueError("Nonfinite simulation configuration")
        if min(self.duration,self.control_interval,self.sensor_hz,self.max_range,
               self.horizontal_goal_tolerance,self.altitude_goal_tolerance)<=0:
            raise ValueError("Simulation durations/rates/tolerances must be positive")
        if self.initialization_seconds<0 or not 0<=self.min_range<self.max_range:
            raise ValueError("Invalid initialization or sensor range")
        if any(type(v) is not int or v<1 for v in (self.azimuth_count,self.elevation_count)):
            raise ValueError("Sensor ray counts must be positive integers")
        if self.condition=="delay100ms" and self.control_interval<0.1:
            raise ValueError("100 ms delay requires a control interval at least 0.1 s")


def pose_from_state(state,timestamp):
    roll,pitch,yaw=-state.phi,-(state.gamma+state.alpha),state.psi
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    q=(sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
       cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)
    return Pose(timestamp,state.position,q)


def _rates(v,control,condition):
    x,y,h,speed,gamma,psi,phi,alpha,thrust=v
    if not np.isfinite(v).all() or speed<=0 or abs(math.cos(gamma))<1e-10:
        raise ValueError("Singular/nonfinite execution dynamics")
    lift_scale,drag_scale,actuator_scale=(.97,1.10,1.25) if condition=="model_mismatch" else (1.,1.,1.)
    normal=thrust*math.sin(alpha)+1.86*speed*speed*(.03*math.sin(alpha)+.028)*lift_scale
    wind_y=1.0 if condition=="crosswind" else 0.0
    return np.array([
        speed*math.cos(gamma)*math.cos(psi),
        speed*math.cos(gamma)*math.sin(psi)+wind_y,speed*math.sin(gamma),
        thrust*math.cos(alpha)-1.86*speed*speed*(.001*alpha*alpha+.002)*drag_scale-9.81*math.sin(gamma),
        (normal*math.cos(phi)-9.81*math.cos(gamma))/speed,
        normal*math.sin(phi)/(speed*math.cos(gamma)),
        2*(control.phi-phi)/actuator_scale,(control.alpha-alpha)/actuator_scale,
        10*(control.T-thrust)/actuator_scale],dtype=float)


def plant_step(state,control,dt,condition="nominal"):
    """Independent RK4 numerical plant; delay is applied by the command loop."""
    if condition not in ("nominal","crosswind","model_mismatch","delay100ms"):
        raise ValueError("Unsupported execution condition")
    if not math.isfinite(dt) or dt<=0:raise ValueError("Invalid plant step")
    if not all(math.isfinite(v) for v in asdict(control).values()):raise ValueError("Nonfinite control")
    names=("x","y","h","V","gamma","psi","phi","alpha","T")
    v=np.array([getattr(state,k) for k in names],dtype=float)
    k1=_rates(v,control,condition);k2=_rates(v+.5*dt*k1,control,condition)
    k3=_rates(v+.5*dt*k2,control,condition);k4=_rates(v+dt*k3,control,condition)
    out=v+dt*(k1+2*k2+2*k3+k4)/6
    if not np.isfinite(out).all():raise ValueError("Nonfinite integrated execution state")
    out[5]=math.atan2(math.sin(out[5]),math.cos(out[5]))
    yaw_rate=float(_rates(out,control,condition)[5])
    return State(**dict(zip(names,out.tolist())),yaw_rate=yaw_rate)


def sampled_clearance(position,obstacles,ground_height=None):
    """Euclidean signed clearance at one reference-point sample; not a tube."""
    p=np.array(position,dtype=float);values=[]
    for obj in obstacles:
        delta=p-np.array(obj.center)
        if isinstance(obj,Sphere):values.append(float(np.linalg.norm(delta))-obj.radius)
        elif isinstance(obj,Cylinder):
            radial=math.hypot(delta[0],delta[1])-obj.radius
            vertical=abs(delta[2])-obj.height/2
            values.append(math.hypot(max(radial,0),max(vertical,0))+min(max(radial,vertical),0))
        else:raise TypeError("Unsupported truth geometry")
    if ground_height is not None:values.append(float(p[2]-ground_height))
    return min(values) if values else None


def segment_contacts_geometry(a,b,obstacles,ground_height=None):
    """Exact analytic chord contact for spheres/upright finite cylinders.

    This tests recorded RK4 chords, not the nonlinear trajectory between them.
    """
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);delta=b-a
    if ground_height is not None and min(a[2],b[2])<=ground_height:return True
    for obj in obstacles:
        q=a-np.asarray(obj.center)
        if isinstance(obj,Sphere):
            norm=float(delta@delta)
            t=float(np.clip(-(q@delta)/norm,0,1)) if norm>0 else 0.
            if float(np.linalg.norm(q+t*delta))<=obj.radius+1e-12:return True
        elif isinstance(obj,Cylinder):
            # Intersect parameter intervals for the closed horizontal disk and
            # vertical slab. This does not use sensor ray-intersection code.
            aa=float(delta[:2]@delta[:2]);bb=float(q[:2]@delta[:2])
            cc=float(q[:2]@q[:2])-obj.radius**2
            if aa<=1e-24:
                if cc>0:continue
                low,high=0.,1.
            else:
                disc=bb*bb-aa*cc
                if disc< -1e-12:continue
                root=math.sqrt(max(0.,disc))
                low,high=max(0.,(-bb-root)/aa),min(1.,(-bb+root)/aa)
            if abs(delta[2])<=1e-15:
                if abs(q[2])>obj.height/2:continue
            else:
                z0=(-obj.height/2-q[2])/delta[2];z1=(obj.height/2-q[2])/delta[2]
                low=max(low,min(z0,z1));high=min(high,max(z0,z1))
            if low<=high+1e-12:return True
        else:raise TypeError("Unsupported truth geometry")
    return False


def _envelope(state,config):
    return (all(math.isfinite(v) for v in asdict(state).values()) and
        config.min_speed<=state.V<=config.max_speed and config.min_altitude<=state.h<=config.max_altitude and
        abs(state.phi)<=config.max_roll and abs(state.gamma)<=config.max_gamma and
        config.min_alpha<=state.alpha<=config.max_alpha and 0<=state.T<=config.max_thrust)


def parse_scene(scene):
    """Require caller-supplied geometry, full initial state, and a single goal."""
    for field in ("scene_id","start","goal","obstacles"):
        if field not in scene:raise ValueError("Scene requires "+field)
    if not isinstance(scene['scene_id'],str) or not scene['scene_id']:
        raise ValueError("scene_id must be a nonempty string")
    required={f.name for f in fields(State)}-{'yaw_rate'}
    if not required.issubset(scene['start']):
        raise ValueError("start must specify all nine physical State fields; supply a genuine initial/trim state")
    state=State(**scene['start'])
    goal=tuple(float(v) for v in scene['goal'])
    if len(goal)!=3 or not all(math.isfinite(v) for v in goal):raise ValueError("Invalid goal")
    if not all(math.isfinite(v) for v in asdict(state).values()):raise ValueError("Nonfinite initial state")
    obstacles=[]
    for obj in scene['obstacles']:
        if obj.get('type')=='sphere':obstacles.append(Sphere(tuple(obj['center']),obj['radius']))
        elif obj.get('type')=='cylinder':obstacles.append(Cylinder(tuple(obj['center']),obj['radius'],obj['height']))
        else:raise ValueError("Only sphere/cylinder scene obstacles are supported")
    ground=scene.get('ground_height')
    if ground is not None and not math.isfinite(ground):raise ValueError("Invalid ground height")
    return state,goal,tuple(obstacles),ground


def run_episode(scene,output_dir,config=None,simulation=None,mode='diverse',model=None,model_hash=None):
    """Run one supplied scene and record every query, including failures.

    Initialization continues the supplied initial actuator commands for at most
    two seconds while scans accumulate. This is not a verified fallback. A
    rejection after initialization terminates recording; it is not an aircraft
    stop/recovery maneuver. No automatic scene generation or campaign is used.
    """
    config=config or Config();simulation=simulation or SimulationConfig()
    if mode not in MODES:raise ValueError("Unknown schedule")
    if mode in ('learned','combined') and (model is None or not getattr(model,'is_trained',False)):
        raise ValueError("A trained loaded model is required for learned schedules")
    state,goal,truth,ground=parse_scene(scene)
    output=Path(output_dir)
    if output.exists() and any(output.iterdir()):raise FileExistsError("Refusing to overwrite episode records")
    output.mkdir(parents=True,exist_ok=True)
    observed=ObservedMap(config.voxel_size)
    directions=lidar_directions(simulation.azimuth_count,simulation.elevation_count)
    start_state=state;clock_time=0.;next_scan=0.;query_number=0;scan_number=0
    cache=None;history=[];previous=None;ever_accepted=False
    # Explicit initialization policy: retain the initially supplied actuator
    # values. Task-cost previous-command term stays inactive before acceptance.
    requested_previous=Control(state.phi,state.alpha,state.T,config.H)
    initial_clearance=sampled_clearance(state.position,truth,ground)
    minimum_clearance=initial_clearance;physical_contacts=0
    sampled_margin=initial_clearance is not None and initial_clearance<config.margin
    outcome='time_limit';total_primary=0;total_successor=0;total_dispatch=0
    fresh=0;cached=0;initialization_waits=0
    provenance={'schema':'fwcontinuation.reconstruction.episode.v1',
        'evidence_kind':'new_synchronous_numerical_diagnostic','historical_result_reproduced':False,
        'clock_policy':'SIMULATION_CLOCK_ZERO_COMPUTATIONAL_LATENCY',
        'asynchronous_realtime_evidence':False,'scene_sha256':canonical_hash(scene),
        'source_sha256':source_hashes(),'model_sha256':model_hash,'mode':mode,
        'config':asdict(config),'simulation':asdict(simulation),
        'sensor_pose_convention':'world_from_body xyzw; roll=-phi,pitch=-(gamma+alpha),yaw=psi',
        'truth_access':'sensor and independent evaluator only; planner receives accumulated observed voxels',
        'clearance_scope':'Euclidean physical reference-point clearance sampled at every RK4 step; minimum between samples is not certified',
        'contact_scope':'analytic physical contact of chords between RK4 samples; no airframe/tracking tube',
        'initialization_policy':'hold supplied initial actuator values until a sequence passes, up to initialization_seconds'}
    write_json(output/'manifest.json',provenance)
    write_json(output/'scene.json',scene)
    write_json(output/'initial_map.json',observed.snapshot())
    with ExitStack() as stack:
        streams={k:stack.enter_context((output/(k+'.jsonl')).open('x',encoding='utf-8'))
                 for k in ('frames','queries','states','scans','controls')}
        def log(name,value):streams[name].write(json.dumps(value,allow_nan=False,separators=(',',':'))+'\n')
        def record_scan():
            nonlocal scan_number,next_scan
            pose=pose_from_state(state,clock_time)
            beams=trace_scan(pose,directions,truth,min_range=simulation.min_range,
                             max_range=simulation.max_range,ground_height=ground)
            observed.integrate_scan(beams,clock_time)
            log('scans',{'scan_id':scan_number,'timestamp':clock_time,'pose':asdict(pose),
                         'beams':[asdict(b) for b in beams],'map_version':observed.version})
            scan_number+=1;next_scan=scan_number/simulation.sensor_hz
        log('states',{'time':0.,'state':asdict(state),'sampled_physical_clearance':initial_clearance})
        while clock_time<simulation.duration-1e-10:
            if clock_time>=next_scan-1e-10:record_scan()
            frame={'scene_id':scene['scene_id'],'query_id':str(query_number),
                   'time':clock_time,'state':asdict(state),'goal':list(goal),
                   'config':asdict(config),'previous':asdict(previous) if previous else None,
                   'map':observed.snapshot(),'evidence_kind':provenance['evidence_kind']}
            log('frames',frame)
            # Immutable observed-map input; physical truth is never passed here.
            snapshot=ObservedMap.from_snapshot(frame['map'])
            wall_start=time.perf_counter()
            result=plan(state,goal,snapshot,config,mode,model,previous)
            proposal_wall=time.perf_counter()-wall_start
            decision=dispatch(TimedState(state,clock_time),result,cache,history,
                              lambda:snapshot,config,clock=lambda:clock_time)
            total_primary+=result.primary_steps;total_successor+=result.steps;total_dispatch+=decision.steps
            entry=result_record(frame,result,mode,model_hash)
            entry.update(time=clock_time,proposal_wall_seconds_diagnostic=proposal_wall,
                computational_latency_applied_seconds=0.0,dispatch={
                    'accepted':decision.accepted,'reason':decision.reason,'source':decision.source,
                    'attempts':decision.attempts,'steps':decision.steps})
            query_number+=1
            # Evaluate the supplied initial/current truth even if a proposal is
            # rejected. A planner failure must not hide an existing contact.
            if segment_contacts_geometry(state.position,state.position,truth,ground):
                outcome='physical_contact';physical_contacts+=1
                entry['execution_action']='terminate_physical_contact';log('queries',entry);break
            if not _envelope(state,config):
                outcome='envelope_violation';entry['execution_action']='terminate_envelope_violation'
                log('queries',entry);break
            if math.hypot(state.x-goal[0],state.y-goal[1])<simulation.horizontal_goal_tolerance and abs(state.h-goal[2])<simulation.altitude_goal_tolerance:
                outcome='goal';entry['execution_action']='goal_already_reached';log('queries',entry);break
            if decision.accepted:
                ever_accepted=True;cache=decision.sequence;command=cache.controls[0]
                fresh+=decision.source=='fresh';cached+=decision.source=='cached'
                previous=command;entry['execution_action']='verified_'+decision.source
            elif not ever_accepted and clock_time<simulation.initialization_seconds-1e-10:
                command=requested_previous;initialization_waits+=1;entry['execution_action']='unverified_initialization_hold'
            else:
                outcome='initialization_failure' if not ever_accepted else 'execution_rejection'
                entry['execution_action']='terminate_recording_no_stop_maneuver';log('queries',entry);break
            log('queries',entry)
            end=min(simulation.duration,clock_time+simulation.control_interval,
                    clock_time+command.duration)
            interval_start=clock_time
            while clock_time<end-1e-10:
                # Split at sensor timestamps and at the delayed-command switch.
                delayed=(simulation.condition=='delay100ms' and clock_time<interval_start+.1-1e-10)
                actual=requested_previous if delayed else command
                step_end=min(end,clock_time+config.execution_dt,next_scan)
                if delayed:step_end=min(step_end,interval_start+.1)
                if step_end<=clock_time+1e-12:
                    record_scan();continue
                try:new=plant_step(state,actual,step_end-clock_time,simulation.condition)
                except (ValueError,OverflowError,ZeroDivisionError):
                    outcome='invalid_dynamics';break
                contact=segment_contacts_geometry(state.position,new.position,truth,ground)
                clearance=sampled_clearance(new.position,truth,ground)
                if clearance is not None:
                    minimum_clearance=clearance if minimum_clearance is None else min(minimum_clearance,clearance)
                    sampled_margin|=clearance<config.margin
                history.append(AppliedControl(clock_time,step_end,actual))
                log('controls',{'start':clock_time,'end':step_end,'control':asdict(actual),
                                'requested_control':asdict(command),'delayed':delayed})
                state=new;clock_time=step_end
                log('states',{'time':clock_time,'state':asdict(state),
                              'sampled_physical_clearance':clearance,'preceding_chord_contact':contact})
                if contact:outcome='physical_contact';physical_contacts+=1;break
                if not _envelope(state,config):outcome='envelope_violation';break
                if clock_time>=next_scan-1e-10:record_scan()
            requested_previous=command
            if outcome in ('physical_contact','envelope_violation','invalid_dynamics'):break
        # Goal arrival during the last integrated interval is still counted.
        if outcome=='time_limit' and math.hypot(state.x-goal[0],state.y-goal[1])<simulation.horizontal_goal_tolerance and abs(state.h-goal[2])<simulation.altitude_goal_tolerance:
            outcome='goal'
    summary={**provenance,'scene_id':scene['scene_id'],'outcome':outcome,
        'elapsed_simulation_seconds':clock_time,'initial_state':asdict(start_state),
        'final_state':asdict(state),'query_count':query_number,'scan_count':scan_number,
        'primary_integration_steps':total_primary,'successor_integration_steps':total_successor,
        'dispatch_integration_steps':total_dispatch,'fresh_sequences':fresh,'cached_sequences':cached,
        'initialization_waits':initialization_waits,'physical_chord_contacts':physical_contacts,
        'minimum_sampled_physical_clearance':minimum_clearance,
        'sampled_euclidean_margin_incursion':sampled_margin,
        'termination_policy':'report/stop recording; no physical emergency maneuver is inferred'}
    write_json(output/'summary.json',summary)
    return summary
