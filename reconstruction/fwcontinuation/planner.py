"""Finite-horizon search reconstructed from the supplied manuscript.

K is a fixed scheduled-set cap, not a wall-clock deadline. A pruned position
is never refilled. Inference scores only order explicit validation queries.
"""
import math
import numpy as np
from dataclasses import replace
from .types import Candidate, SearchResult, Witness
from .dynamics import generate_primary, generate_successors, rollout
from .features import encode_features, observation_features

MODES = ('cost', 'diverse', 'learned', 'combined', 'observed', 'full')

def task_cost(initial, terminal, command, goal, previous=None):
    dx,dy,dz = (goal[0]-terminal.x,goal[1]-terminal.y,goal[2]-terminal.h)
    cost = .02*math.sqrt(dx*dx+dy*dy+dz*dz)+.15*abs(dz)
    if math.hypot(dx,dy) >= 1:
        angle = math.atan2(dy,dx)-terminal.psi
        cost += 3*abs(math.atan2(math.sin(angle),math.cos(angle)))
    if previous is not None:
        cost += abs(command.phi-previous.phi)+.1*abs(command.alpha-previous.alpha)+.05*abs(command.T-previous.T)
    return cost

def feasible_candidates(initial, goal, observed_map, config, previous=None):
    candidates=[]; total_steps=0
    controls=generate_primary(initial,goal,config)
    for index,command in enumerate(controls):
        path=rollout(initial,command,config,observed_map)
        total_steps+=path.steps
        if path.valid:
            cost=task_cost(initial,path.states[-1],command,goal,previous)
            if math.isfinite(cost): candidates.append(Candidate(index,command,path,cost))
    if not candidates and previous is not None:
        command=replace(previous,duration=config.H)
        path=rollout(initial,command,config,observed_map)
        total_steps+=path.steps
        if path.valid:
            cost=task_cost(initial,path.states[-1],command,goal,previous)
            if math.isfinite(cost): candidates.append(Candidate(len(controls),command,path,cost))
    candidates.sort(key=lambda c:(c.cost,c.index))
    for rank,candidate in enumerate(candidates): candidate.rank=rank
    return candidates,total_steps

def maneuver_distance(a,b,initial,config):
    speed=max(config.min_speed,initial.V)
    sy=min(math.pi,9.81*math.tan(config.max_roll)*config.H/speed)
    sh=speed*config.H*math.tan(config.max_gamma)
    angle=a.terminal.psi-b.terminal.psi
    heading=math.atan2(math.sin(angle),math.cos(angle))
    return math.hypot(heading/sy if sy>0 else 0,
                      (a.terminal.h-b.terminal.h)/sh if sh>0 else 0)

def build_schedule(candidates,initial,config,mode,scores=None):
    if mode not in MODES: raise ValueError('Unknown schedule '+mode)
    cap=len(candidates) if mode=='full' else min(config.K,len(candidates))
    if not cap:return []
    if mode in ('cost','full'):return candidates[:cap]
    if mode in ('learned','combined','observed'):
        if scores is None or any(c.index not in scores for c in candidates):
            raise ValueError('All candidate priorities are required')
        if any(not math.isfinite(float(x)) or not 0<=x<=1 for x in scores.values()):
            raise ValueError('Priorities must be finite probabilities/fractions')
    selected=[candidates[0]]; remaining=candidates[1:]
    closest={c.index:float('inf') for c in remaining}
    while len(selected)<cap:
        for c in remaining:
            closest[c.index]=min(closest[c.index],maneuver_distance(c,selected[-1],initial,config))
        best=None; best_value=-float('inf')
        for c in remaining:
            d=closest[c.index]
            if mode=='diverse': value=d
            elif mode=='learned': value=scores[c.index]
            else:value=config.coverage_weight*scores[c.index]+(1-config.coverage_weight)*d/(1+d)
            # Pure learned order depends on relative scores even when all are
            # tiny. The manuscript's 1e-7 tolerance belongs to the combined
            # priority, not to a rescaling-sensitive probability threshold.
            tolerance=0.0 if mode=='learned' else config.tie_tolerance
            if best is None or value>best_value+tolerance:
                best=c;best_value=value
        selected.append(best)
        remaining=[c for c in remaining if c.index!=best.index]
    return selected

def continuation_query(candidate,observed_map,config):
    records=[]; work=0
    for index,command in enumerate(generate_successors(candidate.terminal,config)):
        path=rollout(candidate.terminal,command,config,observed_map,start_time=config.H)
        work+=path.steps
        records.append({'successor_id':index,'steps':path.steps,'valid':path.valid,'reason':path.reason})
        if path.valid:
            return Witness(candidate,command,path,index),work,records
    return None,work,records

def _priorities(initial,candidates,goal,observed_map,config,mode,model):
    if mode=='observed':
        return {c.index:float(max(observation_features(c.terminal,observed_map,config)[::2])) for c in candidates}
    if mode not in ('learned','combined'):return None
    features=np.asarray([encode_features(initial,c,goal,observed_map,config) for c in candidates])
    if model.input_size not in (28,58):raise ValueError('Only 28/58-input reconstructed models are accepted')
    values=np.asarray(model.predict_proba(features[:,:model.input_size])).reshape(-1)
    if len(values)!=len(candidates):raise ValueError('Model returned the wrong number of scores')
    return {c.index:float(v) for c,v in zip(candidates,values)}

def search_candidates(initial,candidates,goal,observed_map,config,mode='diverse',
                      model=None,prune=True,deferred=True,query_fn=None):
    """Search a prevalidated pool; query_fn is an explicit test/oracle hook."""
    if mode not in MODES:raise ValueError('Unknown schedule '+mode)
    if mode in ('learned','combined') and model is None:
        raise ValueError('A trained checkpoint is required; no random/legacy fallback')
    if len({c.index for c in candidates})!=len(candidates):raise ValueError('Duplicate stable candidate IDs')
    candidates=sorted(candidates,key=lambda c:(c.cost,c.index))
    for rank,c in enumerate(candidates):c.rank=rank
    result=SearchResult('no_feasible_primary',candidates=candidates,map_version=observed_map.version)
    if not candidates:return result
    if config.K==0 and mode!='full':result.status='budget_exhausted';return result
    scores=None
    def score():
        nonlocal scores
        scores=_priorities(initial,candidates,goal,observed_map,config,mode,model)
        if mode in ('learned','combined'):result.inference_calls+=1
        result.recorded_scores=scores or {}
    if not deferred:score()
    query=query_fn or (lambda candidate:continuation_query(candidate,observed_map,config))
    def perform(c,position):
        witness,work,successors=query(c)
        if not isinstance(work,int) or work<0:raise ValueError('Invalid integration work')
        result.steps+=work
        result.queries.append({'position':position,'candidate_id':c.index,'cost_rank':c.rank,
            'cost':c.cost,'action':'queried','steps':work,
            'label':1 if witness else 0,'successors':successors})
        if witness is not None and (result.witness is None or c.key<result.witness.candidate.key):result.witness=witness
    seed=candidates[0];result.schedule=[seed.index];perform(seed,0)
    if result.witness is not None:
        result.status='verified';return result
    if min(config.K,len(candidates))<=1 and mode!='full':
        result.status='finite_library_exhausted' if len(candidates)==1 else 'budget_exhausted'
        return result
    if deferred:score()
    schedule=build_schedule(candidates,initial,config,mode,scores)
    result.schedule=[c.index for c in schedule]
    for position,c in enumerate(schedule[1:],start=1):
        if prune and result.witness is not None and c.key>=result.witness.candidate.key:
            result.queries.append({'position':position,'candidate_id':c.index,'cost_rank':c.rank,
                'cost':c.cost,'action':'pruned','steps':0,'label':None,'successors':[]})
            continue
        perform(c,position)
        if result.witness is not None and mode in ('cost','full'):break
    result.status=('verified' if result.witness is not None else
                   'finite_library_exhausted' if len(schedule)==len(candidates) else 'budget_exhausted')
    return result

def plan(initial,goal,observed_map,config,mode='diverse',model=None,previous=None,
         prune=True,deferred=True):
    """A proposal never executes a primary-only fallback. Dispatch is separate."""
    version=observed_map.version
    candidates,primary_steps=feasible_candidates(initial,goal,observed_map,config,previous)
    result=search_candidates(initial,candidates,goal,observed_map,config,mode,model,prune,deferred)
    result.primary_steps=primary_steps
    if observed_map.version!=version:
        raise RuntimeError('Map changed during proposal construction; use an immutable snapshot')
    return result
