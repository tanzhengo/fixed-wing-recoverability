import unittest
import math
from dataclasses import replace
from unittest.mock import patch
from fwcontinuation.types import State,Control,Config,Rollout,Candidate,Witness
from fwcontinuation.planner import search_candidates,build_schedule

class Map:
    version=1
    def state(self,p):return 'free'

class Model:
    input_size=58
    def __init__(self):self.calls=0
    def predict_proba(self,x):
        self.calls+=1
        return [min(.99,float(row[26])/10) for row in x]

def pool():
    control=Control(0,0,1)
    return [Candidate(i,control,Rollout([State(psi=i*.1,h=20+i%3)],[0],True),float(i)) for i in range(9)]

def oracle(positive):
    def query(c):
        path=Rollout([c.terminal],[0],True)
        witness=Witness(c,c.control,path,0) if c.index in positive else None
        return witness,(c.index+1)*7,[{'valid':witness is not None}]
    return query

class SearchTests(unittest.TestCase):
    def test_noninteger_budgets_and_library_counts_rejected(self):
        for fields in [dict(K=1.5),dict(headings=2.5),dict(vertical_levels=3.5),dict(K=True)]:
            with self.assertRaises(ValueError):Config(**fields)
    def test_seed_success_skips_network(self):
        model=Model();r=search_candidates(State(),pool(),(250,0,20),Map(),Config(),mode='combined',model=model,query_fn=oracle({0}))
        self.assertEqual(model.calls,0);self.assertEqual(r.steps,7);self.assertEqual(r.witness.candidate.index,0)
    def test_pruning_preserves_selection_without_refill(self):
        for mode in ['diverse','combined','learned','observed','cost','full']:
            for positive in [{1,2,3,6},{8},{0},set()]:
                args=(State(),pool(),(250,0,20),Map(),Config(K=7))
                a=search_candidates(*args,mode=mode,model=Model(),prune=False,query_fn=oracle(positive))
                b=search_candidates(*args,mode=mode,model=Model(),prune=True,query_fn=oracle(positive))
                self.assertEqual(a.schedule,b.schedule)
                self.assertEqual(a.witness.candidate.index if a.witness else None,b.witness.candidate.index if b.witness else None)
                self.assertLessEqual(b.steps,a.steps)
                for row in b.queries:
                    if row['action']=='pruned':self.assertIsNone(row['label'])
    def test_deferred_equal_eager(self):
        for mode in ['combined','learned']:
            for positive in [{0},{2,5},set()]:
                args=(State(),pool(),(250,0,20),Map(),Config(K=8))
                a=search_candidates(*args,mode=mode,model=Model(),deferred=False,query_fn=oracle(positive))
                b=search_candidates(*args,mode=mode,model=Model(),deferred=True,query_fn=oracle(positive))
                self.assertEqual(a.steps,b.steps);self.assertEqual(a.queries,b.queries)
    def test_no_witness_never_primary_fallback(self):
        r=search_candidates(State(),pool(),(250,0,20),Map(),Config(K=2),query_fn=oracle(set()))
        self.assertEqual(r.status,'budget_exhausted');self.assertIsNone(r.witness)
        r=search_candidates(State(),pool(),(250,0,20),Map(),Config(K=20),query_fn=oracle(set()))
        self.assertEqual(r.status,'finite_library_exhausted')
    def test_zero_budget_and_missing_weights(self):
        r=search_candidates(State(),pool(),(250,0,20),Map(),Config(K=0),query_fn=oracle({0}))
        self.assertEqual(r.steps,0);self.assertFalse(r.queries)
        with self.assertRaises(ValueError):search_candidates(State(),pool(),(250,0,20),Map(),Config(),mode='learned')
    def test_equal_cost_stable_id(self):
        candidates=pool()
        for c in candidates:c.cost=1.0
        r=search_candidates(State(),candidates[::-1],(250,0,20),Map(),Config(K=9),query_fn=oracle({2,3,7}))
        self.assertEqual(r.witness.candidate.index,2)
    def test_no_pruned_refill(self):
        candidates=pool(); config=Config(K=4)
        r=search_candidates(State(),candidates,(250,0,20),Map(),config,query_fn=oracle({1,2,3,4,5,6,7,8}))
        self.assertEqual(len(r.schedule),4)
        self.assertTrue({row['candidate_id'] for row in r.queries}.issubset(set(r.schedule)))
    def test_zero_scores_combined_equals_diverse(self):
        candidates=pool();config=Config(K=9)
        a=build_schedule(candidates,State(),config,'diverse')
        b=build_schedule(candidates,State(),config,'combined',{c.index:0 for c in candidates})
        self.assertEqual([c.index for c in a],[c.index for c in b])
    def test_pure_learned_order_invariant_to_small_positive_scaling(self):
        candidates=pool();config=Config(K=9)
        scores={c.index:(c.index+1)/10 for c in candidates}
        a=build_schedule(candidates,State(),config,'learned',scores)
        b=build_schedule(candidates,State(),config,'learned',{i:s*1e-10 for i,s in scores.items()})
        self.assertEqual([c.index for c in a],[c.index for c in b])

if __name__=='__main__':unittest.main()
