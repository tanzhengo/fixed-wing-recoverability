"""Numerical/unit fixtures only; none are flight data or experiment results."""
from dataclasses import replace
import json
import math
import numpy as np
import unittest
import tempfile
import contextlib
import io
from pathlib import Path
from fwcontinuation.features import state_features,observation_features,encode_features
from fwcontinuation.learning import ContinuationMLP
from fwcontinuation.types import State,Control,Rollout,Candidate,Config

raises = unittest.TestCase().assertRaises


def candidate(terminal,cost=4.0):
    return Candidate(3,Control(0.,0.,1.),Rollout([terminal],[0.],True),cost)


def test_scalar_slots_and_units():
    initial=State(x=2,y=3,h=20,V=20,gamma=.035,phi=.061,alpha=.044,T=2,yaw_rate=.12)
    terminal=State(x=12,y=23,h=23,V=18,gamma=.07,psi=math.pi/2,phi=.122,alpha=.088,T=4,yaw_rate=.34)
    cfg=Config(min_altitude=10,max_altitude=50,H=3,B=3,vertical_clearance=2)
    f=state_features(initial,candidate(terminal),[112,223,33],cfg)
    expected=[.9,.2,.2,.2,.2,.34,1.,.1,.1,.1,.1,.12,.1,.2,.1,1.,0.,.8,-.4,.2,.26,.54,.3,.3,0.,0.,.2,.2]
    np.testing.assert_allclose(f,expected,atol=1e-14)


def test_probe_pairs_order_and_fractions():
    class RecordingMap:
        def __init__(self):self.calls=[]
        def state(self,p):
            i=len(self.calls);self.calls.append(p)
            return ["free","free","unknown","occupied","unknown"][i%5]
    m=RecordingMap();t=State(x=2,y=3,h=20,V=10,psi=0.)
    f=observation_features(t,m,Config(B=3))
    np.testing.assert_allclose(f,np.tile([.4,.4],15))
    assert len(m.calls)==75
    for group,(beta,eta) in enumerate((b,e) for b in [-45,-22.5,0,22.5,45] for e in [-20,0,20]):
        direction=np.array([math.cos(math.radians(eta))*math.cos(math.radians(beta)),
                            math.cos(math.radians(eta))*math.sin(math.radians(beta)),math.sin(math.radians(eta))])
        np.testing.assert_allclose(m.calls[group*5],np.array(t.position)+6*direction)
        np.testing.assert_allclose(m.calls[group*5+4],np.array(t.position)+30*direction)


def test_features_invariant_to_rigid_frame_change():
    initial=State(x=5,y=-4,h=18,psi=.4,yaw_rate=.2)
    terminal=State(x=19,y=7,h=22,psi=.8,yaw_rate=.3)
    goal=np.array([80.,12.,25.]);cfg=Config(min_altitude=5,max_altitude=45)
    class Map:
        def state(self,p):
            return "free" if p[0]>15 else ("unknown" if p[1]>0 else "occupied")
    angle=.7;c,s=np.cos(angle),np.sin(angle);rot=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    shift=np.array([100.,-30.,8.])
    def transform_state(v):
        p=rot@np.array(v.position)+shift
        return replace(v,x=p[0],y=p[1],h=p[2],psi=v.psi+angle)
    class TransformedMap:
        def state(self,p):return Map().state(rot.T@(np.array(p)-shift))
    a=encode_features(initial,candidate(terminal),goal,Map(),cfg)
    b=encode_features(transform_state(initial),candidate(transform_state(terminal)),rot@goal+shift,
                      TransformedMap(),replace(cfg,min_altitude=13,max_altitude=53))
    np.testing.assert_allclose(a,b,atol=2e-14)


def test_invalid_map_labels_and_nonfinite_features():
    class InvalidMap:
        def state(self,p):return "maybe"
    with raises(ValueError):observation_features(State(),InvalidMap(),Config())
    with raises(ValueError):state_features(State(V=float('nan')),candidate(State()),[0,0,0],Config())


def test_bce_huber_backprop_matches_finite_differences(state_only):
    rng=np.random.default_rng(42)
    x=rng.normal(0,.1,(4,58));y=np.array([0.,1.,1.,0.]);work=np.array([0.,1.,3.,6.])
    model=ContinuationMLP.for_training(seed=9,state_only=state_only,auxiliary_weight=.1)
    _,grads=model.loss_and_gradients(x,y,work)
    eps=1e-6
    for name,index in [("W1",(0,1)),("W1",(40,5)),("b1",(3,)),("W2",(1,2)),("b2",(2,)),("W3",(2,0)),("W3",(4,1)),("b3",(0,)),("b3",(1,))]:
        original=model.params[name][index]
        model.params[name][index]=original+eps
        plus=model.loss_and_gradients(x,y,work)[0]
        model.params[name][index]=original-eps
        minus=model.loss_and_gradients(x,y,work)[0]
        model.params[name][index]=original
        np.testing.assert_allclose(grads[name][index],(plus-minus)/(2*eps),atol=3e-8,rtol=2e-4)


def test_trained_compact_export_and_round_trip_parity(tmp_path,state_only):
    rng=np.random.default_rng(18);x=rng.normal(0,.2,(12,58))
    y=(x[:,0]>0).astype(float);work=np.arange(12)
    model=ContinuationMLP.for_training(seed=5,state_only=state_only,auxiliary_weight=.1)
    with raises(RuntimeError):model.predict_proba(x)
    with raises(RuntimeError):model.export_compact()
    model.fit(x,y,work,x.copy(),y.copy(),epochs=3,batch_size=5)
    expected=model.predict_proba(x)
    compact=model.export_compact()
    assert compact.input_size==(28 if state_only else 58)
    np.testing.assert_allclose(compact.predict_proba(x[:,:compact.input_size]),expected,atol=1e-14)
    assert model.best_validation_bce==min(v['validation_bce'] for v in model.history)
    for suffix in ['.json','.npz']:
        path=tmp_path/('weights'+suffix);compact.save(path)
        restored=ContinuationMLP.load(path)
        np.testing.assert_array_equal(restored.predict_proba(x[:,:restored.input_size]),compact.predict_proba(x[:,:compact.input_size]))
    with raises(ValueError):compact.predict_proba(np.zeros((1,103)))
    with raises(ValueError):compact.predict_proba(np.full((1,compact.input_size),np.nan))


def test_checkpoint_exact_tie_keeps_first_epoch():
    m=ContinuationMLP.for_training(seed=3)
    m.validation_bce=lambda x,y:.5
    x=np.zeros((2,58));m.fit(x,np.array([0.,1.]),np.zeros(2),x,np.array([0.,1.]),epochs=3)
    assert m.best_epoch==1


def test_load_rejects_legacy_and_nonfinite_weights(tmp_path):
    m=ContinuationMLP.for_training(seed=4)
    x=np.zeros((2,58));m.fit(x,np.array([0.,1.]),np.zeros(2),x,np.array([0.,1.]),epochs=1)
    p=tmp_path/'m.json';m.export_compact(p)
    data=json.loads(p.read_text());data['metadata']['architecture'][0]=103
    p.write_text(json.dumps(data))
    with raises(ValueError):ContinuationMLP.load(p)
    m.export_compact(p);data=json.loads(p.read_text());data['parameters']['W1'][0][0]=float('nan')
    p.write_text(json.dumps(data))
    with raises(ValueError):ContinuationMLP.load(p)


def test_training_cli_dataset_contract_and_scene_leakage(tmp_path):
    from scripts.train_model import load_dataset,main
    a=tmp_path/'train.npz';b=tmp_path/'validation.npz'
    arrays={'X':np.zeros((2,58)),'y':np.array([0,1]),'work':np.array([2,3]),
            'scene_id':np.array(['shared','shared']),'query_id':np.array([1,2])}
    np.savez(a,**arrays);np.savez(b,**arrays)
    x,y,work,scene=load_dataset(a)
    assert x.shape==(2,58) and scene.tolist()==['shared','shared']
    with contextlib.redirect_stderr(io.StringIO()),raises(SystemExit):
        main(['--train',str(a),'--validation',str(b),'--output-dir',str(tmp_path/'runs'),
              '--dataset-description','Numerical unit fixture, not flight data'])
    assert not (tmp_path/'runs').exists()
    arrays['X']=np.zeros((2,103));np.savez(a,**arrays)
    with raises(ValueError):load_dataset(a)


def load_tests(loader, tests, pattern):
    suite=unittest.TestSuite()
    for func in [test_scalar_slots_and_units,test_probe_pairs_order_and_fractions,
                 test_features_invariant_to_rigid_frame_change,
                 test_invalid_map_labels_and_nonfinite_features,
                 test_checkpoint_exact_tie_keeps_first_epoch]:
        suite.addTest(unittest.FunctionTestCase(func))
    for state_only in [False,True]:
        suite.addTest(unittest.FunctionTestCase(
            lambda state_only=state_only:test_bce_huber_backprop_matches_finite_differences(state_only),
            description=f"finite-difference gradient check state_only={state_only}"))
        def compact_case(state_only=state_only):
            with tempfile.TemporaryDirectory() as directory:
                test_trained_compact_export_and_round_trip_parity(Path(directory),state_only)
        suite.addTest(unittest.FunctionTestCase(compact_case,
                         description=f"compact export parity state_only={state_only}"))
    def invalid_case():
        with tempfile.TemporaryDirectory() as directory:
            test_load_rejects_legacy_and_nonfinite_weights(Path(directory))
    suite.addTest(unittest.FunctionTestCase(invalid_case))
    def dataset_case():
        with tempfile.TemporaryDirectory() as directory:
            test_training_cli_dataset_contract_and_scene_leakage(Path(directory))
    suite.addTest(unittest.FunctionTestCase(dataset_case))
    return suite


if __name__=="__main__":
    unittest.main()
