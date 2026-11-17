"""Explicit NumPy reconstruction of the 58-64-32-2 training architecture.

No historical weights are included. Inference on initialized-but-untrained
weights is prohibited. Training results are new results, not manuscript results.
"""
import copy
import json
from pathlib import Path
import numpy as np

FORMAT = "FWCONTINUATION_RECONSTRUCTED_MLP_V1"
PARAMETER_NAMES = ("W1", "b1", "W2", "b2", "W3", "b3")


def _finite_array(value, name):
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError(name + " contains nonfinite values")
    return result


def _sigmoid(x):
    out = np.empty_like(x, dtype=np.float64)
    pos = x >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    e = np.exp(x[~pos])
    out[~pos] = e / (1.0 + e)
    return out


class ContinuationMLP:
    """Use for_training(...) to initialize, or load(...) for inference.

    predict_proba requires a 2-D batch of exactly input_size columns.
    Compact State models take 28 columns; all training models take 58.
    """
    def __init__(self):
        self.input_size = 58
        self.output_size = 2
        self.state_only = False
        self.auxiliary_weight = 0.0
        self.seed = None
        self.is_trained = False
        self.params = {}
        self.history = []
        self.best_epoch = None
        self.best_validation_bce = None
        self.provenance = {"implementation": "manuscript-based reconstruction",
                           "historical_weights": False}

    @classmethod
    def for_training(cls, seed=20261101, state_only=False, auxiliary_weight=0.1):
        if not np.isfinite(auxiliary_weight) or auxiliary_weight < 0:
            raise ValueError("Auxiliary weight must be finite and nonnegative")
        obj = cls()
        obj.seed = int(seed)
        obj.state_only = bool(state_only)
        obj.auxiliary_weight = float(auxiliary_weight)
        rng = np.random.default_rng(obj.seed)
        # Explicit reconstruction choice: He initialization, identical shape and
        # RNG sequence for paired state/observation and BCE/Aux variants.
        for k, (ni, no) in enumerate([(58, 64), (64, 32), (32, 2)], 1):
            obj.params[f"W{k}"] = rng.normal(0.0, np.sqrt(2.0/ni), (ni, no))
            obj.params[f"b{k}"] = np.zeros(no, dtype=np.float64)
        return obj

    def _batch(self, x):
        x = _finite_array(x, "Feature batch")
        if x.ndim != 2 or x.shape[1] != self.input_size:
            raise ValueError(f"Expected (N,{self.input_size}) features; legacy 103-input models are unsupported")
        if self.state_only and self.input_size == 58:
            x = x.copy()
            x[:, 28:] = 0.0
        return x

    def _forward(self, x):
        if not self.params:
            raise RuntimeError("Initialize explicitly for training or load trained weights")
        x = self._batch(x)
        z1 = x @ self.params["W1"] + self.params["b1"]
        h1 = np.maximum(z1, 0.0)
        z2 = h1 @ self.params["W2"] + self.params["b2"]
        h2 = np.maximum(z2, 0.0)
        outputs = h2 @ self.params["W3"] + self.params["b3"]
        if not np.isfinite(outputs).all():
            raise ValueError("Network evaluation produced nonfinite outputs")
        return outputs, (x, z1, h1, z2, h2)

    def predict_proba(self, batch):
        if not self.is_trained:
            raise RuntimeError("No trained/loaded weights: random initialization is not an inference model")
        output, _ = self._forward(batch)
        return _sigmoid(output[:, 0])

    def _targets(self, y, work, n):
        y = _finite_array(y, "Feasibility labels")
        work = _finite_array(work, "Work labels")
        if y.shape != (n,) or work.shape != (n,) or n == 0:
            raise ValueError("Nonempty labels and work must be one-dimensional and match the batch")
        if not np.isin(y, [0.0, 1.0]).all() or np.any(work < 0):
            raise ValueError("Labels must be binary and integration work nonnegative")
        return y, work

    def loss_and_gradients(self, batch, y, work):
        """Mean BCE + lambda*Huber1(q-log1p(work)); no weight decay here."""
        if self.output_size != 2:
            raise RuntimeError("Compact inference exports cannot be trained")
        pred, (x, z1, h1, z2, h2) = self._forward(batch)
        y, work = self._targets(y, work, len(pred))
        logits, q = pred[:, 0], pred[:, 1]
        residual = q - np.log1p(work)
        absolute = np.abs(residual)
        bce = np.mean(np.logaddexp(0.0, logits) - y * logits)
        huber = np.mean(np.where(absolute <= 1.0, 0.5*residual**2, absolute-0.5))
        grad_out = np.column_stack((_sigmoid(logits)-y,
                                    self.auxiliary_weight*np.clip(residual,-1.0,1.0))) / len(y)
        g = {"W3": h2.T @ grad_out, "b3": grad_out.sum(axis=0)}
        dz2 = (grad_out @ self.params["W3"].T) * (z2 > 0.0)
        g.update(W2=h1.T @ dz2, b2=dz2.sum(axis=0))
        dz1 = (dz2 @ self.params["W2"].T) * (z1 > 0.0)
        g.update(W1=x.T @ dz1, b1=dz1.sum(axis=0))
        return float(bce+self.auxiliary_weight*huber), g

    def validation_bce(self, batch, y):
        pred, _ = self._forward(batch)
        y, _ = self._targets(y, np.zeros(len(pred)), len(pred))
        return float(np.mean(np.logaddexp(0.0,pred[:,0])-y*pred[:,0]))

    def fit(self, x, y, work, validation_x, validation_y, *, epochs=60,
            batch_size=2048, learning_rate=1e-3, weight_decay=1e-5):
        """Train with Adam and restore the earliest minimum-validation-BCE epoch.

        The work target is required even for BCE-only training, to make the
        dataset contract explicit. Only validation BCE selects checkpoints.
        This does not select deployment seeds or the closed-loop schedule.
        """
        if not self.params or self.output_size != 2 or self.input_size != 58:
            raise RuntimeError("fit requires an explicitly initialized 58-input training model")
        if (not isinstance(epochs, int) or epochs < 1 or
            not isinstance(batch_size, int) or batch_size < 1 or
            not np.isfinite([learning_rate,weight_decay]).all() or
            learning_rate <= 0 or weight_decay < 0):
            raise ValueError("Invalid training hyperparameters")
        x = self._batch(x)
        vx = self._batch(validation_x)
        y, work = self._targets(y, work, len(x))
        vy, _ = self._targets(validation_y, np.zeros(len(vx)), len(vx))
        self.is_trained = False
        self.history = []
        rng = np.random.default_rng(self.seed)
        first = {k:np.zeros_like(v) for k,v in self.params.items()}
        second = {k:np.zeros_like(v) for k,v in self.params.items()}
        step = 0
        best = float("inf")
        best_weights = None
        for epoch in range(1,epochs+1):
            order = rng.permutation(len(x))
            total = 0.0
            for start in range(0,len(x),batch_size):
                ids = order[start:start+batch_size]
                loss, grad = self.loss_and_gradients(x[ids], y[ids], work[ids])
                total += loss*len(ids)
                step += 1
                for key in PARAMETER_NAMES:
                    # Coupled L2 decay matches the conventional Adam option,
                    # not decoupled AdamW. Biases are included explicitly.
                    g = grad[key]+weight_decay*self.params[key]
                    first[key] = 0.9*first[key]+0.1*g
                    second[key] = 0.999*second[key]+0.001*g*g
                    self.params[key] -= learning_rate*(first[key]/(1-0.9**step)) / (np.sqrt(second[key]/(1-0.999**step))+1e-8)
            val = self.validation_bce(vx, vy)
            self.history.append({"epoch":epoch,"training_loss":total/len(x),"validation_bce":val})
            if val < best:  # Exact ties keep the earlier epoch.
                best = val
                self.best_epoch = epoch
                best_weights = {k:v.copy() for k,v in self.params.items()}
        self.params = best_weights
        self.best_validation_bce = best
        self.is_trained = True
        self.provenance.update(training_examples=len(x), validation_examples=len(vx),
                               optimizer="Adam, coupled L2 decay", epochs=epochs,
                               batch_size=batch_size, learning_rate=learning_rate,
                               weight_decay=weight_decay,
                               claim="new training on caller-provided data; not recovered historical weights")
        return self

    def export_compact(self, path=None):
        """Return a trained single-output copy, dropping masked state columns."""
        if not self.is_trained:
            raise RuntimeError("Cannot export untrained random weights")
        compact = copy.deepcopy(self)
        compact.output_size = 1
        compact.input_size = 28 if self.state_only else 58
        compact.params["W1"] = self.params["W1"][:compact.input_size,:].copy()
        compact.params["W3"] = self.params["W3"][:,:1].copy()
        compact.params["b3"] = self.params["b3"][:1].copy()
        if path is not None:
            compact.save(path)
        return compact

    def _metadata(self):
        return {"format":FORMAT,"architecture":[self.input_size,64,32,self.output_size],
                "state_only":self.state_only,"trained":self.is_trained,"seed":self.seed,
                "auxiliary_weight":self.auxiliary_weight,"best_epoch":self.best_epoch,
                "best_validation_bce":self.best_validation_bce,"history":self.history,
                "provenance":self.provenance}

    def save(self, path):
        if not self.is_trained:
            raise RuntimeError("Refusing to save untrained weights as an inference artifact")
        path = Path(path)
        metadata = self._metadata()
        for key,v in self.params.items():
            _finite_array(v,key)
        if path.suffix.lower() == ".json":
            path.write_text(json.dumps({"metadata":metadata,"parameters":{k:v.tolist() for k,v in self.params.items()}},indent=2,allow_nan=False),encoding="utf-8")
        elif path.suffix.lower() == ".npz":
            np.savez_compressed(path,metadata=np.array(json.dumps(metadata,allow_nan=False)),**self.params)
        else:
            raise ValueError("Model path must end in .json or .npz")

    @classmethod
    def load(cls, path):
        """Load only this explicit schema; never reinterpret 103-input weights."""
        path = Path(path)
        if path.suffix.lower() == ".json":
            data = json.loads(path.read_text(encoding="utf-8"))
            metadata, params = data["metadata"], data["parameters"]
        elif path.suffix.lower() == ".npz":
            with np.load(path,allow_pickle=False) as data:
                if set(data.files) != set(PARAMETER_NAMES)|{"metadata"}:
                    raise ValueError("Unexpected model archive entries")
                metadata = json.loads(str(data["metadata"].item()))
                params = {k:data[k].copy() for k in PARAMETER_NAMES}
        else:
            raise ValueError("Model path must end in .json or .npz")
        # Reject nonfinite metadata as well as nonfinite tensors; JSON's default
        # decoder otherwise accepts non-standard NaN/Infinity tokens.
        json.dumps(metadata,allow_nan=False)
        arch = metadata.get("architecture")
        if (metadata.get("format") != FORMAT or metadata.get("trained") is not True or
            arch not in ([58,64,32,2],[58,64,32,1],[28,64,32,1])):
            raise ValueError("Unsupported/untrained model; legacy 103-input weights cannot be loaded")
        if type(metadata.get("state_only")) is not bool:
            raise ValueError("Model must declare state_only as a boolean")
        if arch[0] == 28 and not metadata["state_only"]:
            raise ValueError("28-input compact models must declare state_only")
        if set(params) != set(PARAMETER_NAMES):
            raise ValueError("Missing or unexpected model parameter")
        obj = cls()
        obj.input_size,obj.output_size=arch[0],arch[-1]
        obj.state_only=metadata["state_only"]
        obj.auxiliary_weight=float(metadata.get("auxiliary_weight",0.0))
        if not np.isfinite(obj.auxiliary_weight) or obj.auxiliary_weight < 0:
            raise ValueError("Invalid auxiliary weight")
        shapes={"W1":(arch[0],64),"b1":(64,),"W2":(64,32),"b2":(32,),
                "W3":(32,arch[-1]),"b3":(arch[-1],)}
        for k in PARAMETER_NAMES:
            v=_finite_array(params[k],k)
            if v.shape != shapes[k]:
                raise ValueError(f"Invalid shape for {k}: expected {shapes[k]}")
            obj.params[k]=v
        obj.is_trained=True
        obj.seed=metadata.get("seed")
        obj.best_epoch=metadata.get("best_epoch")
        obj.best_validation_bce=metadata.get("best_validation_bce")
        obj.history=metadata.get("history",[])
        obj.provenance=metadata.get("provenance",{})
        return obj
