"""Stops a run at the first non-finite logged loss or grad norm instead of training on for hours; writes
<output_dir>/NONFINITE, which scripts/pipeline_gate.py turns into a pipeline halt."""
import math
from pathlib import Path

from transformers.trainer_callback import TrainerCallback


class NonFiniteStop(TrainerCallback):
    def on_log(self, args, state, control, logs=None, **kw):
        bad = [k for k, v in (logs or {}).items() if isinstance(v, float) and not math.isfinite(v)]
        if bad:
            control.should_training_stop = True
            if state.is_world_process_zero:
                (Path(args.output_dir) / "NONFINITE").write_text(f"non-finite {bad} at step {state.global_step}\n")
