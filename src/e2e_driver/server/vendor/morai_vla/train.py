"""Teacher training entry: the recipe's alpamayo1_5_sft.train_hf (same Trainer with lr multipliers, same Hydra layout),
plus `resume` (checkpoint dir, or `true` for the latest in output_dir), which train_hf never passes to trainer.train().
Usage (cwd train/): python -m morai_vla.train --config-name step2 paths.alpamayo15=... data.root=... [resume=true]
"""
import json, os

import hydra, hydra.utils as hyu, torch
from alpamayo.common import config_utils, misc
from alpamayo1_5_sft.performance.apply_optimizations import apply_runtime_optimizations
from alpamayo1_5_sft.performance.perf_utils import perf_plain
from alpamayo1_5_sft.trainer import ReasoningVLA_Trainer, TrainingArguments
from omegaconf import OmegaConf

from morai_vla.early_stop import TrajEarlyStop


@hydra.main(version_base=None, config_path="../configs", config_name="step2")
def main(cfg):
    apply_runtime_optimizations({"optimizations": perf_plain(cfg)})
    misc.seed_everything(42)
    args = TrainingArguments(**OmegaConf.to_container(cfg.trainer, resolve=True))
    model = hyu.instantiate(cfg.model, _convert_="partial")
    train_ds = hyu.instantiate(cfg.data.train_dataset, _convert_="partial", model_config=model.config)
    val_ds = hyu.instantiate(cfg.data.val_dataset, _convert_="partial", model_config=model.config)
    collate = hyu.instantiate(cfg.data.collate_fn, _convert_="partial", model_config=model.config)
    trainer = ReasoningVLA_Trainer(model=model, args=args, train_dataset=train_ds, eval_dataset=val_ds, data_collator=collate,
                                   callbacks=[hyu.instantiate(c, _convert_="partial") for c in cfg.callbacks.values()])
    if cfg.trainer.get("deepspeed"):
        trainer.accelerator.state.deepspeed_plugin.hf_ds_config._dtype = torch.float32   # as train_hf: fp32 trajectory inputs
    if trainer.is_world_process_zero():
        os.makedirs(cfg.paths.output_dir, exist_ok=True)
        config_utils.save_config(cfg, os.path.join(cfg.paths.output_dir, "config.yaml"), resolve_paths=True, include_hydra_config=True)
    trainer.train(resume_from_checkpoint=cfg.get("resume", None))
    es = next((c for c in trainer.callback_handler.callbacks if isinstance(c, TrajEarlyStop)), None)
    if es: es.finalize(trainer.model, cfg.paths.output_dir)                       # final/ = the lowest-ADE measurement
    if args.load_best_model_at_end and trainer.is_world_process_zero():              # which checkpoint final/ is (pipeline metrics)
        st = trainer.state
        with open(os.path.join(cfg.paths.output_dir, "best.json"), "w") as f:
            json.dump({"best_model_checkpoint": st.best_model_checkpoint, "best_global_step": getattr(st, "best_global_step", None),
                       args.metric_for_best_model: st.best_metric, "last_step": st.global_step}, f, indent=1)
    if cfg.get("save_final", True):                                                   # off for smoke/profiling runs (~23 GB each)
        trainer.save_model(os.path.join(cfg.paths.output_dir, "final"))
    if torch.distributed.is_initialized():
        torch.distributed.destroy_process_group()


if __name__ == "__main__":
    main()
