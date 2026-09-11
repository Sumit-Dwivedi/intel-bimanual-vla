"""bimanual: Bimanual VLA manipulation on Intel Core Ultra.

Package layout (authoritative in PLAN.md section 3 and ARCHITECTURE.md section 1;
the two must agree on names):

    bimanual.command      CommandSource, CommandEvent, TextCommandSource, VoiceCommandSource
    bimanual.language     Grounder, RuleGrounder, SkillCall, TaskPlan
    bimanual.control      Coordinator, IKSolver, scripted skill primitives, SkillExecutor
    bimanual.policy       PolicyBackend, TorchPolicyBackend, OpenVinoPolicyBackend
    bimanual.perception   PerceptionBackend, StatePerception, VisionPerception (PoseNet)
    bimanual.sim          TableSettingEnv, Randomizer, SceneConfig, assets/*.xml
    bimanual.eval         EvalHarness, EpisodeRecorder, metrics

This file is intentionally near-empty at M01 (repo scaffold). Each module above is
filled in by its own PLAN.md work package (M02 onward); importing `bimanual` here
must not import any submodule that depends on MuJoCo, torch, or OpenVINO being
importable, because M01's done-when #2 (`python -c "import bimanual"`) must pass on
every host, including the laptop where MuJoCo cannot load (see ADR-020).
"""

__version__ = "0.1.0"
