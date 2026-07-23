# Project History: InternNav

**Date Range:** 2025-07-25 to 2026-06-18 (approximately 11 months)
**Total Commits:** 164
**Total Code Churn:** 2,581 files changed, ~204,482 insertions, ~70,320 deletions
**Versions Released:** 0.1.0, 0.1.1, 0.2.0, 0.3.0, 0.3.1
**Active Branches:** main, dev, fix_dataset, wzcai_navdp_train, plus release branches
**Key Contributors (~25 total):** Yukai Wang, wzcai99, yugangli666, wangyukai, Yuqiang Yang, Gariscat, fengdelin, Meng Wei, Jiaqi Peng, ChaimZhu, Huang Wensi, DuangZhu, Tai Wang, Dong An, and others.

---

## 1. Initial Project Foundation (2025-07-25 to 2025-08-15)

The project began with a single large initial commit containing the full skeleton of the InternNav codebase, including agent implementations (CMA, RDP, Seq2Seq, InternVLA-N1), dataset loaders, evaluators, habitat environment extensions, model architectures, configuration system, and training scripts.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-07-25 | `70b8c65` | **Initial commit** -- full project skeleton with agents, configs, datasets, evaluators, models, and habitat extensions |
| 2025-07-26 | `27450da` | Update requirements for Isaac Sim and Habitat |
| 2025-07-30 | `9b2c65c` | Paths/Docs/Requirements fixes and demo improvement |
| 2025-07-30 | `f64ed02` | Fix Isaac requirements |
| 2025-08-01 | `1343239` | Change habitat evaluation data to InternData-N1 format |
| 2025-08-01 | `9d3bca3` | Improve gradio demo |
| 2025-08-01 | `01d73ed` | Update README with InternVLA-N1 benchmark results |
| 2025-08-05 | `a35e84f` | Add challenge guidelines and scripts |
| 2025-08-06 | `2f25d5d` | Fix requirements |
| 2025-08-08 | `e4e5b7d` | Update README with Docker and citations, repair code from previous PR |
| 2025-08-11 | `fe8693e` | Fix common issues in README, improve dataset downloading |
| 2025-08-14 | `dc0be61` / `9ef54eb` | Default config and path updates |
| 2025-08-15 | `f1dbffb` / `8ef766d` | NavDP config updates |

---

## 2. Challenge Infrastructure & Benchmark Development (2025-08-15 to 2025-09-04)

Major effort went into building challenge infrastructure, CI pipelines, testing, and benchmark support for the IROS competition.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-08-15 | `8ef766d` | NavDP config update |
| 2025-08-18 | `cd46374` | Update challenge guidelines, fix bugs |
| 2025-08-18 | `2714f67` | NavDP policy update |
| 2025-08-20 | `30f2f63` | NavDP policy update |
| 2025-08-20 | `c47f25a` | **Load .safetensors checkpoints** -- support for multiple model formats |
| 2025-08-21 | `3b11012` | Submission test |
| 2025-08-22 | `73ac382` | Fix dataset depth shape mismatch |
| 2025-08-22 | `2d49299` | Fix inference bugs |
| 2025-08-28 | `922a1a9` | Update challenge docs for image self-test |
| 2025-08-29 | `14584b1` | **Support Asynchronous Inference/Eval of InternVLA-N1** |
| 2025-09-01 | `4da9b75` / `e4c2b2d` / `a7838e5` | Async inference support for VLN-PE, docstrings, typo fixes |
| 2025-09-04 | `015efce` | **CI Pipeline and Code Style Improvements** -- GitHub Actions CI, issue templates, pre-commit config, unit and function tests |

---

## 3. IROS Onsite Competition & Real-World Deployment (2025-09-04 to 2025-10-13)

The project pivoted significantly toward real-world robot deployment for the IROS onsite competition, adding SDKs, real-world camera support, and server infrastructure.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-09-04 | `e25227a` | Update challenge README, clone release branch |
| 2025-09-05 | `43e0078` | Fix empty episodes |
| 2025-09-09 | `47d6596` | Delete unused `load_from_pretrained` flag |
| 2025-09-09 | `ecfa1fd` | Challenge doc update |
| 2025-09-09 | `0dd0050` | Visualization Arrow and distribution guard |
| 2025-09-16 | `46f8494` | IROS Challenge default config replaced agent |
| 2025-09-19 | `de2fe95` | Challenge config: env_num to 1 |
| 2025-09-28 | `b37a0ce` | **Add real-world deployment code of InternVLA-N1** -- server, KV cache, MPC/PID controllers, robot client |
| 2025-09-30 | `63598ed` through `a41806a` | **Real-world camera and robot setup** -- 7 commits adding camera support, robot readiness, real-world environment |
| 2025-09-30 | `254d5da` | **Major refactor** (115 files) -- structural reorganization |
| 2025-10-09 | `57ba669` | Update folder name |
| 2025-10-10 | `6a7d29c` | **IROS Onsite Competition Phase Update** -- full SDK (cam, control, real_world_env, save_obs, test_agent, test_robot), rules in EN/ZH, captures (832 insertions) |
| 2025-10-10 | `c4c6c4f` | Fix dataset path inconsistency and cuda:1 bug |
| 2025-10-13 | `674da39` | Update onsite challenge robot information |
| 2025-10-13 | `aaff0b9` | Update docs |

---

## 4. Major Architecture Refactoring (2025-09-17 to 2025-09-24)

A significant restructuring effort consolidated the codebase, fixing imports, adding benchmarks, and setting up the agent server architecture.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-09-17 | `2806f3b` | **Restructure prototype** (422 files touched) |
| 2025-09-17 | `cd0f863` | Fix minor naming typos |
| 2025-09-18 | `040b9f0` | Benchmark prototype (46 files) |
| 2025-09-18 | `88d8350` | Add InternUtopia benchmark (35 files, 3065 insertions) |
| 2025-09-18 | `58c1c29` | Rename dataloader to episode loader |
| 2025-09-18 | `ecc754d` / `c85013a` | Fix benchmark and general imports |
| 2025-09-18 | `cb2a515` | **Rewrite import paths in ./baselines** |
| 2025-09-18 | `93fa99e` | Setup pre-commit |
| 2025-09-18 | `b008050` / `b0cff68` | Add/configure LongCLIP submodule |
| 2025-09-18 | `43c0e78` / `16b171b` | General import and path fixes |
| 2025-09-19 | `bab95d1` | **Fixed all imports (first attempt)** -- 456 files changed, massive import path cleanup |
| 2025-09-19 | `3fa24f8` / `f0ca7e3` | Update setup and source imports |
| 2025-09-19 | `2811ff8` | Fix minor relative import issues |
| 2025-09-19 | `e130fcc` / `7d72294` | Import agent |
| 2025-09-19 | `7b52253` | Merge remote-tracking branch into refactor-baselines |
| 2025-09-19 | `56ec0ae` | **Move depth anything to src** (16 files) |
| 2025-09-22 | `173c254` | **Add agent server** |
| 2025-09-22 | `1a4bd91` | Fix RDP |
| 2025-09-22 | `9863fba` / `33b6551` | **Update start script using agent server, all baselines runnable** |
| 2025-09-22 | `a93e847` | Fix camera setting |
| 2025-09-23 | `33b6551` | Update evaluator type, habitat import |
| 2025-09-24 | `02070c5` | Fix pytest |
| 2025-09-24 | `6cccb26` | Logger fix |
| 2025-09-24 | `e151e7d` | Update episode loader and evaluator doc |
| 2025-09-26 | `dd5f43c` | Update base model |
| 2025-09-26 | `a7e139f` | Add docs |

---

## 5. Code Quality, Dependency Isolation & Further Refactoring (2025-10-13 to 2025-10-30)

After the initial restructuring, the team focused on isolating dependencies, refactoring the evaluator/env pipeline, and improving code organization.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-10-14 | `324a3bf` | Update README |
| 2025-10-14 | `6b4cdd0` | **Multi-GPU training and checkpoint save** -- major NavDP training overhaul |
| 2025-10-16 | `cc5bb81` | Add support for NavDP finetuning |
| 2025-10-17 | `6d7df72` | **Inference-only demo and real-world experiment code** -- KV cache, MPC/PID controllers, inference-only Jupyter notebook, real-world server |
| 2025-10-18 | `63f0c3c` | **Onsite SDK; Refactor agent and server** -- comm_utils, stream thread, test_server |
| 2025-10-20 | `6f94ba0` | Fix typo in mode name, add `use_async` config for async inference |
| 2025-10-22 | `865fd98` | **Refactor scripts folder** -- reorganize evaluation, demo, challenge, and iros scripts (41 files) |
| 2025-10-24 | `96517b1` | **Refactor evaluator, env, and extensions** -- major restructuring of evaluators, environment utilities, and habitat extensions (76 files) |
| 2025-10-27 | `d1010dc` | Initialize fix_dataset branch |
| 2025-10-28 | `efa856b` | Fix eval_habitat import |
| 2025-10-29 | `a0f3c82` | **Solve import issues, isolate dependency and requirements** -- separate requirements files, simple agent, dependency isolation (34 files) |
| 2025-10-29 | `ce6d62d` | Fix NavDP training gradient |
| 2025-10-29 | `3416684` | Delete interiornav (cleanup) |
| 2025-10-29 | `b99efb5` | Remove projects folder |
| 2025-10-30 | `5adf875` | Update README |

---

## 6. NavDP Training Refinement & Dataset Pipeline (2025-10-30 to 2025-11-07)

Significant work on the NavDP (Navigation Decision Policy) training pipeline, including finetuning support, gradient fixes, and dataset/loader renaming.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-10-30 | `81d8a0b` | Merge with main branch |
| 2025-10-31 | `b69af2c` | Minor renaming |
| 2025-11-03 | `c48cdc2` | Fix import bug in training script |
| 2025-11-03 | `6566261` | Support NavDP finetune |
| 2025-11-03 | `eb1d0e3` | Merge with main branch |
| 2025-11-03 | `b73c85e` | Update NavDP training parameters |
| 2025-11-03 | `c7af729` | Rename `EvalDatasetCfg` to `EpisodeCfg` |
| 2025-11-03 | `ea87f89` | Refactor `load_data` function, rename `dataloader` to `episode_iterator` |
| 2025-11-04 | `e0d265c` | Restore visualization feature, refactor RDP utilities into geometry utils |
| 2025-11-05 | `1545c20` | Support NavDP finetuning |
| 2025-11-05 | `ea95f2d` / `8dfe9b5` | Merge branches |
| 2025-11-05 | `fe1a004` | Rename data loader to episode loader (both file and class) |
| 2025-11-06 | `644b59d` | Modify class name for NavDP |
| 2025-11-07 | `8e9574e` | Update class name |
| 2025-11-07 | `1f9fc7d` | **Fix no_grad bug for NavDP training, support NavDP finetuning** |

---

## 7. Version 0.2.0: Habitat Refactor & 3D Printing (2025-11-27 to 2025-12-08)

| Date | Commit | Description |
|------|--------|-------------|
| 2025-11-27 | `f74a79b` | **Update 3D Printing Files for Camera of Unitree Go2** -- 3MF and STEP files for Go2 robot camera mount |
| 2025-12-02 | `431cfed` | **Habitat Refactor & Distributed VLNPE Refactor** -- massive restructure: distributed evaluator, episode loader, habitat extensions reorganization, new result logging system, torchrun support, CI test fixes (54 files, 2,917 insertions, 3,224 deletions) |
| 2025-12-08 | `5d4f6a5` | **Bump to v0.2.0** |
| 2025-12-08 | `b654fbb` / `20110f7` | Version bump and merge |

---

## 8. InternVLA-N1 Training, VLLN Bench & New Controllers (2025-12-15 to 2025-12-31)

A major expansion adding the full InternVLA-N1 training pipeline, support for the VL-LN benchmark, and new navigation controllers.

| Date | Commit | Description |
|------|--------|-------------|
| 2025-12-15 | `fb1d2a5` | **Add training code for InternVLA-N1** -- InternVLA-N1 dataset, policy, architecture, trainer (QwenVL base), NextDiT trajectory models, ROPE2D, dual system training scripts, zero2/zero3 configs (46 files, 4,635 insertions) |
| 2025-12-19 | `be95bca` | **Support "Flash without collision" Controller** -- new collision-aware flash controller for safer navigation |
| 2025-12-23 | `fd70eeb` | **Add training code for baseline of VLLN Bench** -- VLLN dataset, unified trainer, training script |
| 2025-12-26 | `c96c4e0` | **Support Evaluation in VL-LN Bench** -- dialog agent, object navigation, distributed evaluator, NPC system, habitat dialog/object environments, measures (36 files, 2,853 insertions) |
| 2025-12-27 | `7128121` / `84af356` | **Add MIT License** |
| 2025-12-30 | `49e86aa` | Fix issues noticed during Regression benchmark (16 files, cleanup of 808 deletions) |
| 2025-12-31 | `6a04b17` | Update inference-only demo for InternVLA-N1-DualVLN |

---

## 9. Version 0.3.0/0.3.1: Bug Fixes, Dataset Format Updates & Citations (2026-01-05 to 2026-03-10)

| Date | Commit | Description |
|------|--------|-------------|
| 2026-01-05 | `0303b1b` | Fix pre-commit issues (11 files) |
| 2026-01-05 | `5f3246c` | Fix VLLN path |
| 2026-01-05 | `4ed5bab` | **Bump to v0.3.0** |
| 2026-01-07 | `0f82252` | Add changelog, update task name to IIGN |
| 2026-02-05 | `832852a` | Update citation organization |
| 2026-02-05 | `b8d36a9` | Update submodule path |
| 2026-02-05 | `554fd97` | Support `vis_debug` option for Habitat evaluation |
| 2026-02-06 | `82467fb` | **Update dataset conversion for InternData-N1 VLN-PE v0.5 dataset format** (5 files, 392 insertions) |
| 2026-02-10 | `2802b96` | **Bump to v0.3.1** |
| 2026-02-10 | `1d8d078` / `5e287ed` | Version bump and pre-commit fix |
| 2026-03-03 | `748a30e` / `72bd906` / `f0fb98e` / `5544d95` | **Update NavDP training pipeline for latest InternData-N1 format** |
| 2026-03-09 | `c6f18c8` | Revise citations in README |
| 2026-03-10 | `7a5c624` | Revise citations in README (#310) |

---

## 10. G1 Humanoid Robot Deployment & Pixel-Goal Navigation (2026-06-05 to 2026-06-18)

The most recent work focuses on deploying the navigation system on the Unitree G1 humanoid robot with a D455 depth camera, adding pixel-goal navigation, experiment logging, and safety mechanisms.

| Date | Commit | Description |
|------|--------|-------------|
| 2026-06-05 | `9130be2` | **Add G1 real-world deployment client** -- full G1 client with HTTP-based navigation, controllers, thread utilities, workstation server script, deployment guide (10 files, 1,196 insertions) |
| 2026-06-06 | `25ebbfc` | Configure G1 client for D455 deployment |
| 2026-06-06 | `b206526` | Fix NavDP realworld server startup |
| 2026-06-06 | `e9311c4` | Add DualVLN workstation launch script |
| 2026-06-06 | `5dfbd29` | Detach DualVLN trajectories before numpy conversion |
| 2026-06-17 | `798d5d8` | **Add pixel goal visualization records** -- visual goal tracking in real-world deployment agent |
| 2026-06-17 | `b5ebf1d` | **Auto-generate experiment record pages** -- markdown page generation for each experiment, experiment_records/ directory |
| 2026-06-18 | `254668d` | **Add G1 navigation run logger** -- comprehensive logging system (5 files, 517 insertions) |
| 2026-06-18 | `ca12f47` | **Fix G1 DDS interface setup** -- DDS (Data Distribution Service) configuration for G1 robot communication |
| 2026-06-18 | `0dc98cb` | Mirror G1 run logs on workstation server |
| 2026-06-18 | `e7da274` | Fix workstation server g1_client import |
| 2026-06-18 | `28c9414` | Pull G1 logs after DualVLN server exit |
| 2026-06-18 | `384d134` | **Handle G1 stop action and safety stop** -- safety stop mechanism for the G1 robot |

---

## Architectural Milestones Summary

| Milestone | Date | Significance |
|-----------|------|-------------|
| Initial commit | 2025-07-25 | Full project skeleton with all core agents, datasets, models, evaluators |
| Async inference | 2025-08-29 | First major performance optimization for InternVLA-N1 evaluation |
| CI pipeline | 2025-09-04 | GitHub Actions, testing framework, code quality enforcement |
| IROS onsite SDK | 2025-10-10 | Complete real-world robot SDK for competition (cam, control, stream, env) |
| Real-world deployment | 2025-09-28 | First robot deployment code with server, KV cache, MPC/PID controllers |
| Import restructuring | 2025-09-19 | Massive 456-file import path cleanup |
| Agent server architecture | 2025-09-22 | Shift to agent-server architecture for baselines |
| Evaluator/env refactor | 2025-10-24 | 76-file restructuring of evaluation pipeline |
| Dependency isolation | 2025-10-29 | Separate requirements files, import isolation |
| Habitat + distributed refactor | 2025-12-02 | Major v0.2.0 milestone: distributed evaluation, episode loader, torchrun |
| InternVLA-N1 training | 2025-12-15 | Full training pipeline with QwenVL, NextDiT, ROPE2D, dual-system training |
| VL-LN Bench support | 2025-12-23/26 | Dialog agent, object navigation, VLLN training code |
| v0.3.0 release | 2026-01-05 | RegBench fixes, IIGN task naming |
| v0.3.1 release | 2026-02-10 | Dataset format update for InternData-N1 v0.5 |
| G1 robot deployment | 2026-06-05 | Complete G1 humanoid robot client with D455 camera, pixel-goal nav |
| Safety stop + logging | 2026-06-18 | Production safety features for real-world robot operation |

## Project Evolution Overview

The project started in July 2025 as a research framework for vision-language navigation (VLN) with support for multiple agents (CMA, RDP, Seq2Seq, InternVLA-N1) and training pipelines. Throughout August-October 2025, the focus shifted heavily toward real-world robot deployment for the IROS onsite competition, adding physical robot SDKs, camera support, and server infrastructure alongside a major codebase restructuring.

November-December 2025 saw a second major wave of development: the InternVLA-N1 training pipeline (the project's flagship model), distributed evaluation support, the VL-LN benchmark (dialog and object navigation), and collision-aware controllers. Two formal releases (v0.2.0 and v0.3.0) were cut during this period.

In early 2026, the project matured with dataset format updates, citation management, and training pipeline refinements.

The most recent work (June 2026) represents a new phase: deploying on the Unitree G1 humanoid robot with Intel D455 depth camera for pixel-goal navigation, with comprehensive experiment logging, DDS communication, and safety stop mechanisms -- indicating a shift toward production-grade real-world deployment on legged robots.
