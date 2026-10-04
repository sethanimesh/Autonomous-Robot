# Models, inference backends, and performance limits

These guides retain the original source investigation and historical results, with a 2026-10-04 revision for the hospital assistance scenario and the current message-delivery code. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions; no live robot was tested for this revision. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [snapshot](SOURCE_SNAPSHOT.md) for provenance. Repository links resolve from this folder; cited source line numbers belong to the original investigation unless a current-source review is identified.

**C11. Inference performance and resource placement**

TensorRT FP16 applies to the configured Jetson person, face, and embedding engines—not the entire pipeline.

| Component | Execution path | Evidence limits |
|---|---|---|
| YOLOX-s | Jetson TensorRT; explicit configured backend. | Auto-mode CPU fallback exists, but explicit TensorRT configuration fails instead of silently downgrading. |
| YuNet | Jetson TensorRT. | Recorded engine contained both FP16 and FP32 tensors. |
| AntelopeV2 | Jetson TensorRT, batch-one 112×112 input. | Recorded engine contained 829 FP16 tensors and one FP32 tensor. |
| SegFormer-B0 | Mac PyTorch/Transformers, normally MPS. | No TensorRT or explicit half-precision conversion in this path. |
| Depth Anything V2 | Mac PyTorch/Transformers, normally MPS. | Metric model designation does not establish accurate metres on this camera. |
| Gemini | Cloud through Mac adapters. | Current source selects `gemini-3.5-flash-lite`, `MINIMAL`; dated results used other models. |
| GeoCalib / range anchoring alternatives | Diagnostic experiments. | Not promoted into the active approach policy. |

The TensorRT builder records input/output shapes, hashes, TensorRT version, device, build time, and inspector precision counts. Face inference reads measured precision from an engine sidecar at startup; it does not independently re-inspect every engine tensor on every launch. “Mixed-precision TensorRT engines with FP16 enabled and inspected” is more precise than “all inference is FP16.” See engine metadata/build logic, lines 58–105 and 174–228 — `robot/jetson/perception/build_engine.py:58` and current cloud model constant — `robot/cloud_models.py:1`.

The recorded Jetson inventory is Ubuntu 22.04.5, L4T 36.4.3, ROS Humble, Python 3.10.12, OpenCV 4.5.4, and roughly 7.4 GiB RAM. The EV3 inventory is ev3dev Stretch and Python 3.5.3. These are historical inventory records, not refreshed device observations.

Model-stage speed does not equal mission responsiveness. A ~17 ms person detector coexists with seconds of camera movement/settling, cloud calls, body/range waiting, and retries. Recorded integrated runs lasting 30–100 seconds are compatible with fast neural inference.

I found token usage and call timings in selected cloud artifacts, but not a complete, reproducible cost-per-mission calculation. Nor did I find a controlled whole-stack latency distribution or end-to-end memory/compute profile for the final current configuration.

[Back to reading guide](README.md)
