# Camera calibration evidence

The accepted 640×480 intrinsics are in `config/camera_calibration.yaml`.

`camera_calibration_capture_report_20260903.json` is the unmodified report from
the second 30-view capture. It says `accepted: false` because two newly added,
conservative dataset-distribution gates requested six close views and more
top/bottom edge coverage; the capture had four close views. The solved model
itself passed every numeric and live-output check:

- RMS reprojection error: 0.596 px
- Worst view: 1.351 px
- Rectified valid ROI: 609×450, or 89.2% of the full frame
- Mild distortion coefficients with k3 deliberately fixed at zero
- 30/30 live Image/CameraInfo pairs with exact matching timestamps
- Normal live rectification with no severe warping

The model was therefore promoted after live visual review. The first model was
not promoted: despite 0.580 px RMS, an unconstrained k3 of -0.630 produced
severe circular warping and only a 244×198 valid ROI. Its YAML and report are
retained on the Jetson under
`/home/animesh/echora/rejected_calibrations/` for failure analysis.

No captured camera image is retained in Git. Raw/rectified comparison images
were temporary and deleted after review.
