# Camera mounting and alignment protocol

**Status:** tooling complete; physical mounting and reference capture not completed.

Use two RGB views: wrist and overhead. Mount both rigidly, tape the mounts and cables, and
ensure neither can be moved by normal arm travel. Record the physical camera identity and
USB path, not only an OpenCV integer, because device indices may change after reboot.

The currently visible capture nodes are `/dev/video0` (icspring), `/dev/video2` (C920), and
`/dev/video4` (USB Camera). The operator must identify which two are the intended views.
Their paired metadata nodes (`video1`, `video3`, `video5`) are not recording choices.

## Reference and session ritual

1. Clear the workspace and place permanent visual registration marks in view.
2. Capture named reference images after the mounts are taped.
3. At every session start, capture new check images with `--verdict pass` only after visual
   comparison against the references. A failed check stops collection until corrected.
4. Record start/end servo health and temperature in the session log.
5. After recording one test episode, run the timing-sidecar audit. Any failure quarantines
   that episode and blocks corpus collection.

Reference and check manifests include device identity, requested settings, actual image
size, SHA-256, session ID, and operator verdict. A hash proves which image was reviewed; it
does not prove alignment by itself.

