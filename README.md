# MorphBench

**Manipulation With Any Morphology** — project page and interactive in-browser demo.

🔗 Live site: **https://morphbench.github.io/**

Build a modular robot arm from parametric tube modules in the node-graph editor,
hit **Generate**, and explore it live in 3D — all in the browser, no install.
Part of the MorphBench manipulation benchmark spanning **190 modular arm morphologies**.

> 📄 Paper · 💻 Code · 🗃️ Dataset — releases coming soon.

## Local preview

```bash
python3 -m http.server 8000
# then open http://localhost:8000
```

## What's here

This repository is the **published static site only** — a self-contained snapshot of the
`web-viewer/` app. Third-party libraries (three.js, cytoscape, OrbitControls) are vendored
under `vendor/` so the live demo has no CDN/network dependency.

---

© Xinrui Zu, Vrije Universiteit Amsterdam.
