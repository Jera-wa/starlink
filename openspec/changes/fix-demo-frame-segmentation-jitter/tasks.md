## 1. Frame Ingress and Queueing

- [x] 1.1 Identify or implement the image-refresh logical frame extractor at UART ingress and queue complete frames instead of raw contiguous ring-buffer slices.
- [x] 1.2 Add bounded frame-queue storage and backpressure handling for control frames, 525-byte frames, and oversized payloads without reintroducing silent drops.

## 2. Deterministic SLE Fragmentation and Reassembly

- [x] 2.1 Add internal fragment metadata plus deterministic fragment sizing based on runtime negotiated payload limits and documented legal connection/data-length settings.
- [x] 2.2 Implement receiver-side reassembly and defer UART egress until a complete logical frame is available.
- [x] 2.3 Change UART egress to issue one DMA write per complete logical frame and track completion or error explicitly.

## 3. Validation and Regression Targets

- [x] 3.1 Log effective payload, requested versus negotiated connection parameters, and per-frame delay metrics for image-refresh runs.
- [ ] 3.2 Re-run the image-refresh capture and verify that 13-byte and 32-byte control frames remain intact, 525-byte frames are emitted once per frame, and 2151-byte frames fragment only by deterministic payload rules.
- [ ] 3.3 Document measured send-to-receive and intra-frame gap results against the baseline captured in `发送数据.csv` and `采集数据.csv`.
