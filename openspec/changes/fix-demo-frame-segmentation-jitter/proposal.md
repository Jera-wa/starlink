## Why

Recent trace capture from `发送数据.csv` and `采集数据.csv` shows that the demo bridge does not preserve logical frame boundaries under image-refresh traffic. 525-byte logical frames are emitted as 2-3 UART bursts with roughly 7-20 ms intra-frame gaps, while send-to-receive delay clusters around roughly 13-26 ms, which makes short frames look randomly fragmented and prevents deterministic handling of larger payloads such as 2151-byte frames.

## What Changes

- Define frame-aware bridge behavior so logical UART frames are queued as units instead of raw contiguous ring-buffer slices.
- Define deterministic SLE fragmentation and receiver-side reassembly so internal wireless chunking does not leak to UART egress.
- Define legal runtime transport tuning and trace-based regression criteria for image-refresh traffic, including actual negotiated limits and gap measurements.

## Capabilities

### New Capabilities
- `demo-frame-integrity-transport`: Preserve logical frame boundaries across the demo UART<->SLE bridge, apply deterministic fragmentation when transport limits require it, and expose regression signals for image-refresh workloads.

### Modified Capabilities
None.

## Impact

- Affected code: `application/samples/demo/demo.c`, `application/samples/demo/demo_sle.c`, `application/samples/demo/demo_uart.c`, and `application/samples/demo/demo_config.h`.
- Affected behavior: UART ingress framing, SLE fragment sizing, receiver-side reassembly, UART DMA egress, and connection/data-length tuning logs.
- Affected validation: image-refresh capture workflow using `发送数据.csv` and `采集数据.csv`, plus follow-up transport traces after implementation.
