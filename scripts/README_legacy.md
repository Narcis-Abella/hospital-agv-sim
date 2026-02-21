# Legacy Python Noise Nodes

These Python scripts were the original noise modeling implementation.
They were superseded by the C++ nodes in `src/` due to measurable
latency introduced by the Python GIL under 100 Hz sensor callbacks.

Kept for reference and benchmarking purposes.