# Terrain-Aware Evolution of Pilots Trajectory Project

This code takes its base from this github repository (https://github.com/paulsaswata/Wind_Aware_Aircraft_Trajectory_Generation/tree/master).

The main objective is to add terrain-awareness when computing emergency trajectory paths by extending a series of Dubin paths together. The first iteration of this project concerned itself with Dubin path generation with wind augmentation.

Currently, here is the feature set of the newer edition:

## New Additions

- Automatic USGS Terrain Downloader
- Offline Terrain Data Downsampling
- Dynamic, Online Terrain Chunk Loading and Unloading
- Cubic-Spline Interpolated Sample-Index to Curvilinear Coordinate Transformation
- Policy-based Chunking for Terrain Resolution
- Boundary Intersection by Low Dimensional Nearest-Neighbor Search through Hierarchical Navigable Small World (HNSW)
- Python-Interface Integration (Coming soon)
- Parallelized Intersection Detection and Path Generation
- Monte-Carlo Tree Search for Partial Dubin Path Progressions
