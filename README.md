# Streaming Multiprocessor (SM) Unit

This module models the internal structure of a GPU Streaming Multiprocessor (SM), focusing on L1 cache, load/store units, and warp scheduling behavior.  
It provides a comprehensive validation framework for the SM’s memory subsystem components, designed for use with SST or standalone analysis.

---

## **Components**

### **1. L1 Sector Cache**
- 4-way set-associative cache
- 32-byte sectors with sector-based tag management
- Configurable hit/miss tracking
- Integrated Miss Status Holding Registers (MSHR) for handling multiple outstanding requests

### **2. Load/Store (LD/ST) Unit**
- Supports memory coalescing at warp level  
- Generates efficient global memory transactions  
- Tracks load/store queue utilization and memory throughput

### **3. MSHR**
- Request merging and dependency tracking  
- Manages outstanding cache misses  
- Validated against GPU microarchitecture papers

### **4. Memory Coalescing Engine**
- Groups 32-thread warp accesses into minimal memory transactions  
- Analyzes transaction count, coalescing efficiency, and bandwidth utilization  

---

## **Validation and Reference Framework**
Validated against foundational GPU architecture research and NVIDIA hardware models (Fermi, Kepler, Volta, Turing, and Ampere).  

### **Features**
-  9 L1 cache validation tests (hit rate, MSHR, sector efficiency)  
-  10 LD/ST unit tests (coalescing accuracy, bandwidth utilization)  
-  Cross-reference validation against ISPASS 2009 and HPCA 2014 papers  
-  One-command automated validation pipeline  
-  Visualization tools for performance and cache metrics  
-  Standalone build, no external dependencies  
-  Documentation citing 40+ academic references  

---

## **Next Steps**
Planned integration with:
- Warp Scheduler model
- Shared Memory and L2 interconnect interface
- SST integration hooks for latency and bandwidth analysis

