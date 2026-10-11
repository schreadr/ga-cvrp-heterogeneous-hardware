The data extracted from the performance comparision in [Heer's thesis](https://doi.org/10.23860/thesis-heer-maximilian-2022) for the first three columns. The last column's data is from this project. See experiments other results.

### **Extended Performance & Convergence Time Table**

| Problem Instance | Heer's FPGA Impl Best Gap (Time to best sol) | Abdelatti et al. GPU Impl Best Gap (Time to best sol) | Abdelatti et al. CPU Impl Best Gap (Time to best sol) | Our Heterogeneous FPGA-CPU Impl Avg Gap ± Std (Time to best sol) |
| :--- | :--- | :--- | :--- | :--- |
| **P-n70-k10** | **6.5%** (10000 s) | **5%** (1250 s) | **2%** ( < 500s>) | **2.35% ± 0.74%** (25.2 ± 26.7 s) |
| **X-n101-k25** | **5%** (4800 s) | **5%** (800 s) | **5%** (2200 s) | **2.91% ± 0.81%** (37.8 ± 45.0 s) |
| **X-n125-k30** | **4.5%** (8000 s plateau) | **3%** (7500 s) | **~3.5%** (1000 s) | **2.77% ± 0.35%** (143.3 ± 70.3 s) |
| **X-n139-k10** | **7%** (10000 s) | **6%** (15000 s) | **6%** (7500 s) | **1.45% ± 0.50%** (177.4 ± 193.5 s) |
| **X-n148-k46** | **6%** (6000 s) | **4%** (4000 s) | **5%** (500 s) | **3.58% ± 1.11%** (133.5 ± 94.7 s) |

---

### **Detailed Statistical Breakdown (Our Implementation, 5 Seeds)**

| Problem Instance | Avg Gap (%) | Std Dev Gap (%) | Avg Time to Best Sol | Std Dev Time | Avg Total Wall Time |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **P-n70-k10** | **2.35%** | ± 0.74% | **25.2 s** | ± 26.7 s | 86.0 s |
| **X-n101-k25** | **2.91%** | ± 0.81% | **37.8 s** | ± 45.0 s | 172.6 s |
| **X-n125-k30** | **2.77%** | ± 0.35% | **143.3 s** | ± 70.3 s | 224.7 s |
| **X-n139-k10** | **1.45%** | ± 0.50% | **177.4 s** | ± 193.5 s | 435.9 s |
| **X-n148-k46** | **3.58%** | ± 1.11% | **133.5 s** | ± 94.7 s | 292.0 s |

---

### **Data Extraction Script**
The values in the last column are produced by [`extract_comparison_data.py`](extract_comparison_data.py):
```bash
python3 extract_comparison_data.py
```