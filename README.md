# Ceres Workshop 教學專案

## 專案簡介

本專案旨在教學如何使用 Ceres Solver 進行數值優化，並結合實際案例進行學習。Ceres 是一個功能強大的 C++ 數值優化庫，廣泛應用於計算機視覺、機器人學等領域。本專案包含多個模組，涵蓋自動微分、數值微分、雅可比矩陣計算等主題，幫助使用者深入理解 Ceres 的使用方法。

## 專案結構

```
CMakeLists.txt       # 頂層 CMake 配置文件
bin/                 # 編譯後的可執行文件
build/               # 編譯過程中的中間文件
src/                 # 源碼目錄
  ch6/               # 第六章相關代碼
  common/            # 公用函數與工具
  curve_fitting_autodiff/  # 自動微分曲線擬合
  curve_fitting_numdiff/   # 數值微分曲線擬合
  pose_graph_autodiff/     # 自動微分位姿圖優化
  ...
data/               # 測試數據
lib/                 # 編譯生成的庫文件
```

## 環境需求

- 操作系統：Linux
- 編譯器：支持 C++11 的編譯器
- CMake：3.1 或更高版本
- Ceres Solver：已安裝
- Eigen：線性代數庫

## 編譯與運行

1. **配置環境**
   確保已安裝 Ceres Solver 和 Eigen。

2. **編譯專案**
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **運行示例**
   編譯完成後，可執行文件將生成於 `bin/` 目錄下。例如：
   ```bash
   ./bin/curve_fitting_autodiff
   ```

## 主要模組說明

### 1. 曲線擬合模組
- **curve_fitting_autodiff**：使用自動微分進行曲線擬合。
- **curve_fitting_numdiff**：使用數值微分進行曲線擬合。

### 2. 位姿圖優化模組
- **pose_graph_autodiff**：基於自動微分的位姿圖優化。
- **pose_graph_jacobian**：基於雅可比矩陣的位姿圖優化。

## 推導相關說明

### 1. Pose Graph 自動微分
在 `src/pose_graph_autodiff/include/pose_manifold.hpp` 文件中，提到需要定義一個 `PlusJacobian`，以便建立連鎖律，讓 Ceres 可以推導出終極 Jacobian。

### 2. 第六章推導細節
在 `src/ch6/main.cpp` 文件中，包含以下推導相關的註解：
- **第 61 行**：提供了一個推導的參考連結：[https://www.cnblogs.com/vivian187/p/16502590.html](https://www.cnblogs.com/vivian187/p/16502590.html)。
- **第 65 行**：說明擾動推導與更新方式一致性的重要性。
- **第 181 行**：指出手動推導的 Jacobian 通常用矩陣方式表達更佳。

這些細節對於深入理解 Ceres 的數值優化過程非常重要，建議讀者在學習時參考相關代碼與註解。

## 文件說明

- `src/common/rotation.hpp`：提供旋轉相關的工具函數。
- `src/fileio/txtio.hpp`：處理文件讀寫的工具。
- `data/`：包含測試所需的數據文件。

## 學習目標

- 理解 Ceres Solver 的基本概念與使用方法。
- 掌握自動微分與數值微分的實現。
- 學會如何構建與優化位姿圖。

## 聯繫方式

如有任何問題，歡迎聯繫作者或提交 Issue。

---

本專案由 NCKU-He-Rong 團隊開發，僅供學術研究與教學使用。