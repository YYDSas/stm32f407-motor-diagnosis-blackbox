# W5 上位机工具实测记录（vib_monitor.py）

- **执行时间**：2026-11-25 20:45 ~ 20:52
- **主机环境**：Windows PowerShell 5.1，Python **3.9.13**（`D:\python-hjc\Software\Python39\Install\python.exe`），pyserial **3.5**、matplotlib **3.7.2**
- **串口**：**COM11** `USB-SERIAL CH340` @ 115200 8N1（余下 COM3/4/5/6 为蓝牙串口，COM8 为另一个 USB 串行设备）
- **板端状态**：`valid=1018 corrupt=0 next_seq=22009`（挂载正常，游标连续）

## 1. `dump` — 黑匣子导出 CSV

```powershell
cd motor_diag\tools
python vib_monitor.py dump --port COM11 --count 50 --out ..\..\Doc\w5-blackbox-dump.csv
```

| 项目 | 结果 |
| --- | --- |
| 请求条数 | 50（另试 100，结果相同） |
| 实际返回 | **32 条** |
| 统计 | `valid=1018 corrupt=0 next_seq=22009` |
| 故障分布 | OVERTEMP=11、IWDG_RST=9、LOOSE=7、OVERLOAD=3、BKP_MIG=2 |
| 产物 | `Doc/w5-blackbox-dump.csv`（33 行，含表头） |

**为什么只有 32 条**：不是脚本的问题，是固件端写死的缓冲上限 —— `motor_diag/APP/Src/app_blackbox.c:14`

```c
#define BB_DUMP_BUF_MAX      (32U)
```

即 `dump N` 的 N 最大只生效到 32，要导出更多需要分批或用 `watch` 落盘。

## 2. `snap` — 502-bin 频谱快照

```powershell
python vib_monitor.py snap --port COM11 --slot 32 --out ..\..\Doc\w5-snap-slot32.png
```

```
[SNAP 32] seq_low=21993 ts=138768ms fs=3200.0Hz bins=502 scale_max=8086.0747
  Peak #1:    9.38 Hz (bin   3) | amp=4367.6698
  Peak #2:  431.25 Hz (bin 138) | amp=3931.6185
  Peak #3:   40.62 Hz (bin  13) | amp=3030.3964
[OK] Saved spectrum plot -> Doc/w5-snap-slot32.png
```

- 采样率 `fs=3200 Hz`，分辨率 `3200/2/502 ≈ 3.19 Hz/bin`
- 产物：`Doc/w5-snap-slot32.png`（149 KB 图）+ `Doc/w5-snap-slot32.csv`（502 行原始 q15 幅值）

## 3. `watch` — 实时监听

```powershell
python vib_monitor.py watch --port COM11 --duration 120 --log ..\..\Doc\w5-watch-120s.log
```

```
[WATCH] Listening on COM11 @ 115200 bps
=== Session Summary (120.1 s) ===
Total faults : 0
```

**踩到的坑**：日志文件是 **0 字节**。原因是 `watch` 只被动接收、不主动下发指令，而板子在空闲态不会周期性打印，所以 2 分钟里串口一声不响。

> 结论：**“1 小时健康态稳态测试”不能直接用 `watch --duration 3600`**，否则跑完得到一份空日志、`Total faults: 0` 也不足以证明“无异常”，只能证明“没打印”。
>
> 可行替代：① 期间让板子处于 `stress` 或故障注入状态；② 用定时 `stat` 轮询（见下节）；③ 跑前跑后各 `dump` 一次，比对 `next_seq` 与 `corrupt`。

## 4. 补充实测：定时 `stat` 轮询（60 s）

用 pyserial 每 10~15 s 下发一次 `stat`，采集堆与栈水位：

| 采样点 | Free Heap | Min Ever Free | Task_DSP_FFT | Task_Blackbox | Task_CLI |
| --- | --- | --- | --- | --- | --- |
| T+0 s | 18 768 B | 18 432 B | 510 | 353 | 274 |
| T+10 s | 18 768 B | 18 432 B | 510 | 353 | 274 |
| T+20 s | 18 768 B | 18 432 B | 510 | 353 | 274 |
| T+50 s | 18 768 B | 18 432 B | 510 | 353 | 274 |

全任务栈水位（一次采样）：

```
Task_CLI      X  2   274      IDLE          R  0   117
Task_Super    B  15  248      Task_Sensor   B  22  208
defaultTask   B  24  212      Task_Blackbox B  20  353
Task_UI_LCD   B  10  322      Task_DSP_FFT  S  28  510
Tmr Svc       B  2   230
Free Heap: 18768 B | Min Ever Free: 18432 B
```

- 堆值 **45 s 内零漂移**，`Min Ever Free` 仅比当前低 **336 B**，无泄漏趋势。
- 与 W3 基线（`Free Heap 25 384 B`）相比少了约 **6.6 KB**，对应 LCD 驱动 + `Task_Sensor` / `Task_UI_LCD` / `Task_Super` 三个新任务的开销，属预期。

## 5. 环境与用法踩坑（供以后复现）

1. **路径重复**：在 `motor_diag\tools` 目录里再写 `tools/vib_monitor.py`，会拼成 `tools\tools\` 而报 `[Errno 2] No such file or directory`。要么 `cd ..` 后用带前缀的写法，要么留在 tools 里直接 `python vib_monitor.py`。
2. **`--port COMx` 是占位符**：必须替换为真实端口（本机为 COM11），否则打不开串口。
3. **依赖不用另装**：Python 3.9 环境已有 pyserial / matplotlib；用 WorkBuddy 内置的 3.13 解释器反而会 `ModuleNotFoundError`。
4. **串口独占**：运行前需关闭串口助手，否则报“拒绝访问”。

## 6. 待办（W5 验收清单剩余项）

| 项 | 状态 |
| --- | --- |
| IWDG 死锁自愈 10/10 | 黑匣子中已有 **9 条 `IWDG_RST`**，还差 1 次 |
| `-O0` vs `-O3` 耗时/CPU 对比 | 未记录 |
| 20 次故障注入（≥19/20 检出） | 未系统统计（当前累计 OVERTEMP 11 / OVERLOAD 3 / LOOSE 7） |
| 1 小时健康态稳态 | **未完成**：`Doc/w5-stability-1h.log` 实际只覆盖 21:04:39~21:05:10（约 30 s、29 行），不足以代表 1 h；需按第 3 节方案重跑并重新命名 |
