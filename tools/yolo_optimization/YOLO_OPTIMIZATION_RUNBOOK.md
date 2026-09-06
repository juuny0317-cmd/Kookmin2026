# Ryzen 7 5700U YOLO CPU 최적화 runbook

## 범위와 동결 조건

이 runbook은 대회 차량 Ryzen 7 5700U의 CPU FP32 경로만 다룬다.
NVIDIA/CUDA/FP16/TensorRT 설치나 실행은 하지 않는다. INT8도 이 단계에서는
사용하지 않는다.

다음 값은 C1과 동일하게 유지한다.

- `frame_router_event_driven=true`
- `stanley_event_driven=true`
- `mission_event_driven_lane=false`
- `stanley_state_timeout_s=0.30`
- lane model SHA256:
  `dd2f5e40dba630daede6bcf12d611327de9d42bc1206db2d7c518016748ab55b`
- image size 320, class `center_line`
- inference confidence 0.20, output confidence 0.25, custom NMS IoU 0.60
- Hough, Stanley, speed, mission timeout/logic 값

대상 bag은 다음 하나다.

```text
/home/xytron/xycar_ws/latency_bags/C1_20260809_180504
```

모든 offline run은 camera/lidar/ultrasonic driver를 시작하지 않으며 최종 motor
출력을 `/cpu_ab/motor`로 보낸다. 실제 `/xycar_motor`에는 publish하지 않는다.

## 확인된 runtime 동작

- PyTorch 2.6.0+cu124, `torch.cuda.is_available() == false`
- CPU FP32 Ultralytics PyTorch `AutoBackend`
- MKL/OpenMP 사용 가능, OpenCV thread 1
- logical CPU 0-1, 2-3, ..., 14-15가 각각 SMT sibling이다.
- lane affinity `0-7`은 8 physical core가 아니라 4 physical core/8 logical CPU다.
- CPU frequency driver는 `amd-pstate-epp`, governor는 `powersave`다.

Ultralytics는 첫 CPU predictor 생성 시 PyTorch intra-op thread 수를 내부 기본값
8로 다시 설정한다. 첫 predictor call 이전에만 `torch.set_num_threads()`를 호출한
기존 A/B는 실제로 모두 intra=8이었다. 현재 node와 benchmark는 backend setup
직후 요청값을 다시 적용하고 requested/actual 값을 검증한다. runtime JSON의
`cpu_thread_configuration.verified_after_backend_setup`가 반드시 `true`여야 한다.

## 현재 offline 결론

차량 주행 전 후보는 다음과 같다.

```text
lane intra-op threads: 4
lane inter-op threads: 1
lane CPU affinity: 0-7
OpenCV threads: 1
OMP_DYNAMIC: FALSE
backend/device/dtype: PyTorch eager / CPU / FP32
```

inter-op 16/4/1은 intra=8 단독 추론에서 사실상 동률이었다. 단일 eager graph에
16을 유지할 근거가 없으므로 1을 후보로 선택했다. 전체 stack에서 intra=4,
inter-op=1과 intra=4, inter-op=16도 측정 오차 범위였다.

전체 C1 raw replay의 profiler-ON 대표 결과는 다음과 같다. 이 replay의
`/clock`/recording 부하는 실제 C1 live run과 완전히 같지 않으므로 절대값보다
동일 harness 내 A/B와 cadence를 사용한다.

| intra/inter | inference median/p95 ms | worker median/p95 ms | queue median/p95 ms | processed/received |
|---|---:|---:|---:|---:|
| 8/16 | 85.023 / 102.982 | 88.803 / 106.729 | 33.359 / 70.752 | 105 / 115 |
| 6/16 | 78.849 / 101.145 | 82.799 / 105.791 | 30.339 / 76.367 | 111 / 116 |
| 4/16 | 52.520 / 63.984 | 55.810 / 67.405 | 0.512 / 7.335 | 115 / 115 |
| 2/16 | 57.170 / 74.619 | 60.686 / 77.902 | 0.528 / 10.763 | 115 / 115 |
| **4/1** | **50.706 / 63.394** | **53.846 / 66.507** | **0.520 / 5.092** | **115 / 115** |

C1 recorded reference는 inference 68.933/86.914 ms, worker 72.445/90.058 ms,
queue 0.824/23.101 ms다. 별도 raw replay의 4/1은 detection, center curve,
lane state가 약 12.04-12.08 Hz이고 stale stop은 0이었다. 같은 analyzer의 offline
source→first following motor record proxy는 C1 198.193/244.064 ms에서
4/1 146.890/186.834 ms로 개선 가능성을 보였다. 이는 물리 motor timestamp가
아닌 rosbag record proxy이므로 실차 latency로 단정하지 않는다.

고정 114 frame regression에서 8/16 대비 4/1은 다음 항목이 모두 정확히
동일했다.

- detection count, ordered class, bbox, confidence, custom NMS output
- center_curve point/state와 Hough downstream 값
- lane control categorical/numeric state
- 같은 source의 첫 angle/speed command

## 환경 기록

```bash
cd /home/xytron/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/yolo_optimization/inspect_cpu_environment.py \
  --output /home/xytron/Downloads/analysis/cpu_yolo_optimization/cpu_environment.json
```

이 파일에는 CPU topology, SMT sibling, per-core 현재 frequency, scaling
driver/governor, memory, load average, PyTorch intra/inter-op, MKL/OpenMP,
OpenCV thread 설정이 들어간다. NVIDIA device나 command는 조회하지 않는다.

## 단독 same-frame thread screen

각 조합은 새 process에서 실행한다. 첫 predictor setup 뒤 thread 값을 재적용하고
114개 동일 routed frame의 검출까지 비교한다. 이 결과만으로 최종 설정을 선택하지
않는다.

```bash
python3 src/yolo_optimization/run_cpu_thread_sweep.py \
  /home/xytron/xycar_ws/latency_bags/C1_20260809_180504 \
  --model /absolute/path/to/center_line_yolov10n_320_best.pt \
  --topic /perception/lane/image \
  --intra-op 8 6 4 2 --inter-op 16 --repeats 2 \
  --cpu-affinity 0-7 \
  --output /tmp/c1_cpu_intra.json
```

inter-op은 intra=8에서 16/4/1만 먼저 비교한다.

```bash
python3 src/yolo_optimization/run_cpu_thread_sweep.py \
  /home/xytron/xycar_ws/latency_bags/C1_20260809_180504 \
  --model /absolute/path/to/center_line_yolov10n_320_best.pt \
  --intra-op 8 --inter-op 16 4 1 --repeats 2 \
  --cpu-affinity 0-7 --output /tmp/c1_cpu_interop.json
```

## motor-isolated 전체 stack raw replay

현재 workspace source tree에는 lane model과 `mission_cone_drive` Python module
일부가 없으므로, 복구 전에는 검증에 사용한 외부 module/model 경로를 명시해야
한다. 차량 배포 전에는 이 의존성을 workspace 안의 검증된 artifact로 복구한다.

```bash
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash
export PYTHONPATH=/home/xytron/aug7final_ws/src/mission_cone_drive:${PYTHONPATH:-}

bash src/yolo_optimization/run_c1_cpu_replay.sh \
  /home/xytron/xycar_ws/latency_bags/C1_20260809_180504 \
  /absolute/path/to/center_line_yolov10n_320_best.pt \
  /home/xytron/Downloads/analysis/cpu_yolo_optimization/full_stack \
  intra4_inter1 4 1 on 0-7 false 30 raw
```

`CLOCK_HZ=30`은 rosbag의 simulated clock publish rate일 뿐 C1 제어/모델
파라미터가 아니다. 초기 100 Hz clock run은 DDS/callback 부하가 커져 YOLO
cadence를 왜곡했으므로 최종 thread 표에 사용하지 않는다.

분석:

```bash
python3 /home/xytron/Downloads/analysis/latency_optimization/analyze_yolo_profile.py \
  /path/to/run/bag --output-dir /path/to/run/yolo_analysis

python3 src/yolo_optimization/analyze_yolo_resource_correlation.py \
  /path/to/run/bag --resources /path/to/run/resources.csv \
  --output-dir /path/to/run/resource_analysis

python3 src/yolo_optimization/summarize_cpu_replay.py \
  /path/to/run/bag --motor-topic /cpu_ab/motor \
  --output /path/to/run/replay_summary.json
```

## source-aligned downstream regression

raw mode의 router rate gate는 원본 30 Hz 중 run마다 다른 약 12 Hz frame을 고를
수 있다. 따라서 raw mode는 contention/cadence용이고, 회귀 판정에는 C1에 이미
기록된 114 routed frame을 직접 재생하는 `fixed_lane`을 사용한다. scene YOLO와
router sampling은 이 회귀 모드에서 의도적으로 제외된다.

```bash
# Reference
bash src/yolo_optimization/run_c1_cpu_replay.sh \
  /home/xytron/xycar_ws/latency_bags/C1_20260809_180504 \
  /absolute/path/to/center_line_yolov10n_320_best.pt /tmp/c1_fixed \
  intra8_inter16 8 16 off 0-7 false 30 fixed_lane

# Candidate
bash src/yolo_optimization/run_c1_cpu_replay.sh \
  /home/xytron/xycar_ws/latency_bags/C1_20260809_180504 \
  /absolute/path/to/center_line_yolov10n_320_best.pt /tmp/c1_fixed \
  intra4_inter1 4 1 off 0-7 false 30 fixed_lane

python3 src/yolo_optimization/compare_pipeline_bags.py \
  /tmp/c1_fixed/intra8_inter16/bag \
  /tmp/c1_fixed/intra4_inter1/bag \
  --output /tmp/c1_fixed/source_aligned_regression.json
```

`pipeline_regression_pass`가 true여야 한다.

## resource profiler와 overhead

profiler는 launch default OFF다. 2 Hz standard mode가 기록하는 항목:

- target YOLO process CPU, RSS, thread count, affinity, context switch,
  page fault, process I/O
- total CPU, per-core CPU JSON, load average, CPU PSI
- per-core `scaling_cur_freq`, frequency average/min/max
- bag player/recorder, lane/scene YOLO, Hough, router, Stanley, mission 등
  알려진 process 역할별 CPU/thread/RSS/I/O
- inference, model, total worker, queue timestamp와 가장 가까운 resource sample

standard mode의 callback work는 대표 run에서 median 6.357 ms/500 ms,
profiler process CPU는 한 core 기준 median 7.49%(전체 16 logical capacity의
0.468%)였다. profiler ON/OFF 후보 run의 inference/worker 차이는 run noise
범위였지만 overhead는 0이 아니므로 최종 latency 채택값은 profiler OFF를 함께
보관한다. `PROFILE_THREADS=true` 상세 thread mode는 진단용이며 default OFF다.

개별 inference와 resource sample의 연결 해상도를 확인하기 위한 5 Hz 진단도
한 번 수행했다. nearest timestamp delta는 2 Hz의 median/p95
100.1/233.7 ms에서 5 Hz의 33.5/100.0 ms로 줄었다. 5 Hz callback work는
5.173 ms/200 ms, profiler CPU는 한 core 기준 10.07%(전체 capacity 0.629%)였다.
이 run에서도 inference와 frequency/total CPU/queue의 강한 양의 상관은 나오지
않았다. 5 Hz는 원인 진단용이고 standard/default는 계속 2 Hz/OFF다.

## 13.8 ms와 C1 68.9 ms 검증 결론

기존 13.8 ms는 과거 runbook의 sensor/motor-OFF warmup 값이다. 그 기록은
PyTorch 2.13.0+cu130과 RTX 4060이 장착된 다른 PC(CUDA unavailable)를 명시한다.
현재 Ryzen 차량 runtime은 PyTorch 2.6.0+cu124이므로 같은 runtime 비교가 아니다.
현재 차량 software/hardware에서 13.8 ms는 재현되지 않았다.

현재 Ryzen에서 같은 114 frame을 사용한 단계별 관측은 다음과 같다.

- 단독 intra=8: 약 31-32 ms median
- scene/router를 제외한 fixed-lane stack intra=8: 52.439 ms
- raw 전체 stack intra=8: 85.023 ms
- raw 전체 stack 최적 후보 intra=4/inter=1: 50.706 ms

raw 후보에서 total CPU median 54.7%, lane YOLO 약 2.61 cores, scene YOLO
0.82 core, lane detector 0.45 core, recorder 0.30 core였다. 빠른/느린 inference
quartile의 frequency는 거의 같았고 resource correlation도 약했다. 따라서 현재
증거는 다음을 지지한다.

- 확인됨: Ultralytics의 thread override와 전체 ROS/process contention
- 확인됨: intra=8에서 queue/drop이 발생하고 intra=4에서 제거됨
- 영향 있음: rosbag record, scene YOLO, Hough와 기타 executor/process load
- 주원인 증거 없음: 이 run 안의 CPU frequency 변동
- 직접 비교 불가: 과거 13.8 ms의 다른 hardware/PyTorch runtime

## ONNX Runtime 결정과 실차 전 남은 일

현재 차량에는 `onnx`, `onnxruntime`, `onnxruntime-gpu`가 설치되어 있지 않다.
PyTorch thread/scheduling만으로 median/p95, cadence, queue, exact regression gate를
통과했으므로 ONNX Runtime 설치/변환은 지금 수행하지 않는다. 실차 stationary
replay 반복에서 후보가 재현되지 않거나 실제 주행 A/B 목표를 만족하지 못할 때만
ONNX Runtime CPU FP32를 다음 후보로 연다. INT8은 그 뒤다.

실차 주행 전 남은 gate:

1. workspace에 누락된 mission Python source와 검증 model artifact를 복구한다.
2. 차량의 최종 배포 workspace에서 raw profiler OFF/ON을 각 3회 교차 순서로
   반복해 thermal/run-order 신뢰구간을 만든다.
3. runtime JSON의 CPU/FP32, 4/1, affinity 0-7을 다시 확인한다.
4. stationary replay에서 약 12 Hz cadence, zero drop, stale stop 0,
   source-aligned regression PASS를 재확인한다.
5. 그 뒤에만 기존 C1과 motor를 실제 연결한 짧은 4/1 A/B를 수행한다.
