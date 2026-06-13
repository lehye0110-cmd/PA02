# PA02 : Fast Correlative Scan Matcher 전체 함수 대상 고속화

**이은혜 (12243788)**

---

# 프로젝트 개요

본 보고서는 전체 함수 성능 향상을 목적으로 수행한 PA02 과제의 최종 코드 설명 및 실행 방법에 대한 내용입니다.

PA01에서는 `score_all()` 함수만을 대상으로 CPU 및 GPU 최적화를 수행하였습니다. 그러나 전체 프로파일링 결과 `Score()`, `Branch()` 함수가 여전히 주요 병목으로 나타났으며, 특정 함수의 성능 향상이 전체 matcher 성능 향상으로 항상 이어지는 것은 아님을 확인하였습니다.

따라서 PA02에서는 개별 함수의 실행 시간뿐 아니라 함수 간 호출 구조와 trade-off를 함께 고려하여 최종 버전을 선정하였습니다. CPU 버전은 OpenMP, vector 재사용 등 연산 구조를 중심으로, GPU 버전은 kernel 성능뿐 아니라 memory copy, synchronization, kernel launch overhead를 포함한 GPU 호출 구조를 중심으로 분석하였습니다.

현재 컨테이너에는 최종 보고서에서 채택한 버전만 반영되어 있으며, Jetson Nano 환경에서 실행 가능하도록 구성되어 있습니다.

---

# 실행 방법

## 1. Jetson Nano 접속 및 Docker 컨테이너 실행

```bash
ssh -p 22 rcv@112.171.196.32

ssh student_47@192.168.0.112

sudo docker start student_47
sudo docker exec -it student_47 /bin/bash
```

## 2. ROS Launch 실행

Docker 접속 후 아래 명령어를 실행합니다.

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

nvprof --profile-child-processes \
roslaunch cartographer_parallel cartographer_parallel_with_bag.launch ns:=student47
```

bag 재생이 완료된 후 `Ctrl + C`를 입력하면 Time Profiling 및 Memory Profiling 결과를 확인하실 수 있습니다.

최종 코드는 아래 경로에서 확인할 수 있습니다.

```bash
nano /root/catkin_ws/src/cartographer_parallel/cartographer_parallel/src/fast_matcher.cpp
nano /root/catkin_ws/src/cartographer_parallel/cartographer_parallel/src/score_all.cu
```

---

# 전체 실행 흐름

```text
MakeScans()
      ↓
MakeBounds()
      ↓
MakeGridStack()
      ↓
MakeLowCands()
      ↓
Score()
      ↓
Branch()
      ↓
score_all()
```

`MatchWithWindow()` 내부에서는 `MakeScans()`, `MakeBounds()`, `MakeGridStack()`, `MakeLowCands()`를 순서대로 수행한 뒤 coarse candidate에 대해 `Score()`를 수행합니다. 이후 `Branch()`가 탐색 공간을 계층적으로 분할하며 각 depth에서 `Score()`를 호출하고, `Score()` 내부에서 GPU `score_all()`을 호출하여 최종 score를 계산합니다.

---

# 함수별 CPU / GPU 선정 근거

| 함수 | CPU/GPU | 선정 이유 |
| --- | --- | --- |
| Score() | CPU | score_all 계산보다 candidate 분류 및 입력 준비 비용 비중이 커 CPU 구조 최적화 선택 |
| Branch() | CPU | 재귀 탐색 및 pruning 중심 구조로 CPU 최적화 선택 |
| MakeScans() | CPU | yaw별 계산이 독립적이며 OpenMP 효과가 커 CPU 선택 |
| MakeLowCands() | CPU | 메모리 접근 및 vector 관리 중심이라 CPU 선택 |
| score_all() | GPU | 실제 점수 계산을 수행하는 데이터 병렬 연산이므로 GPU 선택 |

---

# 최종 적용된 최적화

| 함수 | 최종 적용 버전 | 성능 변화 |
| --- | --- | --- |
| score_all() | GPU + default stream sync 제거 | Score total 약 1.6% 감소 (cudaDeviceSynchronize 124,867회 제거) |
| Score() | vector 재사용 (GPU ver) | Score total 약 3.2% 감소 (80430 ms → 79121 ms) |
| Branch() | baseline 유지 | 개선 시도 모두 전체 matcher 성능 악화로 미채택 |
| MakeScans() | OpenMP 병렬화 (threshold=8) | 1016 ms → 438 ms (약 2.3배 개선) |

> **주요 미채택 사례**
> - `branch_depth 4 → 3` : Branch total 28.8% 개선되었으나 Score total이 5902 ms → 26175 ms로 약 4.4배 증가하여 미채택. Branch의 역할은 자신의 실행 시간을 줄이는 것이 아니라 Score로 전달되는 candidate 수를 효과적으로 제어하는 데 있음을 확인.
> - `warp-level reduction` : kernel 실행 시간 5.7% 감소하였으나 전체 matcher 성능은 오히려 소폭 증가. GPU 병목이 kernel 내부보다 memory copy 및 synchronization overhead에 있었기 때문.

---

# score_all() - GPU Score 계산

## 최종 적용

- GPU 사용
- default stream sync 제거 (`cudaDeviceSynchronize()` 124,867회 제거)

## 역할

각 candidate pose에 대해 occupancy grid score를 계산합니다.

## 수도코드

```text
function score_all(candidates, scan_points, grid):

    candidate 정보를 GPU 입력 배열로 변환

    host → device 복사

    CUDA kernel 실행

        각 block은 하나의 candidate 담당
        각 thread는 scan point 일부를 담당
        scan point를 occupancy grid에 투영
        grid score 누적
        block 내부 reduction 수행

    cudaDeviceSynchronize() 호출 제거  ← default stream이 H2D→Kernel→D2H 순서를 이미 보장하므로 불필요

    score 결과를 host로 복사

    candidate score 갱신

    return candidates
```

## 최종 버전 선정 이유

`score_all()`은 모든 candidate에 대해 동일한 계산을 반복하는 데이터 병렬 구조를 가지므로 GPU 병렬 처리에 적합합니다.

GPU profiling 결과 `cudaMemcpy` 호출이 676,714회로 kernel 호출 및 synchronization 호출보다 약 5.4배 많아, 주요 병목이 kernel 계산보다 memory copy와 synchronization overhead에 있음을 확인하였습니다. CUDA default stream에서는 H2D → Kernel → D2H 순서가 이미 보장되므로 kernel 직후의 `cudaDeviceSynchronize()`는 불필요하며, 이를 제거(124,867회)하여 CPU가 GPU 완료를 기다리는 시간을 줄였습니다.

---

# Score() - Candidate Score 관리

## 최종 적용

- vector 재사용 (GPU 버전에서 채택)

## 역할

candidate를 scan별로 분류하고 `score_all()` 호출을 관리합니다.

## 수도코드

```text
function Score(candidates, scans):

    ids, cx, cy, score vector를 함수 외부에서 1회 생성

    reserve()를 통해 필요한 메모리 확보

    for each scan:

        ids.clear()   ← 메모리 해제 없이 size만 초기화
        cx.clear()
        cy.clear()

        현재 scan에 해당하는 candidate 분류

        candidate 정보를 vector에 저장

        score_all() 호출

        score 결과를 candidate에 반영

    score 기준 정렬

    return candidates
```

## 최종 버전 선정 이유

GPU 버전에서 `Score()` 함수의 실제 병목은 kernel 내부 연산뿐 아니라 GPU 호출 전 입력 데이터를 준비하는 CPU-side 과정에도 존재하였습니다. 반복적인 vector 생성과 메모리 할당을 `reserve()` + `clear()` 방식으로 교체하여 GPU 호출 준비 비용을 줄였으며, Score total이 80430 ms → 79121 ms로 약 3.2% 감소하였습니다.

> **참고** : CPU 버전에서는 동일한 vector 재사용이 오히려 5.6% 성능 저하를 보였습니다. CPU 버전의 주요 병목은 vector 생성·소멸이 아니라 candidate 구성과 `score_all()` 호출 과정 자체에 있었기 때문입니다.

---

# Branch() - Branch and Bound 탐색

## 최종 적용

- baseline 유지

## 역할

탐색 공간을 재귀적으로 분할하며 가능성이 낮은 candidate를 제거합니다.

## 수도코드

```text
function Branch(candidates, depth):

    if depth == 0:
        Score() 호출
        최고 score candidate 반환

    child candidate 생성

    upper bound score 계산

    현재 best score보다 가능성이 있는 후보만 유지

    score 기준 정렬

    높은 score 후보부터 재귀 탐색

    return best candidate
```

## 최종 버전 선정 이유

Branch 함수는 재귀 탐색과 pruning이 결합된 구조를 가집니다. 아래 세 가지 개선을 시도하였으나 모두 미채택하였습니다.

| 시도 | Branch 변화 | 전체 영향 | 결론 |
| --- | --- | --- | --- |
| child vector 재사용 | total 약 1.6% 느림 | 유의미한 변화 없음 | child vector 생성이 주요 병목 아님 |
| best score 이하 child pruning | 약 4.2% 느림 | pruning보다 비교 overhead가 큼 | 미채택 |
| branch_depth 4 → 3 | 28.8% 개선 | Score total 4.4배 증가 | 전체 matcher 기준 악화 |

Branch의 역할은 자신의 실행 시간을 줄이는 것이 아니라 Score로 전달되는 candidate 수를 효과적으로 제어하는 데 있으므로, baseline을 유지하였습니다.

---

# MakeScans() - Yaw별 Scan 생성

## 최종 적용

- OpenMP 병렬화
- threshold = 8

## 역할

입력 point cloud를 여러 yaw 각도로 회전시켜 scan 집합을 생성합니다.

## 수도코드

```text
function MakeScans(raw_points, yaw_values):

    if yaw 개수 >= 8:   ← threshold: OpenMP overhead보다 병렬화 이득이 큰 구간

        OpenMP parallel for

            for each yaw:
                current_scan 생성
                for each point:
                    yaw 만큼 회전
                    grid 좌표로 변환
                    current_scan에 저장
                scans[yaw_index] 저장

    else:
        일반 for loop 수행

    return scans
```

## 최종 버전 선정 이유

yaw별 scan 생성은 서로 독립적이므로 thread 간 데이터 의존성이 없어 OpenMP 병렬화에 적합한 구조입니다. 적용 결과 MakeScans total이 1016 ms → 438 ms로 약 2.3배 개선되었습니다.

threshold 실험 결과는 아래와 같습니다.

| threshold | MakeScans total |
| --- | --- |
| 4 | 467 ms |
| **8** | **438 ms (최적)** |
| 16 | 1016 ms (병렬화 기회 감소) |

threshold가 너무 작으면 OpenMP thread 생성 비용이 커지고, 너무 크면 병렬화 기회가 감소하므로 threshold=8을 최종 채택하였습니다.

---

# 결론

본 PA02에서는 개별 함수의 실행 시간보다 전체 matcher 성능을 기준으로 최적화 버전을 선정하였습니다.

실험을 통해 **함수 단위 최적화가 전체 matcher 성능 향상을 보장하지 않음**을 확인하였습니다. Branch depth 감소와 warp-level reduction은 개별 함수 성능은 개선되었지만, 호출 구조 및 overhead 증가로 인해 전체 성능은 오히려 악화되었습니다. 따라서 함수 실행 시간뿐 아니라 호출 구조와 데이터 흐름을 함께 고려한 프로파일링 기반 최적화가 중요함을 확인하였습니다.
