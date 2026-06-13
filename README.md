# PA02 : Fast Correlative Scan Matcher 전체 함수 대상 고속화

**이은혜 (12243788)**

---

# 프로젝트 개요

본 보고서는 전체 함수 성능 향상을 목적으로 수행한 PA02 과제의 최종 코드 설명 및 실행 방법에 대한 내용입니다.
PA01에서는 `score_all()` 함수만을 대상으로 CPU 및 GPU 최적화를 수행하였습니다. 그러나 전체 프로파일링 결과 `Score()`, `Branch()` 함수가 여전히 주요 병목으로 나타났으며, 특정 함수의 성능 향상이 전체 matcher 성능 향상으로 항상 이어지는 것은 아님을 확인하였습니다. 따라서 PA02에서는 개별 함수의 실행 시간뿐 아니라 함수 간 호출 구조와 trade-off를 함께 고려하여 최종 버전을 선정하였습니다.
현재 컨테이너에는 최종 보고서에서 채택한 버전만 반영되어 있으며, Jetson Nano 환경에서 실행 가능하도록 구성되어 있습니다.

---

# 실행 방법

## 1. Jetson Nano 접속 및 Docker 컨테이너 실행

Jetson Nano에 접속한 후 Docker 컨테이너를 실행합니다.

```bash
ssh -p 22 rcv@112.171.196.32

ssh student_47@192.168.0.112

sudo docker start student_47
sudo docker exec -it student_47 /bin/bash
```

---

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

또한, 아래 경로를 통해 최종 코드를 확인할 수 있습니다.

```bash
nano /root/catkin_ws/src/cartographer_parallel/cartographer_parallel/src/fast_matcher.cpp

nano /root/catkin_ws/src/cartographer_parallel/cartographer_parallel/src/score_all.cu
```

---

# 최종 적용된 최적화

보고서 기준 최종 채택 버전은 다음과 같습니다.

| 함수          | 최종 적용 버전                     |
| ----------- | ---------------------------- |
| score_all() | GPU + default stream sync 제거 |
| Score()     | vector 재사용                   |
| Branch()    | baseline 유지                  |
| MakeScans() | OpenMP 병렬화 (threshold=8)     |

보고서에서는 위 네 개 함수를 중심으로 최적화를 수행하였으며, 최종 코드 역시 해당 결과를 반영하였습니다.

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

실제 `MatchWithWindow()` 내부에서는 `MakeScans()`, `MakeBounds()`, `MakeGridStack()`, `MakeLowCands()`를 순서대로 수행한 뒤 coarse candidate에 대해 `Score()`를 수행합니다. 이후 `Branch()`가 탐색 공간을 계층적으로 분할하며 각 depth에서 `Score()`를 호출하고, `Score()` 내부에서 GPU `score_all()`을 호출하여 최종 score를 계산합니다.

---

# score_all() - GPU Score 계산

## 최종 적용

* GPU 사용
* default stream sync 제거

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

    cudaDeviceSynchronize() 호출 제거

    score 결과를 host로 복사

    candidate score 갱신

    return candidates
```

## 최종 버전 선정 이유

`score_all()`은 모든 candidate에 대해 동일한 계산을 반복하는 데이터 병렬 구조를 가집니다. 따라서 GPU 병렬 처리에 적합하며, 실제 프로파일링에서도 전체 matcher 실행 시간의 상당 부분을 차지하였습니다.
또한 CUDA default stream에서는 Host→Device 복사, Kernel 실행, Device→Host 복사가 순차적으로 수행되므로 kernel 직후의 `cudaDeviceSynchronize()`는 불필요하다고 판단하였습니다. 이에, 최종 버전에서는 해당 동기화를 제거하여 불필요한 대기 시간을 줄였습니다.

## 수도코드에서 반영된 부분

```text
CUDA kernel 실행

...

cudaDeviceSynchronize() 호출 제거

score 결과를 host로 복사
```

위 부분이 보고서에서 채택한 "default stream sync 제거" 최적화에 해당합니다.

---

# Score() - Candidate Score 관리

## 최종 적용

* vector 재사용

## 역할

candidate를 scan별로 분류하고 `score_all()` 호출을 관리합니다.

## 수도코드

```text
function Score(candidates, scans):

    ids, cx, cy vector 생성

    reserve()를 통해 필요한 메모리 확보

    for each scan:

        ids.clear()
        cx.clear()
        cy.clear()

        기존 vector 메모리는 유지

        현재 scan에 해당하는 candidate 분류

        candidate 정보를 vector에 저장

        score_all() 호출

        score 결과를 candidate에 반영

    score 기준 정렬

    return candidates
```

## 최종 버전 선정 이유

`Score()` 함수는 matcher 전체에서 매우 많은 횟수로 호출됩니다. 프로파일링 결과 score 계산 자체보다 vector 생성 및 메모리 할당 비용이 반복적으로 발생하고 있음을 확인하였습니다. 따라서 매 호출마다 vector를 새로 생성하는 대신, 한 번 확보한 메모리를 재사용하는 방식을 채택하였습니다.

## 수도코드에서 반영된 부분

```text
reserve()를 통해 필요한 메모리 확보

...

ids.clear()
cx.clear()
cy.clear()
```

`reserve()`로 필요한 메모리를 미리 확보한 뒤 `clear()`만 수행하여 기존 메모리를 재사용합니다.

---

# Branch() - Branch and Bound 탐색

## 최종 적용

* baseline 유지

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

Branch 함수는 재귀 탐색과 pruning이 결합된 구조를 가집니다. 여러 개선을 시도하였으나 함수 자체의 실행 시간 감소가 전체 matcher 성능 향상으로 이어지지 않았으며, 오히려 pruning 효과가 감소하는 경우도 확인하였습니다. 따라서 최종적으로는 baseline 구현을 유지하였습니다.


---

# MakeScans() - Yaw별 Scan 생성

## 최종 적용

* OpenMP 병렬화
* threshold = 8

## 역할

입력 point cloud를 여러 yaw 각도로 회전시켜 scan 집합을 생성합니다.

## 수도코드

```text
function MakeScans(raw_points, yaw_values):

    if yaw 개수 >= 8:

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

Yaw별 scan 생성은 서로 독립적으로 수행되므로 병렬화가 가능합니다. 따라서 yaw loop에 OpenMP를 적용하여 여러 yaw를 동시에 계산하도록 하였습니다.
다만, yaw 개수가 적은 경우에는 OpenMP thread 생성 비용이 오히려 더 크게 작용할 수 있으므로 threshold를 적용하였습니다. 실험 결과 threshold=8이 가장 좋은 성능을 보였으며 최종 버전으로 채택하였습니다.

---

# 결론

본 최종 버전은 개별 함수의 실행 시간보다 전체 matcher 성능을 기준으로 선정하였습니다.

최종적으로

* `score_all()` : GPU 계산 + sync 제거
* `Score()` : vector 재사용
* `Branch()` : baseline 유지
* `MakeScans()` : OpenMP 적용

을 채택하였습니다.

실험 결과 개별 함수의 성능 향상이 항상 전체 matcher 성능 향상으로 이어지는 것은 아니었으며, 함수 간 호출 구조와 데이터 흐름을 함께 고려한 프로파일링 기반 최적화가 중요함을 확인하였습니다. 특히 `Branch()`의 pruning 효과와 `Score()`의 호출 구조가 전체 성능에 큰 영향을 미쳤으며, 단순히 가장 느린 함수를 최적화하는 것보다 전체 호출 구조를 고려한 최적화가 중요함을 확인하였습니다.
