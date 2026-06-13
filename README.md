# PA02 : Fast Correlative Scan Matcher 전체 함수 대상 고속화

**이은혜 (12243788)**

---

# 프로젝트 개요

본 프로젝트는 ROS 기반 Fast Correlative Scan Matcher(FCSM)의 성능 향상을 목적으로 수행한 PA02 과제의 최종 코드이다.

PA01에서는 `score_all()` 함수만을 대상으로 최적화를 수행하였으나, 전체 프로파일링 결과 `Score()`, `Branch()` 함수가 여전히 주요 병목으로 나타났다. 따라서 PA02에서는 개별 함수의 실행 시간뿐 아니라 함수 간 호출 구조와 trade-off를 함께 고려하여 최종 버전을 선정하였다.

현재 저장소에는 최종 보고서에서 채택한 버전만 반영되어 있으며, Jetson Nano 환경에서 실행 가능하도록 구성되어 있다.

---

# 실행 방법

Workspace 최상위 디렉토리에서 빌드한다.

```bash
catkin_make
source devel/setup.bash
```

Fast Correlative Matcher 실행

```bash
roslaunch cartographer_parallel fast_correlative.launch
```

rosbag 포함 실행

```bash
roslaunch cartographer_parallel cartographer_parallel_with_bag.launch
```

---

# 최종 적용된 최적화

| 함수          | 최종 적용 버전                     |
| ----------- | ---------------------------- |
| score_all() | GPU + default stream sync 제거 |
| Score()     | vector 재사용                   |
| Branch()    | baseline 유지                  |
| MakeScans() | OpenMP 병렬화 (threshold=8)     |

---

# 전체 실행 흐름

```text
MakeScans()
      ↓
Branch()
      ↓
Score()
      ↓
score_all()
```

`MakeScans()`에서 yaw별 scan을 생성하고, `Branch()`가 탐색 공간을 계층적으로 줄인다. 이후 `Score()`가 candidate를 정리하여 `score_all()`에 전달하고, `score_all()`이 GPU에서 실제 matching score를 계산한다.

---

# score_all() - GPU Score 계산

최종 적용

* GPU 사용
* default stream sync 제거

역할

각 candidate pose에 대해 occupancy grid score를 계산한다.

수도코드

```text
function score_all(candidates, scan_points, grid):

    candidate 정보를 GPU 입력 배열로 변환

    host → device 복사

    CUDA kernel 실행

        각 block은 하나의 candidate 담당

        각 thread는 scan point 일부를 담당

        scan point를 grid에 투영

        score를 누적

        reduction 수행

    cudaDeviceSynchronize() 호출 제거

    score 결과를 host로 복사

    candidate score 갱신

    return candidates
```

---

# Score() - Candidate Score 관리

최종 적용

* vector 재사용

역할

candidate를 scan별로 분류하고 `score_all()` 호출을 관리한다.

수도코드

```text
function Score(candidates, scans):

    ids, cx, cy vector 생성

    reserve() 수행

    for each scan:

        ids.clear()
        cx.clear()
        cy.clear()

        현재 scan에 해당하는 candidate 분류

        candidate 정보를 vector에 저장

        score_all() 호출

        score 결과를 candidate에 반영

    score 기준 정렬

    return candidates
```

---

# Branch() - Branch and Bound 탐색

최종 적용

* baseline 유지

역할

탐색 공간을 재귀적으로 분할하며 가능성이 낮은 candidate를 제거한다.

수도코드

```text
function Branch(candidates, depth):

    if depth == 0:

        Score() 호출

        최고 score candidate 반환

    child candidate 생성

    upper bound score 계산

    가능성이 있는 child만 유지

    score 기준 정렬

    높은 score 후보부터 재귀 탐색

    return best candidate
```

---

# MakeScans() - Yaw별 Scan 생성

최종 적용

* OpenMP 병렬화
* threshold = 8

역할

입력 point cloud를 여러 yaw 각도로 회전시켜 scan 집합을 생성한다.

수도코드

```text
function MakeScans(raw_points, yaw_values):

    if yaw 개수 >= 8:

        OpenMP parallel for

            for each yaw:

                모든 point 회전

                scan 생성

                결과 저장

    else:

        일반 for loop 수행

    return scans
```

---

# 최종 결론

본 최종 버전은 함수 단위 실행 시간보다 전체 matcher 성능을 기준으로 선정하였다.

* `score_all()` : GPU 계산
* `Score()` : vector 재사용
* `Branch()` : baseline 유지
* `MakeScans()` : OpenMP 적용

실험 결과, 개별 함수 성능 향상이 항상 전체 matcher 성능 향상으로 이어지는 것은 아니었으며, 함수 간 호출 구조와 데이터 흐름을 함께 고려하는 것이 중요함을 확인하였다.
