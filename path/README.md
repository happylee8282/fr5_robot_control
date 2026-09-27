# 경로 코드와 데이터

이 폴더에는 초록 경로를 로봇 노즐의 목표 위치·자세로 변환하는 코드가 있다.
실제 경로 파일은 `data/`에서 확인할 수 있다.

```text
path/
├─ cartesian_generator.py     초록 경로 → TCP 위치·자세 생성
├─ path_profile.py            구간별 노즐 간격 계산
├─ clearance_geometry.py      노즐 간격을 적용할 방향 계산
└─ data/
   ├─ green_path/             Step 2 경로 생성 결과로 연결
   └─ generated/              Step 3 경로·IK·실행 결과로 연결
```

## 실제 경로 파일

| 파일 | 내용 |
| --- | --- |
| [green_path/step2_1_accurate_global_path.csv](data/green_path/step2_1_accurate_global_path.csv) | Step 3 입력. 안경 모델 로컬 좌표의 초록 경로, 면 중심, 안쪽·바깥쪽 경계. 단위 mm |
| [green_path/step2_1_accurate_global_path.pcd](data/green_path/step2_1_accurate_global_path.pcd) | 초록 경로를 점군으로 저장한 파일 |
| [generated/cartesian_poses.csv](data/generated/cartesian_poses.csv) | 노즐 간격과 자세, 접근·이탈 구간을 반영한 TCP 경로. 위치 단위 m |
| [generated/cartesian_poses.npz](data/generated/cartesian_poses.npz) | MoveIt 경로 계산기가 읽는 TCP 경로 배열 |
| [generated/verified_joint_trajectory.npz](data/generated/verified_joint_trajectory.npz) | IK 및 검증을 거친 관절 위치·속도·가속도·시간 배열 |
| [generated/tracking_result.csv](data/generated/tracking_result.csv) | 실행 후 기록한 추종 결과 |

`green_path/`는 `step1_modelsolution/path_generation/step2_pcd_path/check_output/`,
`generated/`는 `step1_modelsolution/output/`에 대한 상대 심볼릭 링크다.
원본 파일을 그대로 열기 때문에 기존 생성·실행 흐름에서 파일이 갱신되면 여기에도
같이 반영된다. 링크를 통해 파일을 수정하면 원본도 수정된다.
이 연결은 소스 작업 폴더에서 파일을 찾아보기 위한 것으로, 이 Python 패키지
폴더만 따로 복사하면 외부의 두 데이터 폴더도 함께 배치해야 한다.

## 경로가 만들어지는 순서

```text
path_generation/의 Step 1·2 코드
    ↓
data/green_path/step2_1_accurate_global_path.csv
    ↓ cartesian_generator.py + 현재 안경 snapshot + 높이·자세 설정
data/generated/cartesian_poses.csv / .npz
    ↓ ../core/moveit_solver.py + MoveIt IK/FK·충돌 검사
data/generated/verified_joint_trajectory.npz
    ↓ ../execution/guarded_executor.py + quintic_controller.py
로봇 실행 및 data/generated/tracking_result.csv 기록
```

경로 입력·출력 위치, 높이, 속도 등은
[config/path_solver_mizb.yaml](../../config/path_solver_mizb.yaml)에서 설정한다.
이 문서의 링크는 현재 기본 설정의 데이터 폴더를 가리킨다.

`generated/` 파일은 해당 생성·실행 단계를 수행한 뒤 생긴다. 남아 있는 파일은
이전 실행 결과일 수 있으므로 현재 장면에서 실행할 때는 안경 snapshot에 맞춰
Cartesian 경로와 IK 검증 결과를 다시 생성한다. 전체 실행 순서는
[프로젝트 README](../../README.md#전체-실행)를 참고한다.
