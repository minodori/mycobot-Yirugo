# 로컬 USB 직결 제어 검토 — RPi 없이 노트북에서 myCobot 280 Arduino 구동 (2026-07-31)

현재 실물 제어는 **노트북(시뮬레이션·계획) → 네트워크 → RPi(`sync_plan`) → USB
→ 로봇** 구조다(`docs/Moveit2 setup record.md` 7장). 이 문서는 그 중간의 RPi를
빼고 **노트북에 로봇을 USB로 직결**해서 `sync_plan`을 로컬에서 돌리는 방안을
검토·정리한 것이다. 대상 하드웨어는 지금 쓰는 JetCobot(RPi 내장)이 아니라
**myCobot 280 for Arduino**(USB 시리얼로 호스트 PC에 직결하는 버전).

**상태: 설계 검토만 완료, 실물 미검증.** 아래 "테스트 절차"는 하드웨어가
준비되면 그대로 따라가면 되는 순서다.

---

## 1. 결론

**가능하고, 구조적으로 더 단순해진다.**

근거:

1. `sync_plan`은 `/joint_states` 구독 + pyserial write가 전부다. RPi에 있어야
   할 이유가 "로봇이 Pi에 USB로 물려 있어서"였을 뿐, 노드 자체에 원격 요소가
   없다.
2. upstream에 이미 Arduino 버전용 릴레이가 있다 —
   `src/mycobot_ros2/mycobot_280/mycobot_280_moveit2_control/mycobot_280_moveit2_control/sync_plan_arduino.py`.
   관절 순서(`joint2_to_joint1`~`joint6output_to_joint6`), `MyCobot280` 클래스,
   baud 1000000까지 지금 쓰는 `rpi/sync_plan.py`와 동일하고, 포트만
   `/dev/ttyUSB*`/`/dev/ttyACM*` 자동 감지로 다르다. 즉 이 워크스페이스가
   해온 개조(_async, throttle, 동적 speed, 그리퍼/LED relay, 기동 안전 검사)를
   그대로 얹을 수 있는 같은 뼈대다.
3. `mycobot_280arduino_moveit2` 패키지(URDF/SRDF/joint_limits/ros2_control)도
   이미 vendored 되어 있고 로컬 `install/`에 빌드까지 되어 있다.

### 없어지는 것 (이득)

- 노트북↔RPi DDS 홉(WiFi) — `/joint_states` 도착 지터의 한 원인.
- `ROS_DOMAIN_ID` 맞추기, ssh + nohup + 로그 tail 절차
  (`docs/Moveit2 setup record.md` "실행 방법" 1~6번).
- **scp 배포와 Pi에서 직접 편집된 내용의 diff 절차** — `rpi/sync_plan.py`
  파일 헤더(13~14행)가 경고하는 "그냥 scp하면 Pi 쪽 수정이 사라짐" 문제가
  원천적으로 사라진다. 노드가 워크스페이스 안에서 colcon 빌드로 배포됨.

### 그대로 남는 것 (착각 금지)

- **`sync_plan`을 켜는 순간 실물이 시뮬 자세로 점프하는 위험** — 로컬로 옮겨도
  동일. `_verify_startup_sync`(기동 안전 검사)는 그대로 유효하고 그대로 필요하다.
- **시리얼 포트 이중 점유 금지** — 오히려 로컬에서 더 위험하다. 같은 노트북에서
  캘리브레이션 스크립트나 수동 teleop 스크립트를 무심코 띄우기 쉬움.
- 소프트 `/emergency_stop`이 하드웨어 e-stop이 아니라는 점.

---

## 2. 제조사 문서 근거 (2026-07-31 확인)

공식 문서: <https://docs.elephantrobotics.com/docs/mycobot-280-Arduino-en/>
(실제 페이지 slug는 `mycobot_280_ar_en`)

### 2.1 제조사가 안내하는 ROS2 경로 = 지금 워크스페이스 구조와 동일

[6.4.4 Moveit2 Introduction and Use](https://docs.elephantrobotics.com/docs/mycobot_280_ar_en/3-FunctionsAndApplications/6.developmentGuide/ROS/12.2-ROS2/12.2.5-Moveit2/):

```bash
ros2 launch mycobot_280arduino_moveit2 demo.launch.py
ros2 run mycobot_280_moveit2_control sync_plan_arduino
```

**두 패키지 모두 이미 이 워크스페이스에 vendored 되어 있고 빌드까지 되어 있다.**
즉 로컬 직결은 "제조사가 상정한 표준 경로"이고, 우리가 RPi에서 하던 것은 같은
구조를 원격으로 늘려놓은 것이었을 뿐이다.

### 2.2 포트/보드레이트 — 제조사 문서 안에서도 값이 갈린다

| 출처 | 값 |
|---|---|
| [6.2.1 Environment construction](https://docs.elephantrobotics.com/docs/mycobot_280_ar_en/3-FunctionsAndApplications/6.developmentGuide/python/1_download.html) (Python 장) | `MyCobot280("COM3", 115200)` — "baud rate is 115200 by default, **and some boards are 1000000**" |
| 6.4.4 Moveit2 장 (ROS2) | "default serial port name ... is `/dev/ttyUSB0` and the **baud rate is 1000000**", 일부 기종은 `/dev/ttyACM0` |

문서끼리 상충하므로 **실물에서 확정해야 한다**(6-A 절차). 제조사도 같은 페이지에
"프로그램은 정상 동작하는데 팔이 반응하지 않으면 보드레이트를 확인하라"고
적어두었다 — 우리 프로젝트가 과거에 겪은 실패 모드와 정확히 동일하다
(`docs/Moveit2 setup record.md`, 포트/보드레이트 불일치로 **에러 없이 무반응**).

### 2.3 펌웨어 — RPi 구조에는 없던 신규 운영 절차

같은 Environment construction 페이지:

- 베이스(Basic)에 **minirobot을 굽고, 그 메뉴에서 `Transponder` 기능을 선택**해야
  한다("used to receive and forward the instructions sent by the Basic at the
  bottom to perform the target action").
- 끝단 Atom에는 **atomMain이 공장 출하 시 이미 구워져 있다**(직접 구울 필요 없음).
- MoveIt2 장은 Atom 펌웨어 **6.5 이상**, pymycobot **3.5.3 이상**을 요구한다
  (우리는 4.0.5 이상을 쓸 것이므로 후자는 충족).

⚠️ **Transponder 선택이 전원 인가 때마다 필요한지 확인할 것.** JetCobot/RPi
구조에는 이 단계가 아예 없었으므로, "로봇을 켰는데 아무 반응이 없다"의 새로운
1순위 원인이 된다. 6-A에서 확인해 결과를 이 문서에 적을 것.

### 2.4 연결 위치 / LED / 그리퍼

[하드웨어 인터페이스 문서](https://docs.elephantrobotics.com/docs/mycobot-280-Arduino-en/3-FunctionsAndApplications/5.BasicFunction/5.3-FirmwareFunctionDescription/RoboticArmElectricalInterface.html):

- PC 제어는 **베이스(Basic 보드) 쪽 시리얼**로 연결한다. 끝단에도 Type-C가 있지만
  그건 "PC 통신 및 펌웨어 업데이트용"이다 — 헷갈리지 말 것.
- 끝단 Atom에 **5×5 RGB LED(G27)** 와 버튼(G39)이 있다 → `set_color`로 하는
  단계 표시 기능이 그대로 살아 있을 가능성이 높다. 제조사 Python 예제도
  `mc.set_color(0, 0, 255)`를 그대로 쓴다.
- 끝단에 **그리퍼용 서보 인터페이스**가 있고, 액세서리 문서에 Adaptive/Parallel/
  Flexible 그리퍼가 모두 있다 → `GRIPPER_TYPE=1`(adaptive)은 **어떤 그리퍼를
  실제로 다느냐**의 문제이지 지원 여부의 문제가 아니다.
- 전원은 12V 5A DC 어댑터.

---

## 3. 현재 로컬 환경 실측 (2026-07-31)

| 항목 | 상태 |
|---|---|
| ROS 배포판 | jazzy (`/opt/ros/jazzy`) — RPi와 동일 |
| `pymycobot` | **미설치** (`ModuleNotFoundError`) |
| `dialout` 그룹 | 이미 소속 — 추가 권한 작업 불필요 |
| 시리얼 장치 | 확인 시점에 `/dev/ttyUSB*`, `/dev/ttyACM*` 없음 (로봇 미연결) |
| vendored 패키지 | `mycobot_280_arduino`, `mycobot_280arduino_moveit2` 빌드됨 |

---

## 4. 필요한 변경 (체크리스트)

### 4.1 `pymycobot` 로컬 설치

```bash
pip install --break-system-packages 'pymycobot>=4.0.5'
python3 -c "import pymycobot; print(pymycobot.__version__)"
```

Ubuntu 24.04는 externally-managed 환경이라 `--break-system-packages`(또는 venv)
가 필요하다. venv를 쓸 경우 `ros2 run`으로 뜨는 노드가 그 venv를 보도록
`PYTHONPATH`를 맞춰야 하므로, 단순함을 위해 user-site 설치를 권장.

⚠️ **4.0.5 고정 이유**: `rpi/sync_plan.py` 595~641행이 의존하는
`send_angles(..., _async=True)`의 write-only 분기를 RPi에서 소스로 직접 확인한
버전이 4.0.5다. 더 높은 버전을 깔면 설치 후 `_mesg`에 `if _async:` 분기가
남아 있는지 먼저 확인할 것 — 사라지면 **에러 없이** 호출당 1.5초 블로킹이
돌아오고 relay가 0.64Hz로 떨어진다(과거 "구간 단위로 뚝뚝 끊김"의 정체).

```bash
# 설치 후 확인
python3 - <<'EOF'
import inspect, pymycobot
from pymycobot import MyCobot280
src = inspect.getsource(MyCobot280._mesg if hasattr(MyCobot280,'_mesg') else MyCobot280)
print('_async 분기 존재:', '_async' in src)
print(inspect.signature(MyCobot280.send_angles))
EOF
```

### 4.2 포트/보드레이트 파라미터화

`rpi/sync_plan.py` 242~244행이 `/dev/ttyJETCOBOT` + 1000000으로 하드코딩되어
있다. 이걸 ROS 파라미터(`port`, `baudrate`)로 빼고 기본값을 Arduino 기준으로
둔다.

⚠️ **보드레이트는 실물에서 확인해야 한다** — 제조사 문서 자체가 갈린다(2.2절:
ROS2 장 1000000 vs Python 장 115200). **틀리면 에러 없이 조용히 무반응**이다 —
`docs/Moveit2 setup record.md`에 이 실패 사례가 기록돼 있다(포트/보드레이트
불일치로 명령이 조용히 무시됨). 1000000 → 안 되면 115200 순으로 시도.

### 4.3 udev 별칭 고정

D435도 USB에 물리므로 `/dev/ttyUSB0` 번호가 흔들릴 수 있다. RPi에서 쓰던
`/dev/ttyJETCOBOT`처럼 별칭을 만든다.

```bash
# 연결 후 식별자 확인
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
# /etc/udev/rules.d/99-mycobot.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="ttyMYCOBOT"
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4.4 노드 이식

`rpi/sync_plan.py`를 `mycobot_280_pick` 패키지의 엔트리포인트로 옮긴다
(`setup.py`에 `sync_plan = mycobot_280_pick.sync_plan:main`). 그러면
`colcon build` 한 번으로 배포가 끝나고, 원하면
`src/mycobot_280_pick/launch/pick_pipeline.launch.py`에 넣어 파이프라인과 함께
띄울 수도 있다.

**단, launch에 넣는 것은 신중히** — `sync_plan` 기동 = 실물 이동 명령이므로,
파이프라인 launch에 섞으면 "실행하면 팔이 움직인다"가 기본 동작이 된다.
초기에는 별도 터미널에서 수동으로 띄우는 지금 방식을 유지하고, 기동 안전
검사(`_verify_startup_sync`)를 실물에서 충분히 확인한 뒤에 통합할 것.

이식 시 `rpi/` 디렉터리와 헤더 주석(배포 경로·scp 경고)은 구조가 바뀌므로
같이 갱신해야 한다.

---

## 5. 재검증이 필요한 항목 (하드웨어가 바뀌는 것)

배선만 바뀌는 게 아니라 **로봇 개체와 펌웨어가 바뀐다.** 지금 코드의 상수 중
상당수는 JetCobot 실물에서 실측해 얻은 값이다.

| 항목 | 위치 | 왜 재검증인가 |
|---|---|---|
| J6 부호 반전 | `sync_plan.py:568-570` | 그 개체에서 실측한 회전 방향. 새 로봇에서 반대일 수 있음 |
| 그리퍼 타입/동작 | `GRIPPER_TYPE=1`(adaptive), `set_gripper_state` | 제조사 문서상 지원됨(2.4절) — **어떤 그리퍼를 실제로 장착하느냐**가 변수. adaptive가 아니면 `GRIPPER_TYPE`/개폐 위치 상수를 바꿔야 함 |
| LED 단계 표시 | `_set_color_async` (`_mesg(SET_COLOR, ...)`) | Atom 5×5 RGB가 있으므로 지원됨(2.4절). 단 우리 코드는 공개 `set_color`가 아니라 내부 `_mesg`를 write-only로 부르므로 그 경로가 동작하는지만 확인 |
| **Transponder 모드** | 하드웨어 조작(베이스 화면) | **신규 절차** — 2.3절. 전원 인가마다 필요한지 확인하고 결과를 여기 적을 것 |
| `fresh_mode` | `sync_plan.py:268-271` | upstream Arduino 스크립트도 쓰므로 지원 가능성 높음. 기동 로그로 즉시 확인됨 |
| `SPEED_GAIN_K=1.7` 등 속도 상수 | `sync_plan.py:107-214` | 그 서보 + 그 링크 지연 조건의 실측값. 공식이 각속도 기반이라 큰 틀은 유지되지만 체감 재확인 필요 |
| 관절 영점/한계 | `mycobot_280arduino_moveit2/config` | Arduino용 moveit2 설정이 지금 쓰는 것과 다르면 URDF 계열 전환 여부 판단 필요 |
| look pose | `docs/look_pose.md` | 카메라 마운트가 동일하지 않으면 다시 잡아야 함 |

### 새로 생기는 변수: CPU 경합

YOLO + D435 + MoveIt + relay가 모두 노트북 한 대에 몰린다. relay는 별도
프로세스라 치명적이진 않지만, 20Hz 전송 주기가 흔들리면 그게 그대로 실물
떨림이 된다. `sync_plan` 로그의 `dt=` 값(목표 0.05~0.06s)으로 확인할 것 —
이 지표는 이미 로그에 찍고 있다.

반대로 **네트워크 홉이 사라져 `/joint_states` 도착 지터는 줄어든다.** 두 효과가
반대 방향이라, 실물 체감이 좋아질지 나빠질지는 해봐야 안다.

---

## 6. 테스트 절차 (하드웨어 준비 후)

⚠️ 모든 단계에서 `docs/real_robot_tier_verification.md`의 "절대 원칙"이 그대로
적용된다. 특히 **로봇 주변 사람/물건 확인**과 **시리얼 이중 점유 금지**.

### 6-A. 통신만 확인 (ROS 없이, 팔 안 움직임)

**선행**: 베이스 화면에서 `Transponder`가 선택된 상태여야 한다(2.3절). 이게
안 되어 있으면 아래가 전부 무반응으로 나오는데, 증상이 보드레이트 불일치와
구별되지 않는다 — 먼저 확인할 것.

로봇 연결 → 포트 확인 → `get_angles()`만 읽어본다. 여기서 baud를 확정한다.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
python3 - <<'EOF'
from pymycobot import MyCobot280
import time
for baud in (1000000, 115200):
    try:
        mc = MyCobot280('/dev/ttyUSB0', baud); time.sleep(0.1)
        print(baud, '->', mc.get_angles(), 'fresh_mode:', mc.get_fresh_mode())
    except Exception as e:
        print(baud, '-> 실패', e)
EOF
```

- 6개짜리 리스트가 나오면 그 baud가 정답. `-1`이나 `None`이면 실패
  (`pymycobot`은 통신 실패 시 `-1`을 반환한다 — `_verify_startup_sync` 주석 참고).
- 여기서 `get_fresh_mode()`가 값을 주면 5장 표의 `fresh_mode` 의문도 해소.

### 6-B. 그리퍼/LED 지원 여부 (팔 안 움직임)

```python
mc.set_gripper_state(0, 50, 1); time.sleep(1); mc.set_gripper_state(1, 50, 1)
mc.set_color(0, 0, 255)
```

각각 실제로 동작하는지 눈으로 확인. 안 되면 5장 표의 해당 항목을 코드에서
비활성화(파라미터로 끄기)해야 한다.

### 6-C. 수동 단발 이동 (팔 움직임 — 첫 실물 동작)

현재 각도에서 한 관절만 5도 움직여 보고, **방향이 맞는지**(특히 J6) 확인한다.
이게 5장 표의 "J6 부호 반전" 재검증이다.

### 6-D. relay 기동 (시뮬 → 실물)

1. 로컬에서 `demo_octomap.launch.py`(또는 `pick_pipeline.launch.py`)를 띄워
   `/joint_states`가 발행되게 한다.
2. RViz에서 시뮬을 **실물의 현재 자세**로 먼저 맞춘다.
3. 별도 터미널에서 `sync_plan`을 띄운다. 기동 안전 검사가
   "최대 차이 N도" 로그를 내는지 확인 — 여기서 막히면 검사가 제대로 도는 것이다
   (이게 정상 동작이다).
4. 통과하면 `data_list: [...] | speed=NN (dmax=..., dt=..., ...deg/s)` 로그가
   흐른다. `dt`가 0.05~0.06s 근처인지 확인(5장 CPU 경합 항목).

### 6-E. 부드러움 체감 + 상수 재튜닝

`docs/SPEED_TUNING_HANDOFF.md` 절차 그대로. 로컬 직결에서는 지연 특성이 달라졌
으므로 `SPEED_GAIN_K`는 1.7을 출발점으로 두고 다시 본다(떨리면 1.5, 굼뜨면 1.9).

### 6-F. 전체 파이프라인

`docs/real_robot_tier_verification.md` 순서로 검출 → 목표 → 파지 사이클 검증.

---

## 7. 롤백

RPi 구조는 그대로 남아 있으므로(코드도 `rpi/sync_plan.py` 정본 그대로),
로컬 직결이 잘 안 되면 로봇을 Pi에 다시 물리고 기존 절차로 돌아가면 된다.
**두 구조를 동시에 쓰지는 말 것** — 같은 `/joint_states`를 두 relay가 각각
다른 로봇에 흘리게 된다.

---

## 8. 미확정 사항

- **실제 baud** — 제조사 문서가 115200(Python 장)과 1000000(ROS2 장)으로 갈린다
  (2.2절). 6-A에서 확정.
- **포트 종류** — 기본은 `/dev/ttyUSB0`, 일부 기종 `/dev/ttyACM0`. 6-A에서 확정.
- **Transponder 선택이 매 부팅마다 필요한지** — 2.3절. 6-A에서 확정.
- 장착할 그리퍼 종류(adaptive/parallel/flexible)와 그에 맞는 `GRIPPER_TYPE` —
  지원 여부는 확인됐고 기종만 미정. 6-B에서 확정.
- Arduino용 moveit2 설정(`mycobot_280arduino_moveit2`)으로 갈아타야 하는지,
  지금 쓰는 `mycobot_280_moveit2` 설정 그대로 되는지 — 팔 기구학이 동일하면
  후자로 충분할 가능성이 높지만 URDF diff로 확인 필요.
- 카메라 마운트가 동일한지 → 다르면 핸드-아이 캘리브레이션
  (`docs/handeye_calibration.md`)과 look pose를 다시 잡아야 함.
